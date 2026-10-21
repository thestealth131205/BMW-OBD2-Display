#include "can_obd2.h"
#include "mcp2515.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "service_funcs.h"

static const char *TAG = "CAN_OBD2";

// --- BMW E90 PT-CAN Broadcast-IDs (Platzhalter aus dem C6-Projekt, muessen am
//     Fahrzeug per CAN-Sniffer verifiziert werden) ---
#define ID_RPM        0x0AA   // Byte2-3 little-endian * 0.25 = U/min
#define ID_GFORCE     0x0C4   // Byte0/1 int8 *0.01 = g, Byte2-3 LE *0.1 = km/h
#define ID_WATER      0x1D0   // Byte0 - 40 = degC
#define ID_THROTTLE   0x1F0   // Byte0 linear 0..255 -> 0..100 %

// OBD2 / UDS
#define ID_OBD2_FUNC  0x7DF   // funktionale Broadcast-Adresse
#define ID_OBD2_RESP  0x7E8   // Antwort-ID des Motorsteuergeraets
#define ID_KOMBI      0x611   // Kombiinstrument (CBS-Reset, UDS 0x31)

#define EXTRA_POLL_INTERVAL_MS 250  // Rundlauf-Intervall je PID (4 PIDs -> 1s/Zyklus, wie zuvor nur fuer 0x42)

static volatile float s_speed_kmh    = 0.0f;
static volatile float s_rpm          = 0.0f;
static volatile float s_water_temp   = 0.0f;
static volatile float s_throttle_pct = 0.0f;
static volatile float s_gforce_x     = 0.0f;
static volatile float s_gforce_y     = 0.0f;
static volatile bool  s_online       = false;
static volatile float s_obd2_bat_voltage = 0.0f;

// --- Sensoren-Screen: Lambda (Ratio+Spannung) Sensor 1/2, Ansaugkruemmerdruck
// (MAP) - siehe can_obd2.h fuer die Annahmen/Einschraenkungen dazu. Werden nur
// abgefragt, solange der Sensoren-Screen sichtbar ist (CAN_OBD2_set_sensors_active). ---
static volatile bool  s_sensors_active  = false;
static volatile float s_lambda1_ratio   = 0.0f;
static volatile float s_lambda1_voltage = 0.0f;
static volatile float s_lambda2_ratio   = 0.0f;
static volatile float s_lambda2_voltage = 0.0f;
static volatile float s_intake_pressure = 0.0f;
static volatile float s_baro_pressure   = -1.0f;   // kPa, -1 = noch keine Antwort auf PID 0x33
static volatile bool  s_mil_on          = false;   // PID 0x01, Byte A Bit 7
static volatile float s_oil_temp        = -999.0f; // Service 0x22 PID 0x4402, -999 = noch keine Antwort

#define MAX_DTC 8
static volatile int s_dtc_count = -1;   // -1 = noch nicht ausgelesen
static char s_dtc_codes[MAX_DTC][6];    // z.B. "P0301"

// Dekodiert einen 2-Byte-DTC (OBD2 Mode 03/07, ISO 15031) in Textform,
// z.B. Byte1=0x03,Byte2=0x01 -> "P0301". Liefert false bei Fuell-Bytes (0x00 0x00).
static bool decode_dtc_bytes(uint8_t b1, uint8_t b2, char out[6])
{
    if (b1 == 0 && b2 == 0) return false;
    static const char sys_chars[4] = {'P', 'C', 'B', 'U'};
    char sys = sys_chars[(b1 >> 6) & 0x3];
    int d1 = (b1 >> 4) & 0x3;
    int d2 = b1 & 0x0F;
    int d3 = (b2 >> 4) & 0x0F;
    int d4 = b2 & 0x0F;
    snprintf(out, 6, "%c%d%X%X%X", sys, d1, d2, d3, d4);
    return true;
}

static void decode_frame(const mcp2515_frame_t *f)
{
    switch (f->id) {
    case ID_OBD2_RESP:
        if (f->dlc >= 5 && f->data[1] == 0x41 && f->data[2] == 0x42) {
            // Mode 01, PID 0x42 (Steuergeraete-Spannung): A/B in mV
            uint16_t raw = ((uint16_t)f->data[3] << 8) | f->data[4];
            s_obd2_bat_voltage = raw / 1000.0f;
        } else if (f->dlc >= 7 && f->data[1] == 0x41 && f->data[2] == 0x24) {
            // Mode 01, PID 0x24 (O2-Sensor 1, Lambda+Spannung): A,B,C,D
            uint16_t ratio_raw = ((uint16_t)f->data[3] << 8) | f->data[4];
            uint16_t volt_raw  = ((uint16_t)f->data[5] << 8) | f->data[6];
            s_lambda1_ratio   = ratio_raw * (2.0f / 65536.0f);
            s_lambda1_voltage = volt_raw  * (8.0f / 65536.0f);
        } else if (f->dlc >= 7 && f->data[1] == 0x41 && f->data[2] == 0x25) {
            // Mode 01, PID 0x25 (O2-Sensor 2, Lambda+Spannung)
            uint16_t ratio_raw = ((uint16_t)f->data[3] << 8) | f->data[4];
            uint16_t volt_raw  = ((uint16_t)f->data[5] << 8) | f->data[6];
            s_lambda2_ratio   = ratio_raw * (2.0f / 65536.0f);
            s_lambda2_voltage = volt_raw  * (8.0f / 65536.0f);
        } else if (f->dlc >= 4 && f->data[1] == 0x41 && f->data[2] == 0x0B) {
            // Mode 01, PID 0x0B (Ansaugkruemmerdruck/MAP): A in kPa
            s_intake_pressure = (float)f->data[3];
        } else if (f->dlc >= 4 && f->data[1] == 0x41 && f->data[2] == 0x33) {
            // Mode 01, PID 0x33 (absoluter Luftdruck/Baro): A in kPa
            s_baro_pressure = (float)f->data[3];
        } else if (f->dlc >= 4 && f->data[1] == 0x41 && f->data[2] == 0x01) {
            // Mode 01, PID 0x01 (Monitor Status seit DTC-Loeschen): Byte A
            // Bit 7 = MIL an/aus, Bits 0-6 = Anzahl gespeicherter DTCs
            // (hier nicht ausgewertet, dafuer gibt es bereits Mode 03).
            s_mil_on = (f->data[3] & 0x80) != 0;
        } else if (f->dlc >= 5 && f->data[1] == 0x62 && f->data[2] == 0x44 && f->data[3] == 0x02) {
            // Service 0x22 (ReadDataByIdentifier), BMW-PID 0x4402 (Oeltemperatur):
            // Rohwert*0.75-48 = Grad C, siehe can_obd2.h.
            s_oil_temp = (float)f->data[4] * 0.75f - 48.0f;
        } else if (f->dlc >= 2 && f->data[1] == 0x43) {
            // Mode 03, positive Antwort: Fehlercodes ab Byte 2, je 2 Byte
            int n = 0;
            for (int i = 2; i + 1 < f->dlc && n < MAX_DTC; i += 2) {
                if (decode_dtc_bytes(f->data[i], f->data[i + 1], s_dtc_codes[n])) {
                    n++;
                }
            }
            s_dtc_count = n;
        } else if (f->dlc >= 2 && f->data[1] == 0x44) {
            // Mode 04, positive Antwort: Fehlercodes geloescht
            s_dtc_count = 0;
        }
        break;
    case ID_RPM:
        if (f->dlc >= 4) {
            uint16_t raw = (uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8);
            s_rpm = raw * 0.25f;
        }
        break;
    case ID_GFORCE:
        if (f->dlc >= 4) {
            s_gforce_x = (int8_t)f->data[0] * 0.01f;
            s_gforce_y = (int8_t)f->data[1] * 0.01f;
            uint16_t raw = (uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8);
            s_speed_kmh = raw * 0.1f;
        }
        break;
    case ID_WATER:
        if (f->dlc >= 1) {
            s_water_temp = (float)f->data[0] - 40.0f;
        }
        break;
    case ID_THROTTLE:
        if (f->dlc >= 1) {
            s_throttle_pct = f->data[0] * (100.0f / 255.0f);
        }
        break;
    default:
        break;
    }
}

typedef struct {
    uint8_t len;      // ISO-TP Single-Frame-Laenge (Service-Byte + Datenbytes)
    uint8_t data[3];  // Service, PID-High[, PID-Low]
} obd2_extra_req_t;

static void can_task(void *arg)
{
    mcp2515_frame_t f;
    uint32_t last_extra_poll = 0;
    // Rundlauf: Batteriespannung (0x42) plus die Sensoren-Screen-PIDs
    // (Lambda 1/2, MAP, Baro, MIL-Status, BMW-Oeltemperatur), je 250ms ->
    // kompletter Zyklus weiterhin alle 1s wie zuvor nur fuer die
    // Batteriespannung allein.
    static const obd2_extra_req_t extra_reqs[] = {
        {2, {0x01, 0x42}},       // Batteriespannung (immer im Rundlauf)
        {2, {0x01, 0x24}},       // Lambda 1 (Ratio+Spannung)
        {2, {0x01, 0x25}},       // Lambda 2 (Ratio+Spannung)
        {2, {0x01, 0x0B}},       // Ansaugkruemmerdruck (MAP)
        {2, {0x01, 0x33}},       // absoluter Luftdruck (Baro, fuer Ladedruck)
        {2, {0x01, 0x01}},       // Monitor-Status (MIL)
        {3, {0x22, 0x44, 0x02}}, // BMW-Oeltemperatur (Service 0x22)
    };
    int extra_idx = 0;
    while (1) {
        int drained = 0;
        while (mcp2515_receive(&f) && drained < 16) {
            s_online = true;
            decode_frame(&f);
            drained++;
        }

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_extra_poll >= EXTRA_POLL_INTERVAL_MS) {
            last_extra_poll = now;
            // Ohne sichtbaren Sensoren-Screen nur die Batteriespannung (0x42)
            // abfragen - die uebrigen Werte kosten sonst unnoetig Buszeit,
            // obwohl niemand hinschaut.
            const obd2_extra_req_t *r = s_sensors_active ? &extra_reqs[extra_idx] : &extra_reqs[0];
            uint8_t req[8] = {r->len, 0, 0, 0, 0, 0, 0, 0};
            memcpy(&req[1], r->data, r->len);
            mcp2515_send(ID_OBD2_FUNC, req, 8);
            extra_idx = s_sensors_active
                ? (extra_idx + 1) % (int)(sizeof(extra_reqs) / sizeof(extra_reqs[0]))
                : 0;
        }

        vTaskDelay(pdMS_TO_TICKS(drained ? 2 : 10));
    }
}

bool CAN_OBD2_Init(void)
{
    if (mcp2515_init() != ESP_OK) {
        ESP_LOGW(TAG, "MCP2515 nicht initialisiert - CAN offline, Platzhalterwerte");
        return false;
    }
    xTaskCreatePinnedToCore(can_task, "can_obd2", 4096, NULL, 4, NULL, 0);
    return true;
}

bool  CAN_OBD2_online(void)       { return s_online; }
float CAN_OBD2_speed_kmh(void)    { return s_speed_kmh; }
float CAN_OBD2_rpm(void)          { return s_rpm; }
float CAN_OBD2_water_temp(void)   { return s_water_temp; }
float CAN_OBD2_throttle_pct(void) { return s_throttle_pct; }
float CAN_OBD2_gforce_x(void)     { return s_gforce_x; }
float CAN_OBD2_gforce_y(void)     { return s_gforce_y; }

void CAN_OBD2_read_dtc(void)
{
    uint8_t req[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    mcp2515_send(ID_OBD2_FUNC, req, 8);
}

void CAN_OBD2_clear_dtc(void)
{
    uint8_t req[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    mcp2515_send(ID_OBD2_FUNC, req, 8);
    s_dtc_count = 0;   // optimistisch, wird ggf. per 0x44-Antwort bestaetigt
}

void CAN_OBD2_reset_service_oil(void)
{
    // UDS Routine Control (0x31) an Kombiinstrument, Service-Typ 0x01 = Motoroel
    uint8_t req[8] = {0x04, 0x31, 0x01, 0xFF, 0x01, 0x00, 0x00, 0x00};
    mcp2515_send(ID_KOMBI, req, 8);
}

bool CAN_OBD2_service_func(int idx)
{
    if (idx < 0 || idx >= SERVICE_FUNC_COUNT || SERVICE_FUNCS[idx].len == 0) return false;
    const service_func_t *f = &SERVICE_FUNCS[idx];
    uint8_t req[8] = {0};
    req[0] = f->len; // ISO-TP Single Frame
    memcpy(&req[1], f->data, f->len);
    return mcp2515_send(f->can_id, req, 8) == ESP_OK;
}

int CAN_OBD2_dtc_count(void) { return s_dtc_count; }

const char *CAN_OBD2_dtc_code(int idx)
{
    if (idx < 0 || idx >= s_dtc_count || idx >= MAX_DTC) return NULL;
    return s_dtc_codes[idx];
}

float CAN_OBD2_bat_voltage(void) { return s_obd2_bat_voltage; }

void CAN_OBD2_set_sensors_active(bool active) { s_sensors_active = active; }

float CAN_OBD2_lambda1_ratio(void)   { return s_lambda1_ratio; }
float CAN_OBD2_lambda1_voltage(void) { return s_lambda1_voltage; }
float CAN_OBD2_lambda2_ratio(void)   { return s_lambda2_ratio; }
float CAN_OBD2_lambda2_voltage(void) { return s_lambda2_voltage; }
float CAN_OBD2_intake_pressure(void) { return s_intake_pressure; }

float CAN_OBD2_boost_pressure(void)
{
    if (s_baro_pressure < 0.0f) return -1.0f; // Baro (PID 0x33) noch nicht beantwortet
    return s_intake_pressure - s_baro_pressure;
}

float CAN_OBD2_cam_intake_pos(void)  { return -1.0f; }
float CAN_OBD2_cam_exhaust_pos(void) { return -1.0f; }

bool  CAN_OBD2_mil_on(void)   { return s_mil_on; }
float CAN_OBD2_oil_temp(void) { return s_oil_temp; }
