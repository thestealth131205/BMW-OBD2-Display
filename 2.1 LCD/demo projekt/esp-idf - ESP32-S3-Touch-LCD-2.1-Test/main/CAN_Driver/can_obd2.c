#include "can_obd2.h"
#include "mcp2515.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "CAN_OBD2";

// --- BMW E90 PT-CAN Broadcast-IDs (Platzhalter aus dem C6-Projekt, muessen am
//     Fahrzeug per CAN-Sniffer verifiziert werden) ---
#define ID_RPM        0x0AA   // Byte2-3 little-endian * 0.25 = U/min
#define ID_GFORCE     0x0C4   // Byte0/1 int8 *0.01 = g, Byte2-3 LE *0.1 = km/h
#define ID_WATER      0x1D0   // Byte0 - 40 = degC
#define ID_THROTTLE   0x1F0   // Byte0 linear 0..255 -> 0..100 %

// OBD2 / UDS
#define ID_OBD2_FUNC  0x7DF   // funktionale Broadcast-Adresse
#define ID_KOMBI      0x611   // Kombiinstrument (CBS-Reset, UDS 0x31)

static volatile float s_speed_kmh    = 0.0f;
static volatile float s_rpm          = 0.0f;
static volatile float s_water_temp   = 0.0f;
static volatile float s_throttle_pct = 0.0f;
static volatile float s_gforce_x     = 0.0f;
static volatile float s_gforce_y     = 0.0f;
static volatile bool  s_online       = false;

static void decode_frame(const mcp2515_frame_t *f)
{
    switch (f->id) {
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

static void can_task(void *arg)
{
    mcp2515_frame_t f;
    while (1) {
        int drained = 0;
        while (mcp2515_receive(&f) && drained < 16) {
            s_online = true;
            decode_frame(&f);
            drained++;
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
}

void CAN_OBD2_reset_service_oil(void)
{
    // UDS Routine Control (0x31) an Kombiinstrument, Service-Typ 0x01 = Motoroel
    uint8_t req[8] = {0x04, 0x31, 0x01, 0xFF, 0x01, 0x00, 0x00, 0x00};
    mcp2515_send(ID_KOMBI, req, 8);
}
