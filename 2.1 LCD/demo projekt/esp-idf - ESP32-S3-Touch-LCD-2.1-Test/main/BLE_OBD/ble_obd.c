#include "ble_obd.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"

static const char *TAG = "BLE_OBD";

// Werbenamen-Teilstrings, an denen ein ELM327-BLE-Adapter (z.B. Veepeak
// OBDCheck BLE) erkannt wird - je nach Firmware variiert der genaue Name,
// daher mehrere Kandidaten (Case-insensitive-Teilstring-Suche).
static const char *const OBD_NAME_HINTS[] = {"OBD", "VEEPEAK", "VLINK", "ELM327"};
#define NUM_OBD_NAME_HINTS (sizeof(OBD_NAME_HINTS) / sizeof(OBD_NAME_HINTS[0]))

// BLE-Pairing-Passwoerter, die der Reihe nach probiert werden, falls das
// Geraet beim Verbinden eine PIN verlangt (ESP_GAP_BLE_PASSKEY_REQ_EVT).
// Die meisten guenstigen ELM327-BLE-Klone verlangen gar kein BLE-Pairing -
// dieser Mechanismus greift nur, falls doch eine PIN angefordert wird.
static const uint32_t OBD_PASSKEYS[] = {1234, 5678, 0000};
#define NUM_OBD_PASSKEYS (sizeof(OBD_PASSKEYS) / sizeof(OBD_PASSKEYS[0]))
static int s_passkey_idx = 0;

#define BLE_OBD_APP_ID 0x56
#define MAX_BLE_OBD_SERVICES 16
#define MAX_DTC 8

typedef enum {
    BLE_OBD_REQ_READ_DTC,
    BLE_OBD_REQ_CLEAR_DTC,
    BLE_OBD_REQ_SERVICE_RESET,
} ble_obd_request_t;

typedef struct {
    uint16_t start_handle;
    uint16_t end_handle;
} ble_obd_service_t;

// --- Bluedroid-Zustand ---
static esp_gatt_if_t s_gattc_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id = 0;
static esp_bd_addr_t s_remote_bda = {0};
static volatile bool s_connecting = false;
static volatile bool s_connected = false;
static volatile bool s_notify_ready = false;

static ble_obd_service_t s_services[MAX_BLE_OBD_SERVICES];
static int s_service_count = 0;
static uint16_t s_rx_handle = 0;
static uint16_t s_tx_handle = 0;
static uint16_t s_found_service_start = 0;
static uint16_t s_found_service_end = 0;

static esp_ble_scan_params_t s_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,
};

// --- Notify-Antwortpuffer: ELM327-Antworten enden immer mit dem Prompt
// '>', werden aber ggf. auf mehrere BLE-Notify-Pakete (~20 Byte MTU)
// aufgeteilt - deshalb hier akkumulieren, bis '>' auftaucht. ---
static char s_notify_acc[256];
static int s_notify_acc_len = 0;
static char s_resp_buf[256];
static SemaphoreHandle_t s_resp_ready;
static SemaphoreHandle_t s_session_ready;
static QueueHandle_t s_request_queue;

// --- Live-Werte ---
static volatile bool  s_online          = false;
static volatile float s_speed_kmh       = 0.0f;
static volatile float s_rpm             = 0.0f;
static volatile float s_water_temp      = 0.0f;
static volatile float s_throttle_pct    = 0.0f;
static volatile float s_bat_voltage     = 0.0f;

static volatile int s_dtc_count = -1;
static char s_dtc_codes[MAX_DTC][6];

// Dekodiert einen 2-Byte-DTC (OBD2 Mode 03/07, ISO 15031) in Textform,
// z.B. Byte1=0x03,Byte2=0x01 -> "P0301". Identische Logik wie im
// MCP2515-Pfad (can_obd2.c), hier dupliziert, da beide Module unabhaengige,
// austauschbare Datenquellen sind.
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

// Case-insensitive Teilstring-Suche (strcasestr ist in der ESP-IDF-libc
// nicht garantiert verfuegbar, deshalb selbst implementiert).
static bool name_matches_obd_adapter(const char *name)
{
    if (!name || !name[0]) return false;
    char lower[32];
    size_t n = strlen(name);
    if (n >= sizeof(lower)) n = sizeof(lower) - 1;
    for (size_t i = 0; i < n; i++) lower[i] = (char)tolower((unsigned char)name[i]);
    lower[n] = '\0';

    for (size_t i = 0; i < NUM_OBD_NAME_HINTS; i++) {
        char hint[16];
        size_t hn = strlen(OBD_NAME_HINTS[i]);
        for (size_t j = 0; j < hn; j++) hint[j] = (char)tolower((unsigned char)OBD_NAME_HINTS[i][j]);
        hint[hn] = '\0';
        if (strstr(lower, hint)) return true;
    }
    return false;
}

// Extrahiert den vollstaendigen/kompletten Geraetenamen aus den
// BLE-Advertisement-Daten (identische Logik wie extract_device_name() in
// Wireless.c, hier unabhaengig dupliziert).
static bool extract_adv_name(const uint8_t *adv_data, uint8_t adv_data_len, char *out, size_t out_size)
{
    size_t offset = 0;
    while (offset < adv_data_len) {
        if (adv_data[offset] == 0) break;
        uint8_t length = adv_data[offset];
        if (length == 0 || offset + length > adv_data_len) break;
        uint8_t type = adv_data[offset + 1];
        if (type == ESP_BLE_AD_TYPE_NAME_CMPL || type == ESP_BLE_AD_TYPE_NAME_SHORT) {
            if (length > 1 && length - 1 < out_size) {
                memcpy(out, &adv_data[offset + 2], length - 1);
                out[length - 1] = '\0';
                return true;
            }
            return false;
        }
        offset += length + 1;
    }
    return false;
}

// Zerlegt eine ELM327-ASCII-Antwort in rohe Bytes: jedes gueltige
// 2-Hex-Zeichen-Token wird zu einem Byte, alles andere (z.B.
// "SEARCHING...", "NO DATA", Leerzeichen/CR/LF) wird ignoriert/uebersprungen.
static int hex_tokenize(const char *resp, uint8_t *out, int max_out)
{
    int n = 0;
    const char *p = resp;
    while (*p && n < max_out) {
        while (*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t') p++;
        if (!*p) break;
        bool is_tok = isxdigit((unsigned char)p[0]) && isxdigit((unsigned char)p[1]) &&
                      (p[2] == '\0' || p[2] == ' ' || p[2] == '\r' || p[2] == '\n' || p[2] == '\t');
        if (is_tok) {
            char tmp[3] = {p[0], p[1], '\0'};
            out[n++] = (uint8_t)strtol(tmp, NULL, 16);
            p += 2;
        } else {
            while (*p && *p != ' ' && *p != '\r' && *p != '\n' && *p != '\t') p++;
        }
    }
    return n;
}

static void ble_obd_start_scan(void)
{
    s_connecting = false;
    esp_ble_gap_start_scanning(0); // 0 = dauerhaft scannen, bis esp_ble_gap_stop_scanning()
}

static void ble_obd_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ble_obd_start_scan();
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT && !s_connecting && !s_connected) {
            char name[32];
            if (extract_adv_name(param->scan_rst.ble_adv, param->scan_rst.adv_data_len, name, sizeof(name)) &&
                name_matches_obd_adapter(name)) {
                ESP_LOGI(TAG, "OBD2-BLE-Adapter gefunden: %s - verbinde...", name);
                s_connecting = true;
                esp_ble_gap_stop_scanning();
                esp_ble_gattc_open(s_gattc_if, (uint8_t *)param->scan_rst.bda,
                                    param->scan_rst.ble_addr_type, true);
            }
        }
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(TAG, "BLE-Pairing verlangt PIN - probiere %u", (unsigned)OBD_PASSKEYS[s_passkey_idx]);
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, OBD_PASSKEYS[s_passkey_idx]);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGW(TAG, "BLE-Pairing mit PIN %u fehlgeschlagen", (unsigned)OBD_PASSKEYS[s_passkey_idx]);
            esp_ble_remove_bond_device(param->ble_security.auth_cmpl.bd_addr);
            s_passkey_idx = (s_passkey_idx + 1) % NUM_OBD_PASSKEYS;
            s_connected = false;
            s_notify_ready = false;
            ble_obd_start_scan();
        }
        break;

    default:
        break;
    }
}

// Sucht ueber alle entdeckten Services nach der ersten Charakteristik mit
// NOTIFY-Property (RX, ELM327 -> Display) und der ersten mit WRITE/
// WRITE_NR-Property (TX, Display -> ELM327). Generischer Ansatz statt
// fester UUIDs, da die UUIDs bei ELM327-BLE-Klonen je nach Firmware
// variieren (haeufig FFE0/FFE1 oder FFF0/FFF1/FFF2, aber nicht garantiert).
static void ble_obd_find_rx_tx_char(esp_gatt_if_t gattc_if, uint16_t conn_id)
{
    for (int i = 0; i < s_service_count; i++) {
        uint16_t count = 0;
        esp_ble_gattc_get_attr_count(gattc_if, conn_id, ESP_GATT_DB_CHARACTERISTIC,
                                      s_services[i].start_handle, s_services[i].end_handle,
                                      0, &count);
        if (count == 0) continue;

        esp_gattc_char_elem_t *chars = malloc(sizeof(esp_gattc_char_elem_t) * count);
        if (!chars) continue;
        uint16_t got = count;
        esp_gatt_status_t st = esp_ble_gattc_get_all_char(gattc_if, conn_id,
                                    s_services[i].start_handle, s_services[i].end_handle,
                                    chars, &got, 0);
        if (st == ESP_GATT_OK) {
            uint16_t rx = 0, tx = 0;
            for (int c = 0; c < got; c++) {
                if (!rx && (chars[c].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
                    rx = chars[c].char_handle;
                }
                if (!tx && (chars[c].properties & (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR))) {
                    tx = chars[c].char_handle;
                }
            }
            if (rx && tx) {
                s_rx_handle = rx;
                s_tx_handle = tx;
                s_found_service_start = s_services[i].start_handle;
                s_found_service_end = s_services[i].end_handle;
                free(chars);
                ESP_LOGI(TAG, "ELM327-Service gefunden (RX=0x%04X, TX=0x%04X)", rx, tx);
                esp_ble_gattc_register_for_notify(gattc_if, s_remote_bda, s_rx_handle);
                return;
            }
        }
        free(chars);
    }
    ESP_LOGW(TAG, "Kein Service mit Notify+Write-Charakteristik gefunden - Geraet nicht ELM327-kompatibel?");
}

static void ble_obd_enable_notify_cccd(esp_gatt_if_t gattc_if)
{
    uint16_t count = 0;
    esp_ble_gattc_get_attr_count(gattc_if, s_conn_id, ESP_GATT_DB_DESCRIPTOR,
                                  s_found_service_start, s_found_service_end,
                                  s_rx_handle, &count);
    if (count == 0) {
        // Manche Klone melden die CCCD nicht sauber ueber get_attr_count -
        // Notify wurde durch register_for_notify serverseitig ggf. trotzdem
        // schon aktiviert, daher optimistisch weitermachen.
        s_connected = true;
        s_notify_ready = true;
        xSemaphoreGive(s_session_ready);
        return;
    }

    esp_gattc_descr_elem_t *descrs = malloc(sizeof(esp_gattc_descr_elem_t) * count);
    if (!descrs) return;
    esp_bt_uuid_t cccd_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}};
    uint16_t got = count;
    esp_gatt_status_t st = esp_ble_gattc_get_descr_by_char_handle(gattc_if, s_conn_id, s_rx_handle,
                                                                   cccd_uuid, descrs, &got);
    if (st == ESP_GATT_OK && got > 0) {
        uint16_t notify_en = 1;
        esp_ble_gattc_write_char_descr(gattc_if, s_conn_id, descrs[0].handle,
                                        sizeof(notify_en), (uint8_t *)&notify_en,
                                        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    } else {
        s_connected = true;
        s_notify_ready = true;
        xSemaphoreGive(s_session_ready);
    }
    free(descrs);
}

static void ble_obd_handle_notify(esp_ble_gattc_cb_param_t *param)
{
    int add = param->notify.value_len;
    if (s_notify_acc_len + add < (int)sizeof(s_notify_acc) - 1) {
        memcpy(s_notify_acc + s_notify_acc_len, param->notify.value, add);
        s_notify_acc_len += add;
        s_notify_acc[s_notify_acc_len] = '\0';
    } else {
        // Puffer voll ohne Prompt-Zeichen -> verwerfen, Sync ist verloren
        s_notify_acc_len = 0;
        s_notify_acc[0] = '\0';
        return;
    }

    char *prompt = strchr(s_notify_acc, '>');
    if (prompt) {
        int resp_len = (int)(prompt - s_notify_acc);
        if (resp_len >= (int)sizeof(s_resp_buf)) resp_len = sizeof(s_resp_buf) - 1;
        memcpy(s_resp_buf, s_notify_acc, resp_len);
        s_resp_buf[resp_len] = '\0';
        s_notify_acc_len = 0;
        s_notify_acc[0] = '\0';
        xSemaphoreGive(s_resp_ready);
    }
}

static void ble_obd_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        s_gattc_if = gattc_if;
        esp_ble_gap_set_scan_params(&s_scan_params);
        break;

    case ESP_GATTC_CONNECT_EVT:
        s_conn_id = param->connect.conn_id;
        memcpy(s_remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        s_service_count = 0;
        esp_ble_gattc_search_service(gattc_if, param->connect.conn_id, NULL);
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "BLE-OBD2-Adapter getrennt - suche erneut");
        s_connected = false;
        s_notify_ready = false;
        s_online = false;
        s_service_count = 0;
        s_rx_handle = 0;
        s_tx_handle = 0;
        ble_obd_start_scan();
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        if (s_service_count < MAX_BLE_OBD_SERVICES) {
            s_services[s_service_count].start_handle = param->search_res.start_handle;
            s_services[s_service_count].end_handle = param->search_res.end_handle;
            s_service_count++;
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        ble_obd_find_rx_tx_char(gattc_if, param->search_cmpl.conn_id);
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ble_obd_enable_notify_cccd(gattc_if);
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        s_connected = true;
        s_notify_ready = true;
        xSemaphoreGive(s_session_ready);
        break;

    case ESP_GATTC_NOTIFY_EVT:
        ble_obd_handle_notify(param);
        break;

    default:
        break;
    }
}

// Sendet ein AT-/OBD2-Kommando (ohne CR) und wartet synchron auf die
// ELM327-Antwort (bis zum '>'-Prompt oder Timeout). Nur aus dem
// ble_obd_task-Kontext aufrufen (nicht threadsicher fuer parallele Aufrufe).
static bool send_at_cmd(const char *cmd, char *resp_out, size_t resp_out_size, TickType_t timeout)
{
    if (!s_notify_ready || s_tx_handle == 0) return false;

    char buf[24];
    int len = snprintf(buf, sizeof(buf), "%s\r", cmd);
    if (len < 0 || len >= (int)sizeof(buf)) return false;

    xSemaphoreTake(s_resp_ready, 0); // evtl. stehen gebliebene alte Antwort verwerfen
    esp_err_t err = esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_tx_handle,
                                              (uint16_t)len, (uint8_t *)buf,
                                              ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (err != ESP_OK) return false;

    if (xSemaphoreTake(s_resp_ready, timeout) != pdTRUE) return false;
    if (resp_out) {
        strncpy(resp_out, s_resp_buf, resp_out_size - 1);
        resp_out[resp_out_size - 1] = '\0';
    }
    return true;
}

static void ble_obd_task(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_session_ready, portMAX_DELAY);

    char resp[256];
    // ELM327-Initsequenz: Reset, Echo aus, Header aus, Protokoll = ISO
    // 15765-4 CAN (11 Bit, 500 kBit/s) - passend zum BMW-E90-PT-CAN.
    send_at_cmd("ATZ", resp, sizeof(resp), pdMS_TO_TICKS(3000));
    send_at_cmd("ATE0", resp, sizeof(resp), pdMS_TO_TICKS(1000));
    send_at_cmd("ATH0", resp, sizeof(resp), pdMS_TO_TICKS(1000));
    send_at_cmd("ATSP6", resp, sizeof(resp), pdMS_TO_TICKS(1000));

    int poll_step = 0;
    uint32_t last_bat_poll = 0;

    while (1) {
        if (!s_connected || !s_notify_ready) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        ble_obd_request_t req;
        if (xQueueReceive(s_request_queue, &req, 0) == pdTRUE) {
            uint8_t bytes[32];
            int n;
            switch (req) {
            case BLE_OBD_REQ_READ_DTC:
                if (send_at_cmd("03", resp, sizeof(resp), pdMS_TO_TICKS(2000))) {
                    n = hex_tokenize(resp, bytes, sizeof(bytes));
                    if (n >= 1 && bytes[0] == 0x43) {
                        int dtc_n = 0;
                        for (int i = 1; i + 1 < n && dtc_n < MAX_DTC; i += 2) {
                            if (decode_dtc_bytes(bytes[i], bytes[i + 1], s_dtc_codes[dtc_n])) dtc_n++;
                        }
                        s_dtc_count = dtc_n;
                    }
                }
                break;
            case BLE_OBD_REQ_CLEAR_DTC:
                send_at_cmd("04", resp, sizeof(resp), pdMS_TO_TICKS(2000));
                s_dtc_count = 0;
                break;
            case BLE_OBD_REQ_SERVICE_RESET:
                send_at_cmd("ATSH611", resp, sizeof(resp), pdMS_TO_TICKS(1000));
                send_at_cmd("3101FF01", resp, sizeof(resp), pdMS_TO_TICKS(2000));
                send_at_cmd("ATSH7DF", resp, sizeof(resp), pdMS_TO_TICKS(1000));
                break;
            }
            continue;
        }

        uint8_t bytes[16];
        int n;
        switch (poll_step) {
        case 0:
            if (send_at_cmd("010C", resp, sizeof(resp), pdMS_TO_TICKS(1000))) {
                n = hex_tokenize(resp, bytes, sizeof(bytes));
                if (n >= 4 && bytes[0] == 0x41 && bytes[1] == 0x0C) {
                    s_rpm = (((uint16_t)bytes[2] << 8) | bytes[3]) / 4.0f;
                    s_online = true;
                }
            }
            break;
        case 1:
            if (send_at_cmd("010D", resp, sizeof(resp), pdMS_TO_TICKS(1000))) {
                n = hex_tokenize(resp, bytes, sizeof(bytes));
                if (n >= 3 && bytes[0] == 0x41 && bytes[1] == 0x0D) {
                    s_speed_kmh = (float)bytes[2];
                    s_online = true;
                }
            }
            break;
        case 2:
            if (send_at_cmd("0105", resp, sizeof(resp), pdMS_TO_TICKS(1000))) {
                n = hex_tokenize(resp, bytes, sizeof(bytes));
                if (n >= 3 && bytes[0] == 0x41 && bytes[1] == 0x05) {
                    s_water_temp = (float)bytes[2] - 40.0f;
                    s_online = true;
                }
            }
            break;
        case 3:
            if (send_at_cmd("0111", resp, sizeof(resp), pdMS_TO_TICKS(1000))) {
                n = hex_tokenize(resp, bytes, sizeof(bytes));
                if (n >= 3 && bytes[0] == 0x41 && bytes[1] == 0x11) {
                    s_throttle_pct = bytes[2] * (100.0f / 255.0f);
                    s_online = true;
                }
            }
            break;
        }
        poll_step = (poll_step + 1) % 4;

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_bat_poll >= 1000) {
            last_bat_poll = now;
            if (send_at_cmd("ATRV", resp, sizeof(resp), pdMS_TO_TICKS(1000))) {
                float v = strtof(resp, NULL);
                if (v > 0.0f) s_bat_voltage = v;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void ble_obd_setup_security(void)
{
    // Manche ELM327-BLE-Klone verlangen beim Verbinden ein festes
    // BLE-Pairing-Passwort (siehe OBD_PASSKEYS). Die meisten guenstigen
    // Adapter erzwingen aber gar keine BLE-Sicherheit - diese Parameter
    // greifen nur, falls das Geraet tatsaechlich eine Sicherheitsanfrage
    // stellt (ESP_GAP_BLE_SEC_REQ_EVT/PASSKEY_REQ_EVT).
    esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    uint8_t auth_req = ESP_LE_AUTH_NO_BOND;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    uint8_t key_size = 16;
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
}

static void ble_obd_start_task(void *arg)
{
    (void)arg;
    // Wireless.c fuehrt beim Boot einen einmaligen ~5s-BLE-Scan fuer die
    // Demo-Seite (WIFI/BLE-Geraetezaehler) durch und registriert dafuer
    // seinen eigenen GAP-Callback. Bluedroid erlaubt nur einen globalen
    // GAP-Callback gleichzeitig - wir warten daher, bis dieser Scan sicher
    // abgeschlossen ist, bevor wir unseren Callback registrieren (der
    // WIFI/BLE-Zaehler auf der Demo-Seite ist zu diesem Zeitpunkt laengst
    // final und wird dadurch nicht beeinflusst).
    vTaskDelay(pdMS_TO_TICKS(7000));

    s_resp_ready = xSemaphoreCreateBinary();
    s_session_ready = xSemaphoreCreateBinary();
    s_request_queue = xQueueCreate(4, sizeof(ble_obd_request_t));

    ble_obd_setup_security();
    esp_ble_gap_register_callback(ble_obd_gap_cb);
    esp_ble_gattc_register_callback(ble_obd_gattc_cb);
    esp_ble_gattc_app_register(BLE_OBD_APP_ID);

    xTaskCreatePinnedToCore(ble_obd_task, "ble_obd", 4096, NULL, 4, NULL, 0);

    vTaskDelete(NULL);
}

void BLE_OBD_Init(void)
{
    xTaskCreatePinnedToCore(ble_obd_start_task, "ble_obd_start", 4096, NULL, 3, NULL, 0);
}

bool BLE_OBD_online(void) { return s_online; }
float BLE_OBD_speed_kmh(void) { return s_speed_kmh; }
float BLE_OBD_rpm(void) { return s_rpm; }
float BLE_OBD_water_temp(void) { return s_water_temp; }
float BLE_OBD_throttle_pct(void) { return s_throttle_pct; }
float BLE_OBD_bat_voltage(void) { return s_bat_voltage; }
float BLE_OBD_gforce_x(void) { return 0.0f; }
float BLE_OBD_gforce_y(void) { return 0.0f; }

void BLE_OBD_read_dtc(void)
{
    ble_obd_request_t req = BLE_OBD_REQ_READ_DTC;
    if (s_request_queue) xQueueSend(s_request_queue, &req, 0);
}

void BLE_OBD_clear_dtc(void)
{
    ble_obd_request_t req = BLE_OBD_REQ_CLEAR_DTC;
    if (s_request_queue) xQueueSend(s_request_queue, &req, 0);
    s_dtc_count = 0; // optimistisch, wird ggf. per Antwort bestaetigt
}

void BLE_OBD_reset_service_oil(void)
{
    ble_obd_request_t req = BLE_OBD_REQ_SERVICE_RESET;
    if (s_request_queue) xQueueSend(s_request_queue, &req, 0);
}

int BLE_OBD_dtc_count(void) { return s_dtc_count; }

const char *BLE_OBD_dtc_code(int idx)
{
    if (idx < 0 || idx >= s_dtc_count || idx >= MAX_DTC) return NULL;
    return s_dtc_codes[idx];
}
