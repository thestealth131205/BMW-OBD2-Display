#include "ota_web.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "lvgl.h"
#include "sd_log.h"
#include "ble_obd.h"

// Mindestanzahl Bytes, bevor wir den App-Beschreibungsblock (esp_app_desc_t)
// zuverlaessig aus der Partition zurücklesen koennen - der Block beginnt kurz
// nach dem Bild-/Segment-Header (ueblicherweise bei Offset 32) und ist selbst
// rund 256 Byte gross.
#define OTA_WEB_DESC_CHECK_MIN_BYTES 512

// Eigener SoftAP, damit das Update auch unterwegs ohne vorhandenes WLAN
// funktioniert - Handy/PC verbindet sich direkt mit dem Display.
#define OTA_WEB_SSID "BMW-E90-OTA"
#define OTA_WEB_PASS "bmw320i2010"

static httpd_handle_t s_server = NULL;
static esp_netif_t *s_ap_netif = NULL;
static bool s_enabled = false;

static lv_obj_t *s_scr_ota = NULL;
static lv_obj_t *s_qrcode = NULL;
static lv_obj_t *s_status_label = NULL;
static lv_obj_t *s_progress_bar = NULL;
static lv_obj_t *s_prev_screen = NULL;
static lv_timer_t *s_ui_timer = NULL;

static volatile int  s_progress_percent = 0;
static volatile bool s_upload_active = false;
static volatile bool s_upload_done = false;
static volatile bool s_upload_ok = false;
static volatile bool s_reboot_pending = false;
// s_ap_busy schuetzt nur den Start-Vorgang vor doppeltem Anstossen. Das
// Beenden/Zurueck-Navigieren haengt bewusst NICHT mehr an diesem Flag (siehe
// OTA_Web_SetEnabled) - vorher blieb man auf dem Screen gefangen, wenn der
// Start-Task haengenblieb (WLAN kam nie hoch), weil "Beenden" dann durch die
// alte Busy-Sperre blockiert wurde.
static volatile bool s_ap_busy = false;
static volatile bool s_ap_active = false;
static volatile bool s_cancel_requested = false;
static volatile uint32_t s_start_requested_ms = 0;
static char s_last_error[64] = {0};

// Einfache Upload-Seite: Dateiauswahl + XHR-Upload (rohe Bytes als Body,
// kein multipart/form-data noetig) mit Fortschrittsanzeige im Browser.
static const char OTA_WEB_PAGE[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>BMW Display Update</title>"
"<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:30px}"
"button{font-size:18px;padding:10px 24px;margin-top:16px;background:#2962ff;color:#fff;border:none;border-radius:6px}"
"progress{width:90%;height:22px;margin-top:20px}</style></head><body>"
"<h2>BMW E90 Display &ndash; Firmware-Update</h2>"
"<p>Nur die Datei <b>*_update.bin</b> auswaehlen und hochladen "
"(nicht die -merged.bin, die ist nur fuer das initiale USB-Flashen).</p>"
"<input type='file' id='f'><br>"
"<button onclick='up()'>Hochladen</button>"
"<p id='s'></p>"
"<progress id='p' value='0' max='100' style='display:none'></progress>"
"<script>"
"function up(){"
"var f=document.getElementById('f').files[0];"
"if(!f){document.getElementById('s').innerText='Keine Datei gewaehlt';return;}"
"var p=document.getElementById('p');p.style.display='block';"
"var x=new XMLHttpRequest();"
"x.open('POST','/update',true);"
"x.upload.onprogress=function(e){if(e.lengthComputable){p.value=(e.loaded/e.total)*100;}};"
"x.onload=function(){document.getElementById('s').innerText=x.responseText;};"
"x.onerror=function(){document.getElementById('s').innerText='Uebertragungsfehler';};"
"x.send(f);"
"document.getElementById('s').innerText='Wird hochgeladen...';"
"}"
"</script></body></html>";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, OTA_WEB_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t update_post_handler(httpd_req_t *req)
{
    s_upload_active = true;
    s_upload_done = false;
    s_progress_percent = 0;

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        s_upload_active = false;
        SD_Log("OTA_WEB: keine freie OTA-Partition gefunden");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Keine OTA-Partition gefunden");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        s_upload_active = false;
        SD_Log("OTA_WEB: esp_ota_begin fehlgeschlagen: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA-Start fehlgeschlagen");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int total = remaining > 0 ? remaining : 1;
    int received_total = 0;
    bool desc_checked = false;
    static char buf[4096];

    while (remaining > 0) {
        int to_read = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
        int recv_len = httpd_req_recv(req, buf, to_read);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            esp_ota_abort(ota_handle);
            s_upload_active = false;
            s_upload_done = true;
            s_upload_ok = false;
            SD_Log("OTA_WEB: Empfang abgebrochen (recv_len=%d)", recv_len);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Empfang fehlgeschlagen");
            return ESP_FAIL;
        }
        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            esp_ota_abort(ota_handle);
            s_upload_active = false;
            s_upload_done = true;
            s_upload_ok = false;
            SD_Log("OTA_WEB: esp_ota_write fehlgeschlagen: %s", esp_err_to_name(err));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Schreiben fehlgeschlagen");
            return ESP_FAIL;
        }
        remaining -= recv_len;
        received_total += recv_len;
        s_progress_percent = (received_total * 100) / total;

        // Sobald genug Bytes geschrieben sind, den App-Beschreibungsblock aus
        // der Partition zurücklesen - das erkennt zuverlaessig, ob eine echte
        // App-Firmware (*_update.bin) oder faelschlich die -merged.bin
        // (Bootloader+Partitionstabelle+App ab Offset 0x0) hochgeladen wurde:
        // bei der merged.bin steht an dieser Stelle der Bootloader, nicht der
        // App-Header, der Beschreibungsblock fehlt also an der erwarteten
        // Stelle.
        if (!desc_checked && received_total >= OTA_WEB_DESC_CHECK_MIN_BYTES) {
            desc_checked = true;
            esp_app_desc_t app_desc = {0};
            esp_err_t desc_err = esp_ota_get_partition_description(update_partition, &app_desc);
            if (desc_err != ESP_OK || app_desc.magic_word != ESP_APP_DESC_MAGIC_WORD) {
                esp_ota_abort(ota_handle);
                s_upload_active = false;
                s_upload_done = true;
                s_upload_ok = false;
                SD_Log("OTA_WEB: falsches Image erkannt (kein gueltiger App-Header, magic=0x%08" PRIx32 ")",
                       (uint32_t)app_desc.magic_word);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                    "Falsches Image: Das ist nicht die App-Firmware. "
                    "Bitte die Datei *_update.bin hochladen, nicht die -merged.bin.");
                return ESP_FAIL;
            }
        }
    }

    // Fallback fuer sehr kleine Uploads (z.B. versehentlich eine falsche,
    // winzige Datei), bei denen die Schleife nie OTA_WEB_DESC_CHECK_MIN_BYTES
    // erreicht hat.
    if (!desc_checked) {
        esp_app_desc_t app_desc;
        esp_err_t desc_err = esp_ota_get_partition_description(update_partition, &app_desc);
        if (desc_err != ESP_OK || app_desc.magic_word != ESP_APP_DESC_MAGIC_WORD) {
            esp_ota_abort(ota_handle);
            s_upload_active = false;
            s_upload_done = true;
            s_upload_ok = false;
            SD_Log("OTA_WEB: falsches/zu kleines Image erkannt (%d Bytes)", received_total);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                "Falsches Image: Das ist keine gueltige App-Firmware.");
            return ESP_FAIL;
        }
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        s_upload_active = false;
        s_upload_done = true;
        s_upload_ok = false;
        SD_Log("OTA_WEB: esp_ota_end fehlgeschlagen (ungueltiges Image?): %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Firmware-Image ungueltig");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        s_upload_active = false;
        s_upload_done = true;
        s_upload_ok = false;
        SD_Log("OTA_WEB: esp_ota_set_boot_partition fehlgeschlagen: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Boot-Partition konnte nicht gesetzt werden");
        return ESP_FAIL;
    }

    SD_Log("OTA_WEB: Update erfolgreich (%d Bytes), Neustart folgt", received_total);
    httpd_resp_sendstr(req, "OK - Update erfolgreich, Display startet neu...");
    s_upload_active = false;
    s_upload_done = true;
    s_upload_ok = true;
    s_reboot_pending = true;
    return ESP_OK;
}

// Jeder einzelne Schritt wird mit seinem Rueckgabewert geloggt - vorher
// liefen esp_wifi_stop/set_mode/set_config/start/httpd_start komplett
// ungeprueft durch, ein Fehlschlag irgendwo mittendrin blieb unsichtbar und
// der Rest der Funktion lief trotzdem weiter (z.B. httpd_start auf einem nie
// tatsaechlich gestarteten AP). Gibt bei jedem Fehler sofort false zurueck
// und bricht ab, statt mit einem bereits ungueltigen Zustand weiterzumachen.
static bool start_ap_and_server(void)
{
    s_last_error[0] = '\0';
    esp_err_t err;

    err = esp_wifi_stop();
    SD_Log("OTA_WEB: esp_wifi_stop -> %s", esp_err_to_name(err));

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        SD_Log("OTA_WEB: AP-Netif erstellt: %s", s_ap_netif ? "ok" : "FEHLER (NULL)");
        if (!s_ap_netif) {
            snprintf(s_last_error, sizeof(s_last_error), "AP-Netif fehlgeschlagen");
            return false;
        }
    }

    wifi_config_t ap_config = {0};
    strlcpy((char *)ap_config.ap.ssid, OTA_WEB_SSID, sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = strlen(OTA_WEB_SSID);
    strlcpy((char *)ap_config.ap.password, OTA_WEB_PASS, sizeof(ap_config.ap.password));
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 2;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    SD_Log("OTA_WEB: esp_wifi_set_mode(AP) -> %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        snprintf(s_last_error, sizeof(s_last_error), "set_mode: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    SD_Log("OTA_WEB: esp_wifi_set_config(AP) -> %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        snprintf(s_last_error, sizeof(s_last_error), "set_config: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_start();
    SD_Log("OTA_WEB: esp_wifi_start -> %s (freier interner Heap: %u Byte)",
           esp_err_to_name(err), (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    if (err != ESP_OK) {
        snprintf(s_last_error, sizeof(s_last_error), "wifi_start: %s", esp_err_to_name(err));
        return false;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 4;
    config.lru_purge_enable = true;

    err = httpd_start(&s_server, &config);
    SD_Log("OTA_WEB: httpd_start -> %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        snprintf(s_last_error, sizeof(s_last_error), "httpd: %s", esp_err_to_name(err));
        return false;
    }

    httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &root_uri);

    httpd_uri_t update_uri = {
        .uri = "/update", .method = HTTP_POST, .handler = update_post_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &update_uri);

    SD_Log("OTA_WEB: Access Point aktiv (SSID=%s)", OTA_WEB_SSID);
    return true;
}

static void stop_ap_and_server(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    // Zurueck in den Ausgangszustand (reiner STA-Modus wie nach dem Boot-Scan).
    esp_err_t err = esp_wifi_stop();
    SD_Log("OTA_WEB: Stop esp_wifi_stop -> %s", esp_err_to_name(err));
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    SD_Log("OTA_WEB: Stop esp_wifi_set_mode(STA) -> %s", esp_err_to_name(err));
    err = esp_wifi_start();
    SD_Log("OTA_WEB: Stop esp_wifi_start(STA) -> %s", esp_err_to_name(err));
    s_ap_active = false;
}

// start_ap_and_server()/stop_ap_and_server() rufen esp_wifi_stop/set_mode/
// set_config/start und httpd_start/httpd_stop auf - alles blockierend und in
// Kombination mit dem parallel laufenden BLE-Stack spuerbar langsam. Der
// Schalter wird aber direkt aus dem LVGL-Event-Callback ausgeloest, der
// innerhalb von app_mains lv_timer_handler()-Schleife laeuft - also im
// einzigen Task auf Core 0 neben dem Idle-Task. Blockierte dieser Task lange
// genug, kam der Idle-Task nicht mehr zum Zug und die Task-Watchdog loeste
// einen Reset aus, sobald man den Schalter antippte (genau das beobachtete
// Verhalten: Neustart statt eines laufenden Access Points). Deshalb laufen
// beide Funktionen jetzt in einem eigenen Hintergrund-Task auf Core 1, der
// LVGL-Haupttask auf Core 0 bleibt dabei frei fuer lv_timer_handler().
static void ota_ap_start_task(void *arg)
{
    LV_UNUSED(arg);
    // Die dauerhaft aktive BLE-OBD-Verbindung (alle 20ms ein Kommando) und
    // der WiFi-AP teilen sich auf dem ESP32-S3 dieselbe 2,4-GHz-Antenne
    // (Software-Koexistenz) - bei hoher BLE-Last kann esp_wifi_start() sonst
    // minutenlang haengen bzw. nie zurueckkehren (beobachtet: "WLAN-Start
    // dauert ungewoehnlich lange", Screen blieb dauerhaft stehen). BLE_OBD
    // wird deshalb fuer die Dauer des Updates komplett pausiert, mit kurzer
    // Verzoegerung, damit Scan-Stop/Disconnect sicher durchgelaufen sind,
    // bevor WiFi die Antenne beansprucht.
    BLE_OBD_Suspend();
    vTaskDelay(pdMS_TO_TICKS(300));
    bool ok = start_ap_and_server();
    s_ap_active = ok;
    if (s_cancel_requested) {
        // Waehrend des Starts wurde bereits "Beenden" gedrueckt (die UI ist
        // davon unabhaengig sofort zurueckgesprungen) - jetzt den gerade erst
        // aufgebauten AP sofort wieder abbauen.
        s_cancel_requested = false;
        SD_Log("OTA_WEB: Start fertig, aber zwischenzeitlich Stop angefordert - wird sofort nachgeholt");
        stop_ap_and_server();
        BLE_OBD_Resume();
    } else if (!ok) {
        // WiFi-Start fehlgeschlagen - BLE_OBD nicht sinnlos pausiert lassen.
        BLE_OBD_Resume();
    }
    s_ap_busy = false;
    vTaskDelete(NULL);
}

static void ota_ap_stop_task(void *arg)
{
    LV_UNUSED(arg);
    stop_ap_and_server();
    BLE_OBD_Resume();
    s_ap_busy = false;
    vTaskDelete(NULL);
}

static void close_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    OTA_Web_SetEnabled(false);
}

static void ota_ui_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    if (s_upload_active) {
        lv_label_set_text_fmt(s_status_label, "Upload laeuft: %d%%", s_progress_percent);
        lv_bar_set_value(s_progress_bar, s_progress_percent, LV_ANIM_OFF);
    } else if (s_upload_done) {
        lv_label_set_text(s_status_label, s_upload_ok
                           ? "Update erfolgreich - Neustart..."
                           : "Update fehlgeschlagen - erneut versuchen");
    } else if (s_ap_busy) {
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - s_start_requested_ms;
        if (elapsed > 8000) {
            lv_label_set_text(s_status_label, "WLAN-Start dauert ungewoehnlich lange...");
        }
    } else if (s_ap_active) {
        lv_label_set_text(s_status_label, "AP aktiv - warte auf Verbindung/Upload");
    } else if (s_last_error[0] != '\0') {
        lv_label_set_text_fmt(s_status_label, "Fehler: %s", s_last_error);
    }

    if (s_reboot_pending) {
        s_reboot_pending = false;
        lv_timer_del(s_ui_timer);
        s_ui_timer = NULL;
        // Kurze Pause, damit die HTTP-Antwort sicher beim Client ankommt und
        // die Erfolgsmeldung auf dem Display noch sichtbar ist, bevor der
        // Neustart greift.
        vTaskDelay(pdMS_TO_TICKS(1500));
        esp_restart();
    }
}

static void build_ota_screen(void)
{
    if (s_scr_ota) {
        return;
    }
    s_scr_ota = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr_ota, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_scr_ota, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_scr_ota, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(s_scr_ota);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_label_set_text(title, "WiFi-Update");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 22);

    // QR-Code mit "WIFI:"-URI: Handy-Kamera/Scanner verbindet sich damit
    // direkt, ohne SSID/Passwort manuell eintippen zu muessen (iOS/Android
    // unterstuetzen dieses Format nativ in der Kamera- bzw. QR-Scanner-App).
    char qr_buf[96];
    snprintf(qr_buf, sizeof(qr_buf), "WIFI:T:WPA;S:%s;P:%s;;", OTA_WEB_SSID, OTA_WEB_PASS);
    s_qrcode = lv_qrcode_create(s_scr_ota, 108, lv_color_black(), lv_color_white());
    lv_qrcode_update(s_qrcode, qr_buf, strlen(qr_buf));
    lv_obj_set_style_border_color(s_qrcode, lv_color_white(), 0);
    lv_obj_set_style_border_width(s_qrcode, 5, 0);
    lv_obj_align(s_qrcode, LV_ALIGN_CENTER, 0, -102);

    lv_obj_t *info = lv_label_create(s_scr_ota);
    lv_obj_set_style_text_color(info, lv_color_white(), 0);
    lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(info, 320);
    lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text_fmt(info,
        "SSID: %s   PW: %s\noder Browser: http://192.168.4.1",
        OTA_WEB_SSID, OTA_WEB_PASS);
    lv_obj_align(info, LV_ALIGN_CENTER, 0, -8);

    s_progress_bar = lv_bar_create(s_scr_ota);
    lv_obj_set_size(s_progress_bar, 300, 20);
    lv_obj_align(s_progress_bar, LV_ALIGN_CENTER, 0, 95);
    lv_bar_set_range(s_progress_bar, 0, 100);
    lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);

    s_status_label = lv_label_create(s_scr_ota);
    lv_obj_set_style_text_color(s_status_label, lv_color_white(), 0);
    lv_label_set_text(s_status_label, "Warte auf Upload...");
    lv_obj_align(s_status_label, LV_ALIGN_CENTER, 0, 125);

    lv_obj_t *btn_close = lv_btn_create(s_scr_ota);
    lv_obj_set_style_bg_color(btn_close, lv_color_hex(0xCC2222), 0);
    lv_obj_set_size(btn_close, 160, 50);
    lv_obj_align(btn_close, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_add_event_cb(btn_close, close_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *close_lbl = lv_label_create(btn_close);
    lv_label_set_text(close_lbl, "Beenden");
    lv_obj_center(close_lbl);
}

bool OTA_Web_IsEnabled(void)
{
    return s_enabled;
}

void OTA_Web_SetEnabled(bool enable)
{
    if (enable == s_enabled) {
        return;
    }

    if (enable) {
        // Nur das Einschalten wird durch s_ap_busy vor doppeltem Anstossen
        // geschuetzt - haengt ein vorheriger Start-Versuch noch, wird hier
        // einfach gewartet statt einen zweiten parallel zu starten.
        if (s_ap_busy) {
            return;
        }
        s_enabled = true;
        s_ap_busy = true;
        s_ap_active = false;
        s_cancel_requested = false;
        s_start_requested_ms = (uint32_t)(esp_timer_get_time() / 1000);

        build_ota_screen();
        s_prev_screen = lv_scr_act();
        s_progress_percent = 0;
        s_upload_active = false;
        s_upload_done = false;
        lv_label_set_text(s_status_label, "WLAN wird gestartet...");
        lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
        lv_scr_load(s_scr_ota);

        s_ui_timer = lv_timer_create(ota_ui_timer_cb, 200, NULL);
        xTaskCreatePinnedToCore(ota_ap_start_task, "ota_ap_start", 6144, NULL, 5, NULL, 1);
        SD_Log("OTA_WEB: Start angefordert (SSID=%s)", OTA_WEB_SSID);
    } else {
        // "Beenden" muss immer sofort wirken, auch wenn der Start noch
        // laeuft oder haengengeblieben ist - sonst sitzt man auf diesem
        // Screen fest (genau das war der gemeldete Bug: s_ap_busy blieb
        // haengen, "Beenden" rief OTA_Web_SetEnabled(false) auf, die alte
        // Busy-Sperre am Funktionsanfang hat das dann einfach ignoriert).
        // Die Bildschirm-Navigation haengt deshalb ab sofort an nichts
        // anderem mehr als dem Tastendruck selbst.
        s_enabled = false;
        if (s_ui_timer) {
            lv_timer_del(s_ui_timer);
            s_ui_timer = NULL;
        }
        if (s_prev_screen) {
            lv_scr_load(s_prev_screen);
        }
        if (s_ap_busy) {
            // Start-Task laeuft noch (oder haengt) - der raeumt sich beim
            // Fertigwerden selbst ab (siehe ota_ap_start_task). Kann nicht
            // hier parallel nochmal gestartet werden, sonst doppelte
            // esp_wifi_*-Aufrufe aus zwei Tasks gleichzeitig.
            s_cancel_requested = true;
            SD_Log("OTA_WEB: Beenden waehrend Start noch laeuft/haengt - Stop wird nachgeholt sobald der Start fertig ist");
        } else {
            xTaskCreatePinnedToCore(ota_ap_stop_task, "ota_ap_stop", 4096, NULL, 5, NULL, 1);
            SD_Log("OTA_WEB: Stop angefordert");
        }
    }
}
