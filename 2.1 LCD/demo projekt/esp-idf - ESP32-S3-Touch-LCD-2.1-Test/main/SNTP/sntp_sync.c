#include "sntp_sync.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sd_log.h"
#include "SD_MMC.h"
#include "PCF85063.h"
#include "ble_obd.h"
#include "Wireless.h"

#define SNTP_CONNECT_TIMEOUT_MS 10000
#define SNTP_SYNC_TIMEOUT_MS 8000
#define SD_WAIT_TIMEOUT_MS 5000

#define WIFI_CONFIG_PATH "/sdcard/wifi-einstellungen.txt"

// Legt die Vorlage an, falls die Datei noch nicht existiert, und liest
// andernfalls SSID=/PASSWORT= daraus aus. Gibt true zurueck, wenn eine
// nicht-leere SSID gefunden wurde.
static bool load_or_create_wifi_config(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    ssid[0] = '\0';
    pass[0] = '\0';

    if (SDCard_Size == 0) {
        return false; // keine Karte gemountet - nichts zu lesen/anzulegen
    }

    FILE *f = fopen(WIFI_CONFIG_PATH, "r");
    if (!f) {
        // Datei existiert noch nicht - Vorlage mit Erklaerung/Beispiel anlegen,
        // damit am PC klar ist, wie SSID/Passwort eingetragen werden muessen.
        f = fopen(WIFI_CONFIG_PATH, "w");
        if (f) {
            fprintf(f,
                "# WLAN-Zugangsdaten fuer den automatischen Zeitabgleich (SNTP)\r\n"
                "#\r\n"
                "# Das Display verbindet sich damit EINMALIG kurz nach dem Einschalten\r\n"
                "# mit diesem WLAN (z.B. Handy-Hotspot oder Heim-WLAN), holt sich die\r\n"
                "# Uhrzeit von einem Zeitserver im Internet und schreibt sie in die\r\n"
                "# eingebaute RTC. Mit einer RTC-Pufferbatterie haelt sie die Uhrzeit\r\n"
                "# danach auch ohne Strom - ein einmaliger Abgleich reicht also.\r\n"
                "#\r\n"
                "# Zum Eintragen: Nach dem Gleichheitszeichen ohne Anfuehrungszeichen\r\n"
                "# und ohne Leerzeichen ausfuellen, Datei speichern, Karte zurueck ins\r\n"
                "# Display stecken. Zeilen, die mit # beginnen, werden ignoriert.\r\n"
                "#\r\n"
                "# Beispiel:\r\n"
                "# SSID=MeinWLAN\r\n"
                "# PASSWORT=MeinPasswort123\r\n"
                "\r\n"
                "SSID=\r\n"
                "PASSWORT=\r\n");
            fclose(f);
            SD_Log("SNTP: %s nicht gefunden, Vorlage mit Beispiel angelegt", WIFI_CONFIG_PATH);
        }
        return false;
    }

    char line[160];
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\r' || *p == '\n' || *p == '\0') continue;

        char *nl = strpbrk(p, "\r\n");
        if (nl) *nl = '\0';

        if (strncmp(p, "SSID=", 5) == 0) {
            strlcpy(ssid, p + 5, ssid_len);
        } else if (strncmp(p, "PASSWORT=", 9) == 0) {
            strlcpy(pass, p + 9, pass_len);
        }
    }
    fclose(f);

    return strlen(ssid) > 0;
}

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;

static void sntp_wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Schreibt die aktuelle Systemzeit (per SNTP gesetzt) als LOKALE Zeit
// (Deutschland, inkl. Sommerzeit) in die PCF85063 - sd_log.c und die BMW-UI
// lesen die RTC direkt aus und zeigen sie ohne weitere Umrechnung an.
static bool write_time_to_rtc(void)
{
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    time_t now = 0;
    time(&now);
    if (now < 1700000000) { // < Ende 2023 -> SNTP hat die Zeit nicht wirklich gesetzt
        return false;
    }

    struct tm local_tm;
    localtime_r(&now, &local_tm);

    datetime_t dt = {0};
    dt.year   = (uint16_t)(local_tm.tm_year + 1900);
    dt.month  = (uint8_t)(local_tm.tm_mon + 1);
    dt.day    = (uint8_t)local_tm.tm_mday;
    dt.dotw   = (uint8_t)local_tm.tm_wday; // tm_wday: 0=Sonntag, passt zum PCF85063-Format
    dt.hour   = (uint8_t)local_tm.tm_hour;
    dt.minute = (uint8_t)local_tm.tm_min;
    dt.second = (uint8_t)local_tm.tm_sec;

    PCF85063_Set_All(dt);
    SD_Log("SNTP: RTC gestellt auf %04u-%02u-%02u %02u:%02u:%02u (lokal)",
           dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
    return true;
}

static void sntp_sync_task(void *arg)
{
    (void)arg;

    // Diese Task hat eine hoehere Prioritaet als der Main-Task und koennte
    // sonst schon laufen, bevor app_main() SD_Init() ueberhaupt aufgerufen
    // hat. SD_Init_Done wird dort in jedem Fall gesetzt (Erfolg oder
    // Fehlschlag) - kurz darauf warten, statt auf eine feste Verzoegerung
    // zu vertrauen.
    uint32_t sd_wait = 0;
    while (!SD_Init_Done && sd_wait < SD_WAIT_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(50));
        sd_wait += 50;
    }

    char ssid[33];
    char pass[65];
    if (!load_or_create_wifi_config(ssid, sizeof(ssid), pass, sizeof(pass))) {
        // Keine Karte, Datei fehlte noch (wurde gerade als Vorlage angelegt)
        // oder SSID ist leer gelassen - Feature bleibt inaktiv.
        vTaskDelete(NULL);
        return;
    }

    // Dem Boot-Scan aus Wireless.c (WIFI_Init) Zeit geben, seinen eigenen
    // passiven Scan abzuschliessen und esp_wifi_stop() aufzurufen - sonst
    // konkurrieren zwei Tasks um denselben WiFi-Treiber-Zustand. WiFi_Scan_
    // Finish wird dort erst gesetzt, nachdem esp_wifi_init()/esp_wifi_start()
    // bereits gelaufen sind, das Treiberhandle ist zu diesem Zeitpunkt also
    // in jedem Fall gueltig.
    uint32_t waited = 0;
    while (!WiFi_Scan_Finish && waited < 8000) {
        vTaskDelay(pdMS_TO_TICKS(100));
        waited += 100;
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    // Teilt sich die 2,4-GHz-Antenne mit der dauerhaft aktiven BLE-OBD-
    // Verbindung (gleiches Problem wie beim WLAN-Update, siehe ota_web.c).
    BLE_OBD_Suspend();
    vTaskDelay(pdMS_TO_TICKS(300));

    s_wifi_event_group = xEventGroupCreate();
    esp_event_handler_instance_t wifi_handler = NULL;
    esp_event_handler_instance_t ip_handler = NULL;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &sntp_wifi_event_handler, NULL, &wifi_handler);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &sntp_wifi_event_handler, NULL, &ip_handler);

    wifi_config_t sta_config = {0};
    strlcpy((char *)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid));
    strlcpy((char *)sta_config.sta.password, pass, sizeof(sta_config.sta.password));
    sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    SD_Log("SNTP: esp_wifi_set_mode(STA) -> %s", esp_err_to_name(err));
    err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    SD_Log("SNTP: esp_wifi_set_config -> %s", esp_err_to_name(err));
    err = esp_wifi_start();
    SD_Log("SNTP: esp_wifi_start -> %s", esp_err_to_name(err));
    err = esp_wifi_connect();
    SD_Log("SNTP: esp_wifi_connect -> %s", esp_err_to_name(err));

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(SNTP_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        SD_Log("SNTP: mit Heim-WLAN verbunden, starte Zeitabgleich (pool.ntp.org)");
        esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        esp_netif_sntp_init(&sntp_cfg);
        if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(SNTP_SYNC_TIMEOUT_MS)) == ESP_OK) {
            if (!write_time_to_rtc()) {
                SD_Log("SNTP: Zeitserver-Antwort erhalten, aber Systemzeit wirkt ungueltig - RTC nicht geschrieben");
            }
        } else {
            SD_Log("SNTP: Zeitserver antwortete nicht innerhalb von %ds", SNTP_SYNC_TIMEOUT_MS / 1000);
        }
        esp_netif_sntp_deinit();
    } else {
        SD_Log("SNTP: Heim-WLAN nicht erreichbar/Verbindung fehlgeschlagen - RTC bleibt unveraendert");
    }

    esp_wifi_disconnect();
    esp_wifi_stop();

    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_handler);
    vEventGroupDelete(s_wifi_event_group);
    s_wifi_event_group = NULL;

    BLE_OBD_Resume();

    vTaskDelete(NULL);
}

void SNTP_Sync_Init(void)
{
    xTaskCreatePinnedToCore(sntp_sync_task, "sntp_sync", 4096, NULL, 2, NULL, 0);
}
