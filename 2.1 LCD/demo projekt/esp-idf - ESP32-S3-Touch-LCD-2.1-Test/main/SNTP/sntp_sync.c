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

#define WIFI_CONFIG_PATH "/sdcard/wifi-einstellungen.txt"

static volatile bool s_busy = false;
static volatile bool s_cancel_requested = false;
static char s_status[64] = "Inaktiv";

bool SNTP_Sync_IsBusy(void)
{
    return s_busy;
}

const char *SNTP_Sync_Status(void)
{
    return s_status;
}

void SNTP_Sync_Cancel(void)
{
    if (s_busy) {
        s_cancel_requested = true;
    }
}

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
                "# WLAN-Zugangsdaten fuer den manuellen Zeitabgleich (SNTP)\r\n"
                "#\r\n"
                "# Im Funktionen-Screen des Displays gibt es den Schalter 'Hotspot\r\n"
                "# verbinden'. Wird er eingeschaltet, verbindet sich das Display\r\n"
                "# EINMALIG kurz mit dem hier hinterlegten WLAN (z.B. Handy-Hotspot\r\n"
                "# oder Heim-WLAN), holt sich die Uhrzeit von einem Zeitserver im\r\n"
                "# Internet und schreibt sie in die eingebaute RTC. Mit einer RTC-\r\n"
                "# Pufferbatterie haelt sie die Uhrzeit danach auch ohne Strom - ein\r\n"
                "# einmaliger Abgleich reicht also. Ohne Schalter-Aktivierung wird\r\n"
                "# NIE automatisch nach einem Hotspot gesucht.\r\n"
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

    char ssid[33];
    char pass[65];
    bool have_config = load_or_create_wifi_config(ssid, sizeof(ssid), pass, sizeof(pass));

    if (!have_config) {
        strlcpy(s_status, "Keine SSID in Datei hinterlegt", sizeof(s_status));
        SD_Log("SNTP: %s", s_status);
    } else if (s_cancel_requested) {
        strlcpy(s_status, "Abgebrochen", sizeof(s_status));
    } else {
        // Teilt sich die 2,4-GHz-Antenne mit der dauerhaft aktiven BLE-OBD-
        // Verbindung (gleiches Problem wie beim WLAN-Update, siehe ota_web.c).
        strlcpy(s_status, "Pausiere BLE...", sizeof(s_status));
        BLE_OBD_Suspend();
        vTaskDelay(pdMS_TO_TICKS(300));

        if (s_cancel_requested) {
            strlcpy(s_status, "Abgebrochen", sizeof(s_status));
        } else {
            strlcpy(s_status, "Verbinde...", sizeof(s_status));

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

            // In kleinen Schritten warten statt eines einzigen langen Timeouts,
            // damit ein Cancel (Schalter waehrenddessen ausgeschaltet) zuegig
            // greift statt bis zu 10s zu blockieren.
            EventBits_t bits = 0;
            uint32_t waited = 0;
            while (waited < SNTP_CONNECT_TIMEOUT_MS) {
                bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(200));
                if (bits & (WIFI_CONNECTED_BIT | WIFI_FAIL_BIT)) break;
                if (s_cancel_requested) break;
                waited += 200;
            }

            if (s_cancel_requested) {
                strlcpy(s_status, "Abgebrochen", sizeof(s_status));
            } else if (bits & WIFI_CONNECTED_BIT) {
                strlcpy(s_status, "Zeitabgleich laeuft...", sizeof(s_status));
                SD_Log("SNTP: mit Hotspot verbunden, starte Zeitabgleich (pool.ntp.org)");
                esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
                esp_netif_sntp_init(&sntp_cfg);
                if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(SNTP_SYNC_TIMEOUT_MS)) == ESP_OK) {
                    strlcpy(s_status, write_time_to_rtc() ? "Zeit abgeglichen" : "Fehler: ungueltige Zeit",
                            sizeof(s_status));
                } else {
                    strlcpy(s_status, "Fehler: Zeitserver antwortet nicht", sizeof(s_status));
                }
                esp_netif_sntp_deinit();
            } else {
                strlcpy(s_status, "Hotspot nicht erreichbar", sizeof(s_status));
            }
            SD_Log("SNTP: %s", s_status);

            esp_wifi_disconnect();
            esp_wifi_stop();

            esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler);
            esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_handler);
            vEventGroupDelete(s_wifi_event_group);
            s_wifi_event_group = NULL;
        }

        BLE_OBD_Resume();
    }

    s_cancel_requested = false;
    s_busy = false;
    vTaskDelete(NULL);
}

void SNTP_Sync_Start(void)
{
    if (s_busy) {
        return; // schon ein Versuch aktiv, Doppelstart vermeiden
    }
    s_busy = true;
    s_cancel_requested = false;
    strlcpy(s_status, "Starte...", sizeof(s_status));
    xTaskCreatePinnedToCore(sntp_sync_task, "sntp_sync", 4096, NULL, 2, NULL, 0);
}
