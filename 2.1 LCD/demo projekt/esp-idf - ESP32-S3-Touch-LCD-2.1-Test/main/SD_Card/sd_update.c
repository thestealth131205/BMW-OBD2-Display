#include "sd_update.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "SD_MMC.h"
#include "sd_log.h"

#define SD_UPDATE_BIN_PATH    "/sdcard/update.bin"
#define SD_UPDATE_BACKUP_PATH "/sdcard/update.backup"

// Gleicher Schwellwert wie OTA_WEB_DESC_CHECK_MIN_BYTES in ota_web.c - ab
// dieser Byte-Zahl laesst sich der App-Beschreibungsblock zuverlaessig aus
// der Partition zurueberlesen.
#define SD_UPDATE_DESC_CHECK_MIN_BYTES 512

// Benennt update.bin in update.backup um, egal ob das Einspielen erfolgreich
// war oder nicht - verhindert, dass ein fehlerhaftes/fremdes Image bei jedem
// weiteren Neustart erneut versucht wird. Eine evtl. vorhandene alte
// update.backup wird dabei ueberschrieben.
static void rename_to_backup(void)
{
    remove(SD_UPDATE_BACKUP_PATH);
    if (rename(SD_UPDATE_BIN_PATH, SD_UPDATE_BACKUP_PATH) != 0) {
        SD_Log("SD_UPDATE: Umbenennen in update.backup fehlgeschlagen (errno=%d: %s)",
               errno, strerror(errno));
    }
}

void SD_Update_CheckAndApply(void)
{
    // SD_Init_Done wird auch bei einem FEHLGESCHLAGENEN Mount auf true
    // gesetzt (siehe SD_MMC.c: SD_Init()) - nur SDCard_Size > 0 zeigt einen
    // tatsaechlich erfolgreichen Mount. Ohne diese Pruefung wuerde hier bei
    // nicht gemounteter Karte trotzdem fopen() auf /sdcard versucht.
    if (!SD_Init_Done || SDCard_Size == 0) {
        return;
    }

    FILE *f = fopen(SD_UPDATE_BIN_PATH, "rb");
    if (!f) {
        return; // keine update.bin vorhanden - normaler Bootweg
    }

    SD_Log("SD_UPDATE: update.bin auf SD-Karte gefunden, Einspielen beginnt...");

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        SD_Log("SD_UPDATE: keine freie OTA-Partition gefunden, Abbruch");
        fclose(f);
        rename_to_backup();
        return;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        SD_Log("SD_UPDATE: esp_ota_begin fehlgeschlagen: %s", esp_err_to_name(err));
        fclose(f);
        rename_to_backup();
        return;
    }

    static uint8_t buf[4096];
    size_t received_total = 0;
    bool desc_checked = false;
    bool failed = false;
    size_t n;

    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        err = esp_ota_write(ota_handle, buf, n);
        if (err != ESP_OK) {
            SD_Log("SD_UPDATE: esp_ota_write fehlgeschlagen: %s", esp_err_to_name(err));
            failed = true;
            break;
        }
        received_total += n;

        // Gleiche Pruefung wie beim WLAN-Upload (ota_web.c): erkennt eine
        // falsche Datei anhand des fehlenden App-Beschreibungsblocks, bevor
        // unnoetig die ganze (ggf. grosse) Datei geschrieben wird.
        if (!desc_checked && received_total >= SD_UPDATE_DESC_CHECK_MIN_BYTES) {
            desc_checked = true;
            esp_app_desc_t app_desc = {0};
            esp_err_t desc_err = esp_ota_get_partition_description(update_partition, &app_desc);
            if (desc_err != ESP_OK || app_desc.magic_word != ESP_APP_DESC_MAGIC_WORD) {
                SD_Log("SD_UPDATE: falsches Image erkannt (kein gueltiger App-Header)");
                failed = true;
                break;
            }
        }
    }
    if (!failed && ferror(f)) {
        SD_Log("SD_UPDATE: Lesefehler auf der SD-Karte beim Einlesen von update.bin");
        failed = true;
    }
    fclose(f);

    if (!failed && !desc_checked) {
        esp_app_desc_t app_desc = {0};
        esp_err_t desc_err = esp_ota_get_partition_description(update_partition, &app_desc);
        if (desc_err != ESP_OK || app_desc.magic_word != ESP_APP_DESC_MAGIC_WORD) {
            SD_Log("SD_UPDATE: falsches/zu kleines Image erkannt (%u Bytes)", (unsigned)received_total);
            failed = true;
        }
    }

    if (failed) {
        esp_ota_abort(ota_handle);
        rename_to_backup();
        return;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        SD_Log("SD_UPDATE: esp_ota_end fehlgeschlagen (ungueltiges Image?): %s", esp_err_to_name(err));
        rename_to_backup();
        return;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        SD_Log("SD_UPDATE: esp_ota_set_boot_partition fehlgeschlagen: %s", esp_err_to_name(err));
        rename_to_backup();
        return;
    }

    SD_Log("SD_UPDATE: Update erfolgreich (%u Bytes), update.bin -> update.backup, Neustart folgt",
           (unsigned)received_total);
    rename_to_backup();

    // Kurze Pause, damit die Log-Zeile sicher per fflush()/fsync() auf der
    // Karte landet, bevor der Neustart greift.
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}
