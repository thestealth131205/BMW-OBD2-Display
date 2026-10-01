#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TCA9554PWR.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "ST7701S.h"
#include "CST820.h"
#include "SD_MMC.h"
#include "sd_log.h"
#include "LVGL_Driver.h"
#include "LVGL_Example.h"
#include "Wireless.h"
#include "bmw_ui.h"
#include "can_obd2.h"
#include "ble_obd.h"
#include "esp_core_dump.h"

// Die Boot-Loops/Abstuerze waehrend der Fahrt liessen sich bisher nicht
// diagnostizieren, weil der echte Panic-Grund/Backtrace nur auf der seriellen
// USB-Konsole ausgegeben wird - im Auto ist kein Rechner angeschlossen. Mit
// CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH landet ein Crash-Dump stattdessen auf
// der eigenen "coredump"-Partition (partitions.csv) und bleibt bis zum
// naechsten Boot erhalten. Hier wird er beim Start geprueft, die wichtigsten
// Angaben (Absturzursache, Programmzaehler, Backtrace-Adressen) gehen ins
// SD-Log, danach wird der Dump geloescht, damit er nicht erneut gemeldet wird.
static void log_and_clear_coredump(void)
{
    if (esp_core_dump_image_check() != ESP_OK) {
        return; // kein Dump vorhanden
    }

    esp_core_dump_summary_t summary;
    if (esp_core_dump_get_summary(&summary) == ESP_OK) {
        SD_Log("=== COREDUMP gefunden: Task '%s', PC=0x%08" PRIx32 ", exc_cause=%" PRIu32 ", exc_vaddr=0x%08" PRIx32,
               summary.exc_task, summary.exc_pc, summary.ex_info.exc_cause, summary.ex_info.exc_vaddr);
        char bt[256];
        int p = 0;
        for (uint32_t i = 0; i < summary.exc_bt_info.depth && i < 16; i++) {
            p += snprintf(bt + p, sizeof(bt) - p, "0x%08" PRIx32 " ", summary.exc_bt_info.bt[i]);
            if (p >= (int)sizeof(bt) - 12) break;
        }
        SD_Log("COREDUMP Backtrace (corrupted=%d): %s", (int)summary.exc_bt_info.corrupted, bt);
    } else {
        SD_Log("=== COREDUMP vorhanden, aber Zusammenfassung konnte nicht gelesen werden ===");
    }

    esp_core_dump_image_erase();
}

void Driver_Loop(void *parameter)
{
    while(1)
    {
        QMI8658_Loop();
        RTC_Loop();
        BAT_Get_Volts();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}
void Driver_Init(void)
{
    Flash_Searching();
    BAT_Init();
    I2C_Init();
    PCF85063_Init();
    QMI8658_Init();
    EXIO_Init();                    // Example Initialize EXIO
    xTaskCreatePinnedToCore(
        Driver_Loop, 
        "Other Driver task",
        4096, 
        NULL, 
        3, 
        NULL, 
        0);
}
void app_main(void)
{   
    Wireless_Init();
    Driver_Init();

    LCD_Init();
    Touch_Init();
    SD_Init();
    SD_Log_Init();
    log_and_clear_coredump();
    LVGL_Init();

    // OBD2 per MCP2515 (SPI) starten - liest PT-CAN-Broadcasts im Hintergrund
    CAN_OBD2_Init();

    // OBD2 per Bluetooth-LE-ELM327-Adapter (z.B. Veepeak OBDCheck BLE) -
    // scannt/verbindet im Hintergrund, Standard-Datenquelle in bmw_ui.c
    BLE_OBD_Init();
/********************* BMW Multi-Ansicht (Standard) + Waveshare-Demo *********************/
    // Demo-Seite des Waveshare-Projekts auf einem eigenen Screen aufbauen
    // (erscheint beim 3-Sekunden-Halten in der Bildschirmmitte).
    lv_obj_t *scr_demo = lv_obj_create(NULL);
    lv_scr_load(scr_demo);
    Lvgl_Example1();            // baut auf dem aktiven Screen = scr_demo auf

    // BMW-Multi-Ansicht als Standard-Screen laden.
    BMW_UI_Init(scr_demo);

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
