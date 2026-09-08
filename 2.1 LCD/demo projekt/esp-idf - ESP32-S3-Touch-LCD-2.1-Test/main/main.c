#include <stdio.h>
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
#include "LVGL_Driver.h"
#include "LVGL_Example.h"
#include "Wireless.h"
#include "bmw_ui.h"
#include "can_obd2.h"

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
    LVGL_Init();

    // OBD2 per MCP2515 (SPI) starten - liest PT-CAN-Broadcasts im Hintergrund
    CAN_OBD2_Init();
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
