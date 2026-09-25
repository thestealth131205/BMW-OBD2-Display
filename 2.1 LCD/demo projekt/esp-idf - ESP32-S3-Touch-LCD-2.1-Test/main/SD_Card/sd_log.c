#include "sd_log.h"

#include <stdio.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "SD_MMC.h"
#include "PCF85063/PCF85063.h"

#define SD_LOG_PATH "/sdcard/obd_log.txt"

static FILE *s_file = NULL;
static SemaphoreHandle_t s_mutex = NULL;

void SD_Log_Init(void)
{
    if (SDCard_Size == 0) return;   // keine Karte gemountet

    s_mutex = xSemaphoreCreateMutex();
    s_file = fopen(SD_LOG_PATH, "a");
    if (!s_file) return;

    datetime_t now;
    PCF85063_Read_Time(&now);
    fprintf(s_file, "\r\n=== Start %04d-%02d-%02d %02d:%02d:%02d ===\r\n",
            now.year, now.month, now.day, now.hour, now.minute, now.second);
    fflush(s_file);
}

void SD_Log(const char *fmt, ...)
{
    if (!s_file || !s_mutex) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fprintf(s_file, "[%lu] ", (unsigned long)(esp_timer_get_time() / 1000));
    va_list ap;
    va_start(ap, fmt);
    vfprintf(s_file, fmt, ap);
    va_end(ap);
    fputs("\r\n", s_file);
    fflush(s_file);
    xSemaphoreGive(s_mutex);
}
