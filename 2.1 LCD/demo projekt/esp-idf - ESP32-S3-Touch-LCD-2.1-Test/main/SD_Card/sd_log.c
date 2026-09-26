#include "sd_log.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "SD_MMC.h"
#include "PCF85063/PCF85063.h"

#define SD_LOG_PATH   "/sdcard/obd_log.txt"
#define SD_LOG_BUF    16384   // RAM-Puffer, wird vom Schreib-Task geleert
#define SD_LOG_LINE   200     // max. Laenge einer Zeile inkl. Zeitstempel/CRLF
#define SD_LOG_FLUSH_MS 300

static FILE *s_file = NULL;
static SemaphoreHandle_t s_mutex = NULL;
static char s_buf[SD_LOG_BUF];
static size_t s_len = 0;
static uint32_t s_dropped = 0;

// Schreibt den RAM-Puffer auf die Karte. SD-Zugriffe (fsync) dauern teils
// 10-100 ms und duerfen den Bluetooth-Stack nicht blockieren - deshalb
// laufen sie hier im eigenen Task und nicht im Aufrufer von SD_Log().
static void sd_log_task(void *arg)
{
    (void)arg;
    static char local[SD_LOG_BUF];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SD_LOG_FLUSH_MS));

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        size_t n = s_len;
        uint32_t dropped = s_dropped;
        if (n) memcpy(local, s_buf, n);
        s_len = 0;
        s_dropped = 0;
        xSemaphoreGive(s_mutex);

        if (n) {
            fwrite(local, 1, n, s_file);
            if (dropped) fprintf(s_file, "[log] %lu Zeilen verworfen (Puffer voll)\r\n", (unsigned long)dropped);
            fflush(s_file);
            fsync(fileno(s_file));
        }
    }
}

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
    fsync(fileno(s_file));

    xTaskCreatePinnedToCore(sd_log_task, "sd_log", 4096, NULL, 2, NULL, 1);
}

void SD_Log(const char *fmt, ...)
{
    if (!s_file || !s_mutex) return;

    char line[SD_LOG_LINE];
    int n = snprintf(line, sizeof(line), "[%lu] ", (unsigned long)(esp_timer_get_time() / 1000));
    va_list ap;
    va_start(ap, fmt);
    int m = vsnprintf(line + n, sizeof(line) - n - 2, fmt, ap);
    va_end(ap);
    if (m < 0) return;
    n += m;
    if (n > (int)sizeof(line) - 3) n = sizeof(line) - 3;
    line[n++] = '\r';
    line[n++] = '\n';

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_len + n <= SD_LOG_BUF) {
        memcpy(s_buf + s_len, line, n);
        s_len += n;
    } else {
        s_dropped++;
    }
    xSemaphoreGive(s_mutex);
}
