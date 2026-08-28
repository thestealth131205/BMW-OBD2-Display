/*
 * Minimal LVGL v8 Konfiguration für das BMW E90 AMOLED Gauge Display.
 * Nicht gesetzte Optionen fallen auf die Standardwerte aus
 * lvgl/src/lv_conf_internal.h zurück.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

#define LV_COLOR_DEPTH     16
#define LV_COLOR_16_SWAP   0

/* Zeitbasis über Arduino millis() */
#define LV_TICK_CUSTOM             1
#define LV_TICK_CUSTOM_INCLUDE     "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

#define LV_MEM_SIZE (64U * 1024U)

#define LV_USE_LOG 0

/* Benötigte Widgets */
#define LV_USE_METER      1
#define LV_USE_TILEVIEW   1
#define LV_USE_LABEL      1
#define LV_USE_BTN        1

/* Standard-Font für Labels/Buttons */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT       &lv_font_montserrat_14

#endif /* LV_CONF_H */
