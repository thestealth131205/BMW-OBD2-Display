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

/* 64 KB reichten für dieses UI nicht aus (5 Meter mit je ~19 Zonen-/Skala-
 * Indikatoren, Settings- und DTC-Screen, 20 Farb-Buttons usw.) - der LVGL-
 * Heap lief über, fehlgeschlagene Allocs führten zu leeren/falschen Labels
 * ("F" statt Zahlen, fehlende Settings-Werte, falsche Hintergrundfarben).
 * ESP32-C6 hat 512 KB SRAM, daher genug Reserve für einen größeren Pool. */
#define LV_MEM_SIZE (256U * 1024U)

#define LV_USE_LOG 0

/* Benötigte Widgets */
#define LV_USE_METER      1
#define LV_USE_TILEVIEW   1
#define LV_USE_LABEL      1
#define LV_USE_BTN        1
#define LV_USE_DROPDOWN   1
#define LV_USE_LINE       1

/* Layout für die vertikal scrollbaren Farb-Buttons in den Einstellungen */
#define LV_USE_FLEX       1

/* Standard-Font für Labels/Buttons */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT       &lv_font_montserrat_14

/* Großer Font für die digitalen Gauge-Anzeigen (Tacho-Style) */
#define LV_FONT_MONTSERRAT_48 1

#endif /* LV_CONF_H */
