// Symbol-Icons fuer die 4 Zusatzfelder der Multi-Kachel (Batterie/Gaspedal/
// Drehzahl/Wasser), RGB565+Alpha, 3 Byte/Pixel, 22x22 Pixel.
// Rohdaten in field_icons.c, aus Python/Pillow generiert.
#pragma once

#include <lvgl.h>

extern const uint8_t icon_battery_map[22 * 22 * 3];
extern const lv_img_dsc_t icon_battery_img;

extern const uint8_t icon_pedal_map[22 * 22 * 3];
extern const lv_img_dsc_t icon_pedal_img;

extern const uint8_t icon_tacho_map[22 * 22 * 3];
extern const lv_img_dsc_t icon_tacho_img;

extern const uint8_t icon_water_map[22 * 22 * 3];
extern const lv_img_dsc_t icon_water_img;
