// Nadel-Bilder (Tachonadel-Assets, RGB565+Alpha, 3 Byte/Pixel).
// Rohdaten in src/needle_imgs.cpp, aus vom Nutzer bereitgestellten PNGs generiert.
// Jedes Bild ist eng um die sichtbaren Pixel zugeschnitten; das Rotationszentrum
// (Nadel-Drehpunkt = Meter-Mittelpunkt) liegt bei (pivot_x, pivot_y) relativ zur
// linken oberen Ecke des zugeschnittenen Bildes (kann ausserhalb des Bildes liegen).
#pragma once

#include <lvgl.h>

extern const uint8_t needle_teal_map[72036];
extern const lv_img_dsc_t needle_teal_img;
#define NEEDLE_TEAL_PIVOT_X 43
#define NEEDLE_TEAL_PIVOT_Y 50

extern const uint8_t needle_yellow_map[72036];
extern const lv_img_dsc_t needle_yellow_img;
#define NEEDLE_YELLOW_PIVOT_X 43
#define NEEDLE_YELLOW_PIVOT_Y 50

extern const uint8_t needle_red_map[72036];
extern const lv_img_dsc_t needle_red_img;
#define NEEDLE_RED_PIVOT_X 43
#define NEEDLE_RED_PIVOT_Y 50

extern const uint8_t multi_needle_red_map[4752];
extern const lv_img_dsc_t multi_needle_red_img;
#define MULTI_NEEDLE_RED_PIVOT_X 7
#define MULTI_NEEDLE_RED_PIVOT_Y -138

extern const uint8_t multi_needle_yellow_map[4752];
extern const lv_img_dsc_t multi_needle_yellow_img;
#define MULTI_NEEDLE_YELLOW_PIVOT_X 7
#define MULTI_NEEDLE_YELLOW_PIVOT_Y -138

extern const uint8_t multi_needle_teal_map[4752];
extern const lv_img_dsc_t multi_needle_teal_img;
#define MULTI_NEEDLE_TEAL_PIVOT_X 7
#define MULTI_NEEDLE_TEAL_PIVOT_Y -138

// Kleine BMW-Style-Nadel (dunkler Rundknopf + farbiger Schaft, aus vom Nutzer
// bereitgestellten Fotos des echten BMW-Tachozeigers) - ausschliesslich fuer
// die zweite Wassertemperatur-Nadel (multi_bmw_water_needle in bmw_ui.c).
// 37x66 -> 52x92 hochskaliert (Faktor 1.4, Nutzer-Rueckmeldung Foto v1.0.78:
// Nadel war zu klein, ging im Gewirr der gelben "HIGH TEMP"-Skala unter).
// Pivot proportional mitskaliert (20,17 -> 28,24).
extern const uint8_t needle_small_teal_map[14352];
extern const lv_img_dsc_t needle_small_teal_img;
#define NEEDLE_SMALL_TEAL_PIVOT_X 28
#define NEEDLE_SMALL_TEAL_PIVOT_Y 24

extern const uint8_t needle_small_yellow_map[14352];
extern const lv_img_dsc_t needle_small_yellow_img;
#define NEEDLE_SMALL_YELLOW_PIVOT_X 28
#define NEEDLE_SMALL_YELLOW_PIVOT_Y 24

extern const uint8_t needle_small_red_map[14352];
extern const lv_img_dsc_t needle_small_red_img;
#define NEEDLE_SMALL_RED_PIVOT_X 28
#define NEEDLE_SMALL_RED_PIVOT_Y 24

