#include "bmw_ui.h"
#include "multi_bg_img.h"
#include "multi_bg_bmw_img.h"
#include "needle_imgs.h"
#include "field_icons.h"
#include "can_obd2.h"
#include "ble_obd.h"
#include "sntp_sync.h"
#include "service_funcs.h"
#include "led_ctrl.h"
#include "PCF85063.h"
#include "sd_log.h"
#include "SD_MMC.h"
#include "Buzzer.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"

// --- Farbpalette fuer die Einstellungen (Primaer-/Sekundaerfarbe, wie im
// C6-Projekt inkl. Neongelb) ---
typedef struct {
    const char *name;
    lv_color_t color;
} ColorOption;

static const ColorOption COLOR_PALETTE[] = {
    {"Blau",     LV_COLOR_MAKE(0, 162, 255)},
    {"Rot",      LV_COLOR_MAKE(255, 0, 0)},
    {"Gruen",    LV_COLOR_MAKE(0, 230, 64)},
    {"Gelb",     LV_COLOR_MAKE(255, 214, 0)},
    {"Neongelb", LV_COLOR_MAKE(224, 255, 0)},
    {"Orange",   LV_COLOR_MAKE(255, 140, 0)},
    {"Pink",     LV_COLOR_MAKE(255, 0, 144)},
    {"Lila",     LV_COLOR_MAKE(170, 0, 255)},
    {"Cyan",     LV_COLOR_MAKE(0, 255, 220)},
    {"Weiss",    LV_COLOR_MAKE(255, 255, 255)},
};
static const int COLOR_PALETTE_SIZE = sizeof(COLOR_PALETTE) / sizeof(COLOR_PALETTE[0]);

// Dieselbe Reihenfolge/Namen wie COLOR_PALETTE, aber mit vollen 8-Bit-RGB-
// Werten statt der auf RGB565 (LV_COLOR_DEPTH=16) gestutzten lv_color_t -
// der LED-Control-Screen (led_ctrl.c) steuert einen WS2812-Streifen, der die
// volle 24-Bit-Praezision tatsaechlich darstellen kann, anders als das
// Display selbst. Nur fuer die LED-Farbwahl genutzt, die Swatch-Buttons im
// UI selbst nutzen weiterhin COLOR_PALETTE[].color.
typedef struct { uint8_t r, g, b; } RgbColor;
static const RgbColor COLOR_PALETTE_RGB[] = {
    {0, 162, 255},   // Blau
    {255, 0, 0},     // Rot
    {0, 230, 64},    // Gruen
    {255, 214, 0},   // Gelb
    {224, 255, 0},   // Neongelb
    {255, 140, 0},   // Orange
    {255, 0, 144},   // Pink
    {170, 0, 255},   // Lila
    {0, 255, 220},   // Cyan
    {255, 255, 255}, // Weiss
};

// Primaerfarbe (Geschwindigkeitsanzeige) / Sekundaerfarbe (Nadel-Basisfarbe,
// solange die Wassertemperatur unter 95 Grad liegt) - per Einstellungs-Screen
// veraenderbar, Standardwerte wie im C6-Projekt.
static lv_color_t g_color_primary   = LV_COLOR_MAKE(0, 162, 255);  // Blau
static lv_color_t g_color_secondary = LV_COLOR_MAKE(255, 0, 0);    // Rot

// --- Datenquellen-Auswahl (per gedrückt-halten-Menue umschaltbar):
// Standardmaessig BLE-OBD2 (z.B. Veepeak OBDCheck BLE), alternativ das
// verdrahtete MCP2515-CAN-Modul. ---
typedef enum {
    DATA_SRC_BLE_OBD,
    DATA_SRC_MCP2515,
} data_source_t;
static data_source_t g_data_source = DATA_SRC_BLE_OBD;

// --- Live-Werte (ohne CAN: Platzhalter; Batterie kommt live vom Sensor) ---
static float current_speed_kmh       = 0.0f;
static float current_rpm             = 800.0f;   // Leerlauf
static float current_bat_voltage     = 12.6f;
static float current_throttle_pct    = 0.0f;
static float current_water_temp      = 105.0f;

// --- Start-Testanimation (Anzeigen wandern von 0 auf Anschlag und zurueck) ---
#define ANIM_DELAY_MS       2000    // Wartezeit vor Animationsstart
#define ANIM_UP_MS          4000    // 0 -> Anschlag
#define ANIM_DOWN_MS        2000    // Anschlag -> 0
#define ANIM_TOTAL_MS       (ANIM_DELAY_MS + ANIM_UP_MS + ANIM_DOWN_MS)

#define ANIM_SPEED_MAX      260.0f
#define ANIM_WATER_MAX      105.0f
#define ANIM_BAT_MAX         16.0f
#define ANIM_THROTTLE_MAX   100.0f
#define ANIM_RPM_MAX       8000.0f

// Skala der Multi-Nadel (zeigt die Kuehlmitteltemperatur, nicht die Drehzahl -
// die Drehzahl wird bereits digital + ueber die Schaltanzeige-Kaestchen
// dargestellt). Ende der Skala = 119 Grad, ab 95 Grad faerbt sich die Nadel
// lila (siehe BMW_UI_Update()).
#define MULTI_WATER_SCALE_MIN   40
#define MULTI_WATER_SCALE_MAX  119

// Farbring (3 Bogen-Indikatoren auf derselben Hilfsskala wie die Nadel, siehe
// multi_needle_scale): waechst mit der Temperatur mit, Neongelb bis 100 Grad,
// ab 100 Grad Orange (erst von dieser Stelle an, kein Verblassen davor), ab
// 115 Grad Rot.
#define MULTI_WATER_RING_ORANGE_AT 100
#define MULTI_WATER_RING_RED_AT    115

// Derselbe Farbring im Drehzahl-Modus der grossen Anzeige: Neongelb bis
// 6500 U/min, ab 6500 Orange, ab 6900 Rot (Skala 0-8000 U/min).
#define MULTI_RPM_RING_ORANGE_AT  6500
#define MULTI_RPM_RING_RED_AT     6900

// Winkelbereich der unteren blauen Wasser-Leuchtskala + der zweiten kleinen
// Wasser-Nadel (BMW-Style, Vorbild: "HIGH TEMP"-Bogen unten im Foto-
// Hintergrund multi_bg_bmw_img): 110 Grad breiter Bogen unten (Mitte bei
// LVGL-Skalenwinkel 90 = 6 Uhr; LVGL-Winkel 0 liegt bei 3 Uhr und zaehlt im
// Uhrzeigersinn), WMIR() spiegelt den Temperaturwert, damit er entgegen der
// LVGL-Standardrichtung (im Uhrzeigersinn) waechst - mit steigender
// Temperatur wandert der Bogen von unten-links (kalt) ueber unten (6 Uhr)
// nach unten-rechts (heiss), also gegen den Uhrzeigersinn, wie im
// Vorbildfoto. Fuer Bogen-Indikatoren wird der gespiegelte Wert direkt als
// Skalenwert verwendet (rotation=35, range=110 -> Skalenwinkel 35..145,
// Mitte bei 90 = 6 Uhr). Zuvor stand hier rotation=125 (Mitte bei
// Skalenwinkel 180 = 9 Uhr, also links statt unten) - sichtbar als Bogen
// zwischen 7 und 11 Uhr links der Bildschirmmitte, vom Nutzer gemeldet und
// hier auf die im Kommentar immer schon beschriebene untere Position
// korrigiert. MULTI_WATER_NEEDLE_ROTATION folgt mit demselben Versatz
// (-90 Grad gegenueber der vorherigen, durch mehrere "Needle corrections"-
// Commits von Hand am Geraet eingemessenen Nadel-Rotation -50, also
// -50-90=-140), damit die kleine Nadel weiterhin exakt auf den Bogen zeigt.
// Feinjustierung (Nutzer-Rueckmeldung am Board, Foto v1.0.68): Der blaue Bogen
// soll etwas gegen den Uhrzeigersinn gedreht werden -> ARC_ROTATION 35 -> 27
// (kleinere Startwinkel = Bogen rueckt CCW). Die kleine Wasser-Nadel drehte
// zu weit gegen den Uhrzeigersinn aus der Skala heraus -> NEEDLE_ROTATION
// im Uhrzeigersinn zurueckgeholt (-140 -> -112). Beide Werte sind weiterhin
// per Augenmass gesetzt (Nadel-Meter und Bogen-Meter haben unterschiedliche
// Mittelpunkte), der exakte Feinabgleich erfolgt wie bisher iterativ am Geraet.
#define MULTI_WATER_ARC_RANGE       110
#define MULTI_WATER_ARC_ROTATION     27
#define MULTI_WATER_NEEDLE_ROTATION -112
#define WMIR(v) (int32_t)(MULTI_WATER_SCALE_MIN + MULTI_WATER_SCALE_MAX - (v))

static uint32_t anim_start_tick;
static bool anim_done = false;

// --- UI-Objekte ---
static lv_obj_t *scr_multi;
static lv_obj_t *scr_demo;
static lv_obj_t *scr_settings;
static lv_obj_t *scr_settings_func;
static lv_obj_t *scr_dtc;
static lv_obj_t *scr_service;
static lv_obj_t *service_status_label;
static lv_obj_t *scr_sensors;
static lv_obj_t *sens_lambda1_val, *sens_lambda2_val;
static lv_obj_t *sens_lambda1_volt_val, *sens_lambda2_volt_val;
static lv_obj_t *sens_map_val, *sens_boost_val;
static lv_obj_t *sens_cam_in_val, *sens_cam_ex_val;
static lv_obj_t *sens_oil_val, *sens_mil_val;
static lv_obj_t *scr_led;
static lv_obj_t *led_title_label;
static lv_obj_t *led_strip1_tab_btn, *led_strip2_tab_btn;
static lv_obj_t *led_enable_switch;
static lv_obj_t *led_count_label;
static lv_obj_t *led_count_slider;
static lv_obj_t *led_brightness_label;
static lv_obj_t *led_brightness_slider;
static lv_obj_t *led_speed_label;
static lv_obj_t *led_speed_slider;
static lv_obj_t *led_pattern_solid_btn, *led_pattern_chase_btn;
static lv_obj_t *led_speed_row; // Container (Label+Slider) nur im Chase-Muster sichtbar
static lv_obj_t *led_main_color_swatch, *led_second_color_swatch; // zeigen die aktuelle Farbe, oeffnen per Tipp das Popup
static lv_obj_t *led_color_popup_bg; // Vollbild-Overlay mit Farbraster, nur waehrend geoeffnet != NULL
static lv_obj_t *led_conflict_label; // Warnung: Streifen 2 + MCP2515 teilen sich GPIO19

static lv_obj_t *multi_meter;
static lv_meter_scale_t *multi_needle_scale;
static lv_meter_scale_t *multi_ring_scale;
static lv_meter_indicator_t *multi_needle;
static lv_meter_indicator_t *multi_ring_yellow;
static lv_meter_indicator_t *multi_ring_orange;
static lv_meter_indicator_t *multi_ring_red;

// Zusaetzliche, immer aktive Leuchtskala im inneren Kreis (Vorbild: die
// segmentierte Kuehlmitteltemperatur-Anzeige im BMW-Kombiinstrument) - zeigt
// anders als der Haupt-Ring IMMER die Wassertemperatur, auch wenn die grosse
// Ring/Nadel-Anzeige per Einstellungs-Screen auf Drehzahl umgeschaltet ist.
// Eigene, fest auf den Wasserbereich eingestellte Skala statt der
// umschaltbaren multi_ring_scale.
static lv_meter_scale_t *multi_water_inner_scale;
static lv_meter_indicator_t *multi_water_inner_blue;
static lv_meter_indicator_t *multi_water_inner_orange;
static lv_meter_indicator_t *multi_water_inner_red;

// Grosse Anzeige (Ring+Nadel) in der Multi-Kachel: per Einstellungs-Screen
// umschaltbar zwischen Kuehlmitteltemperatur (Standard) und Drehzahl.
typedef enum {
    MULTI_GAUGE_WATER,
    MULTI_GAUGE_RPM,
} multi_gauge_mode_t;
static multi_gauge_mode_t g_multi_gauge_mode = MULTI_GAUGE_WATER;

// Warnsummer bei Kuehlmitteltemperatur >= 120 Grad, im Einstellungs-Screen
// "Funktionen" ein-/ausschaltbar (Standard: an). g_buzzer_active haelt den
// tatsaechlichen physischen Zustand nach, damit Buzzer_On()/_Off() (I2C-
// Schreibzugriff auf den TCA9554-IO-Expander) nicht bei jedem 100-ms-Tick
// erneut ausgeloest wird, sondern nur bei einem Zustandswechsel.
static bool g_buzzer_enabled = true;
static bool g_buzzer_active  = false;

// Merkt sich den Schalterzustand des Datenloggings (unabhaengig vom aktuell
// offenen log_file, das sich bei jedem Neustart mit neuem Dateinamen oeffnet)
// - wird wie die anderen Einstellungen in NVS gespeichert.
static bool g_datalog_enabled = false;

// LED-Streifen-Einstellungen (LED-Control-Screen), wie die anderen
// Einstellungen in NVS gespeichert. Farben werden direkt als Index in
// COLOR_PALETTE_RGB[] gehalten (nicht als lv_color_t): LV_COLOR_DEPTH=16
// (RGB565) wuerde die 24-Bit-Praezision, die ein WS2812-Streifen tatsaechlich
// darstellen kann, unnoetig auf 5/6/5 Bit je Kanal stutzen.
// Zwei unabhaengig steuerbare Streifen (GPIO0/GPIO19, siehe led_ctrl.h) -
// ein Satz Einstellungen je Streifen, ein Tab-Umschalter im Screen waehlt,
// welcher Satz gerade angezeigt/editiert wird (siehe create_led_screen()).
typedef struct {
    bool enabled;
    led_pattern_t pattern;
    int count;
    uint8_t speed;
    uint8_t brightness;
    uint8_t color_main_idx;
    uint8_t color_second_idx;
} LedStripSettings;

static LedStripSettings g_led[2] = {
    { false, LED_PATTERN_SOLID, 30, 50, 100, 0, 1 }, // Streifen 1 (GPIO0):   Blau/Rot
    { false, LED_PATTERN_SOLID, 30, 50, 100, 0, 1 }, // Streifen 2 (GPIO19):  Blau/Rot
};
static int g_led_active_strip = 0; // 0 = Streifen 1, 1 = Streifen 2 (nur UI-Auswahl)

static void led_apply_enabled(int idx, bool on)      { if (idx == 0) LED_Ctrl_SetEnabled(on);      else LED_Ctrl2_SetEnabled(on); }
static void led_apply_count(int idx, int count)      { if (idx == 0) LED_Ctrl_SetCount(count);     else LED_Ctrl2_SetCount(count); }
static void led_apply_pattern(int idx, led_pattern_t p) { if (idx == 0) LED_Ctrl_SetPattern(p);    else LED_Ctrl2_SetPattern(p); }
static void led_apply_speed(int idx, uint8_t speed)  { if (idx == 0) LED_Ctrl_SetSpeed(speed);     else LED_Ctrl2_SetSpeed(speed); }
static void led_apply_brightness(int idx, uint8_t b) { if (idx == 0) LED_Ctrl_SetBrightness(b);    else LED_Ctrl2_SetBrightness(b); }
static void led_apply_main_color(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx == 0) LED_Ctrl_SetMainColor(r, g, b); else LED_Ctrl2_SetMainColor(r, g, b);
}
static void led_apply_second_color(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx == 0) LED_Ctrl_SetSecondColor(r, g, b); else LED_Ctrl2_SetSecondColor(r, g, b);
}
static lv_obj_t *multi_speed_label;
static lv_obj_t *multi_unit_label; // "KM/H" unter multi_speed_label - im BMW-Style ausgeblendet
static lv_obj_t *multi_bat_label;
static lv_obj_t *multi_throttle_label;
static lv_obj_t *multi_rpm_label;
static lv_obj_t *multi_water_label;
static lv_obj_t *multi_field_icon[4]; // Batterie/Gaspedal/Drehzahl/Wasser, je ueber dem zugehoerigen Feld
// Schwarze Umrandung (8 versetzte Kopien) hinter jedem der 4 Zusatzfelder,
// damit die weisse Schrift auch auf hellem Hintergrund gut lesbar bleibt
static lv_obj_t *multi_bat_outline[8];
static lv_obj_t *multi_throttle_outline[8];
static lv_obj_t *multi_rpm_outline[8];
static lv_obj_t *multi_water_outline[8];

static lv_obj_t *multi_rx_dot;
static lv_obj_t *multi_tx_dot;
#define ACT_DOT_FLASH_MS 120

// --- "BMW Style": alternatives Layout fuer die Multi-Kachel, angelehnt an
// ein Foto eines BMW-Kombiinstruments (Rundinstrument mit Skalen-Teilstrichen
// auf dem wachsenden Farbring, Batterie/km-h/Wasser in kompakten Feldern statt
// der grossen zentralen Geschwindigkeit, zusaetzliche kleine Analognadel nur
// fuer Wassertemperatur unten, Schaltanzeige dazwischen). Per eigenem Schalter
// im Funktionen-Screen umschaltbar (siehe apply_bmw_style()), unabhaengig vom
// bestehenden Wasser-/Drehzahl-Umschalter der grossen Anzeige - letzterer
// bleibt in beiden Stilen aktiv und bestimmt weiterhin, was die grosse
// Nadel/der Ring zeigen.
static bool g_bmw_style_enabled = false;
static lv_obj_t *multi_bg_img_obj; // statisches Hintergrundbild (normaler Stil), im BMW-Style ausgeblendet
static lv_obj_t *multi_bg_bmw_img_obj; // Foto-Hintergrundbild (BMW-Style), im normalen Stil ausgeblendet
static lv_obj_t *multi_bmw_water_meter; // kleines eigenstaendiges Meter-Widget fuer die zweite Wasser-Nadel
static lv_meter_scale_t *multi_bmw_water_needle_scale;
static lv_meter_indicator_t *multi_bmw_water_needle;
#define MULTI_BMW_WATER_METER_SIZE 110

// --- Fehlercode-Screen (per Wisch-Geste erreichbar) ---
static lv_obj_t *dtc_list_label;
static lv_obj_t *dtc_status_label;

// --- Schaltanzeige (6 Fuell-Kaestchen ueber den im Hintergrundbild
// gezeichneten Kaesten): fuellen sich mit steigender Drehzahl (je Kaestchen
// eine eigene Schwelle), ab RPM_SHIFT_BLINK blinken alle gemeinsam wie eine
// digitale Schaltanzeige. Fuellfarben = Umrandungsfarben aus dem
// Hintergrundbild (1-4 weiss/grau, 5 hell-lila, 6 blau). ---
#define RPM_SHIFT_BLINK 6800
static lv_obj_t *rpm_boxes[6];
static const lv_coord_t rpm_box_x[6]  = {-117, -73, -28, 17, 62, 109};
static const int32_t    rpm_box_thr[6] = {1500, 2500, 3500, 4500, 5500, 6500};
static const lv_color_t rpm_box_col[6] = {
    LV_COLOR_MAKE(205, 212, 205), LV_COLOR_MAKE(205, 212, 205),
    LV_COLOR_MAKE(205, 212, 205), LV_COLOR_MAKE(205, 212, 205),
    LV_COLOR_MAKE(190, 85, 246),   // hell-lila
    LV_COLOR_MAKE(70, 95, 235),    // blau
};

// Dunkler Tacho-Hintergrund
static void set_dark_bg(lv_obj_t *obj)
{
    lv_obj_set_style_bg_color(obj, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(obj, lv_color_white(), 0);
    lv_obj_set_style_border_width(obj, 0, 0);
}

// Dunkleres Blau statt des hellen LVGL-Standard-Theme-Blaus, damit weisse
// Beschriftung auf den Buttons besser lesbar ist (betrifft auch den
// gedrueckten Zustand, sonst blitzt beim Antippen wieder das helle Blau auf)
static void set_dark_blue_btn(lv_obj_t *btn)
{
    lv_obj_set_style_bg_color(btn, lv_palette_darken(LV_PALETTE_BLUE, 4), 0);
    lv_obj_set_style_bg_color(btn, lv_palette_darken(LV_PALETTE_BLUE, 3), LV_STATE_PRESSED);
}

// Bild-Nadel um den Meter-Mittelpunkt rotieren (in Ruhestellung nach 6 Uhr).
// Eigene, unsichtbare Hilfsskala mit um -90 versetzter rotation, damit die
// 6-Uhr-Ruhestellung mit der Skalenluecke uebereinstimmt (identische Logik
// wie im C6-Projekt, addImageNeedle()). range/rotation parametrisiert, damit
// dieselbe Funktion auch fuer die kleine Wasser-Nadel mit ihrem eigenen,
// schmalen Bogen unten (siehe MULTI_WATER_ARC_*) genutzt werden kann.
static lv_meter_indicator_t *add_image_needle_ex(lv_obj_t *meter, int32_t min_val, int32_t max_val,
                                              const lv_img_dsc_t *img, lv_coord_t pivot_x, lv_coord_t pivot_y,
                                              uint16_t angle_range, uint16_t rotation,
                                              lv_meter_scale_t **out_scale)
{
    lv_meter_scale_t *needle_scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, needle_scale, min_val, max_val, angle_range, rotation);
    if (out_scale) *out_scale = needle_scale;
    return lv_meter_add_needle_img(meter, needle_scale, img, pivot_x, pivot_y);
}

static lv_meter_indicator_t *add_image_needle(lv_obj_t *meter, int32_t min_val, int32_t max_val,
                                              const lv_img_dsc_t *img, lv_coord_t pivot_x, lv_coord_t pivot_y,
                                              lv_meter_scale_t **out_scale)
{
    return add_image_needle_ex(meter, min_val, max_val, img, pivot_x, pivot_y, 270, 45, out_scale);
}

// Welches der 3 Nadel-Bilder (Tuerkis/Gelb/Rot) zur aktuell eingestellten
// Sekundaerfarbe passt: Blau/Cyan -> Tuerkis, Gelb/Neongelb/Orange -> Gelb,
// alle anderen (inkl. Standard Rot) -> Rot als Fallback (identische Logik
// wie classifySecondaryColor() im C6-Projekt).
static void get_base_needle_img(const lv_img_dsc_t **img, lv_coord_t *pivot_x, lv_coord_t *pivot_y)
{
    for (int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        if (COLOR_PALETTE[i].color.full != g_color_secondary.full) continue;
        const char *name = COLOR_PALETTE[i].name;
        if (strcmp(name, "Blau") == 0 || strcmp(name, "Cyan") == 0) {
            *img = &multi_needle_teal_img; *pivot_x = MULTI_NEEDLE_TEAL_PIVOT_X; *pivot_y = MULTI_NEEDLE_TEAL_PIVOT_Y;
            return;
        }
        if (strcmp(name, "Gelb") == 0 || strcmp(name, "Neongelb") == 0 || strcmp(name, "Orange") == 0) {
            *img = &multi_needle_yellow_img; *pivot_x = MULTI_NEEDLE_YELLOW_PIVOT_X; *pivot_y = MULTI_NEEDLE_YELLOW_PIVOT_Y;
            return;
        }
        break;
    }
    *img = &multi_needle_red_img; *pivot_x = MULTI_NEEDLE_RED_PIVOT_X; *pivot_y = MULTI_NEEDLE_RED_PIVOT_Y;
}

// Wie get_base_needle_img(), aber mit den kleinen BMW-Style-Nadel-Bildern
// (needle_small_*_img, dunkler Rundknopf + farbiger Schaft) statt der
// grossen Multi-Kachel-Variante - fuer die eigenstaendige zweite Wasser-Nadel
// im BMW-Style (siehe apply_bmw_style()). Vormals wurden hier faelschlich die
// grossen needle_*_img (87x276px, fuer die C6-Rundinstrumente gedacht) in
// einem nur 110px grossen Meter-Widget verwendet - das vergroesserte den von
// LVGL fuer die Nadel reservierten Redraw-Bereich weit ueber das Widget
// hinaus und machte den gesamten BMW-Style-Screen spuerbar langsam
// (u.a. verzoegerte/verschluckte Doppeltipps in der Bildschirmmitte).
static void get_small_needle_img(const lv_img_dsc_t **img, lv_coord_t *pivot_x, lv_coord_t *pivot_y)
{
    for (int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        if (COLOR_PALETTE[i].color.full != g_color_secondary.full) continue;
        const char *name = COLOR_PALETTE[i].name;
        if (strcmp(name, "Blau") == 0 || strcmp(name, "Cyan") == 0) {
            *img = &needle_small_teal_img; *pivot_x = NEEDLE_SMALL_TEAL_PIVOT_X; *pivot_y = NEEDLE_SMALL_TEAL_PIVOT_Y;
            return;
        }
        if (strcmp(name, "Gelb") == 0 || strcmp(name, "Neongelb") == 0 || strcmp(name, "Orange") == 0) {
            *img = &needle_small_yellow_img; *pivot_x = NEEDLE_SMALL_YELLOW_PIVOT_X; *pivot_y = NEEDLE_SMALL_YELLOW_PIVOT_Y;
            return;
        }
        break;
    }
    *img = &needle_small_red_img; *pivot_x = NEEDLE_SMALL_RED_PIVOT_X; *pivot_y = NEEDLE_SMALL_RED_PIVOT_Y;
}

// --- Einstellungen dauerhaft speichern (NVS-Flash, ueberlebt Stromverluste,
// unabhaengig von der SD-Karte): Primaer-/Sekundaerfarbe, Datenquelle (BLE/
// MCP), Datenlogging an/aus, Anzeige-Umschalter Wasser/Drehzahl, Warnsummer
// an/aus. nvs_flash_init() laeuft bereits in Wireless_Init() (main.c), lange
// bevor BMW_UI_Init() aufgerufen wird. ---
#define BMW_SETTINGS_NVS_NS "bmw_set"

// Pfad muss zu WIFI_CONFIG_PATH in sntp_sync.c / OTA_WEB_WIFI_CONFIG_PATH in
// ota_web.c passen - bewusst hier dupliziert statt ueber einen Header geteilt
// (gleiches Muster wie dort). Das Standard-Layout (g_bmw_style_enabled) wird
// zusaetzlich zu NVS als "LAYOUT=NORMAL"/"LAYOUT=BMW" in dieser Datei
// gespiegelt, damit es sich auch ueber die OTA-Weboberflaeche (Dropdown +
// Speichern-Button) aendern laesst, ohne das Geraet anzufassen.
#define BMW_UI_WIFI_CONFIG_PATH "/sdcard/wifi-einstellungen.txt"
#define BMW_UI_WIFI_CFG_MAX_LINES 40

// Zeilenpuffer fuer save_startup_layout_to_sd()/save_startup_datalogging_to_sd()
// - bewusst dynamisch (malloc/free) statt wie zuvor "static": ein statischer
// 40x160-Puffer (6,4 KB) reserviert diesen Speicher dauerhaft ab dem
// allerersten Boot im .bss-Segment, noch bevor WiFi/BLE/LCD ueberhaupt
// laufen. Zusammen mit dem ebenfalls neuen "static"-Puffer in sd_update.c
// hat das das verfuegbare interne RAM beim Boot um ~10,5 KB geschrumpft und
// den LCD-Treiber beim Anlegen seiner DMA-Puffer in ESP_ERR_NO_MEM/abort()
// laufen lassen (schwarzer Bildschirm seit v1.0.71) - diese Funktionen
// laufen ohnehin nur kurz und synchron im LVGL-Event-Callback, der Speicher
// wird direkt danach wieder freigegeben.
#define BMW_UI_WIFI_CFG_LINE_LEN 160

// Schreibt den aktuellen g_bmw_style_enabled-Zustand als LAYOUT=NORMAL/BMW in
// die Hotspot-Konfigurationsdatei auf der SD-Karte. Vorhandene Zeilen (SSID/
// PASSWORT/Kommentare) bleiben erhalten - nur LAYOUT=... wird ersetzt bzw. am
// Ende ergaenzt, analog zu write_wifi_config() in ota_web.c.
static void save_startup_layout_to_sd(void)
{
    if (!SD_EnsureMounted()) {
        return; // keine Karte - NVS (siehe bmw_settings_save()) bleibt die einzige Quelle
    }

    char (*lines)[BMW_UI_WIFI_CFG_LINE_LEN] = malloc(BMW_UI_WIFI_CFG_MAX_LINES * BMW_UI_WIFI_CFG_LINE_LEN);
    if (!lines) {
        SD_Log("BMW_UI: kein Speicher fuer Konfigurationsdatei-Zeilenpuffer");
        return;
    }

    int line_count = 0;
    FILE *f = fopen(BMW_UI_WIFI_CONFIG_PATH, "r");
    if (f) {
        while (line_count < BMW_UI_WIFI_CFG_MAX_LINES &&
               fgets(lines[line_count], sizeof(lines[0]), f)) {
            line_count++;
        }
        fclose(f);
    }

    f = fopen(BMW_UI_WIFI_CONFIG_PATH, "w");
    if (!f) {
        SD_Log("BMW_UI: Konfigurationsdatei fuer Standard-Layout nicht beschreibbar (errno=%d: %s)",
               errno, strerror(errno));
        free(lines);
        return;
    }

    const char *layout = g_bmw_style_enabled ? "BMW" : "NORMAL";
    bool layout_written = false;
    for (int i = 0; i < line_count; i++) {
        char *p = lines[i];
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, "LAYOUT=", 7) == 0) {
            fprintf(f, "LAYOUT=%s\r\n", layout);
            layout_written = true;
        } else {
            fputs(lines[i], f);
        }
    }
    if (!layout_written) fprintf(f, "LAYOUT=%s\r\n", layout);

    fflush(f);
    fsync(fileno(f));
    fclose(f);
    free(lines);
}

// Liest LAYOUT=NORMAL/BMW aus derselben Konfigurationsdatei, rein lesend und
// ohne sie bei Fehlen anzulegen (das macht sntp_sync.c bereits beim ersten
// "Hotspot verbinden"-Versuch). Ueberschreibt bei vorhandenem, gueltigem Wert
// den aus NVS geladenen Zustand - so wird eine ueber die OTA-Weboberflaeche
// geaenderte Layout-Auswahl auch wirksam, wenn nie am Touch umgeschaltet wurde.
static void load_startup_layout_from_sd(void)
{
    if (!SD_EnsureMounted()) {
        return;
    }
    FILE *f = fopen(BMW_UI_WIFI_CONFIG_PATH, "r");
    if (!f) {
        return;
    }
    char line[160];
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, "LAYOUT=", 7) != 0) continue;
        p += 7;
        char *nl = strpbrk(p, "\r\n");
        if (nl) *nl = '\0';
        if (strcmp(p, "BMW") == 0) {
            g_bmw_style_enabled = true;
        } else if (strcmp(p, "NORMAL") == 0) {
            g_bmw_style_enabled = false;
        }
        break;
    }
    fclose(f);
}

// Schreibt den aktuellen g_datalog_enabled-Zustand als DATALOGGING=1/0 in die
// Hotspot-Konfigurationsdatei auf der SD-Karte - analog zu
// save_startup_layout_to_sd() oben, nur mit dem anderen Schluessel.
static void save_startup_datalogging_to_sd(void)
{
    if (!SD_EnsureMounted()) {
        return; // keine Karte - NVS (siehe bmw_settings_save()) bleibt die einzige Quelle
    }

    char (*lines)[BMW_UI_WIFI_CFG_LINE_LEN] = malloc(BMW_UI_WIFI_CFG_MAX_LINES * BMW_UI_WIFI_CFG_LINE_LEN);
    if (!lines) {
        SD_Log("BMW_UI: kein Speicher fuer Konfigurationsdatei-Zeilenpuffer");
        return;
    }

    int line_count = 0;
    FILE *f = fopen(BMW_UI_WIFI_CONFIG_PATH, "r");
    if (f) {
        while (line_count < BMW_UI_WIFI_CFG_MAX_LINES &&
               fgets(lines[line_count], sizeof(lines[0]), f)) {
            line_count++;
        }
        fclose(f);
    }

    f = fopen(BMW_UI_WIFI_CONFIG_PATH, "w");
    if (!f) {
        SD_Log("BMW_UI: Konfigurationsdatei fuer Datenlogging nicht beschreibbar (errno=%d: %s)",
               errno, strerror(errno));
        free(lines);
        return;
    }

    bool datalog_written = false;
    for (int i = 0; i < line_count; i++) {
        char *p = lines[i];
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, "DATALOGGING=", 12) == 0) {
            fprintf(f, "DATALOGGING=%d\r\n", g_datalog_enabled ? 1 : 0);
            datalog_written = true;
        } else {
            fputs(lines[i], f);
        }
    }
    if (!datalog_written) fprintf(f, "DATALOGGING=%d\r\n", g_datalog_enabled ? 1 : 0);

    fflush(f);
    fsync(fileno(f));
    fclose(f);
    free(lines);
}

// Liest DATALOGGING=1/0 aus derselben Konfigurationsdatei, rein lesend und
// ohne sie bei Fehlen anzulegen (das macht SNTP_Sync_EnsureConfigFile() beim
// Boot bereits, siehe main.c). Ueberschreibt bei vorhandenem, gueltigem Wert
// den aus NVS geladenen Zustand - analog zu load_startup_layout_from_sd().
static void load_startup_datalogging_from_sd(void)
{
    if (!SD_EnsureMounted()) {
        return;
    }
    FILE *f = fopen(BMW_UI_WIFI_CONFIG_PATH, "r");
    if (!f) {
        return;
    }
    char line[160];
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, "DATALOGGING=", 12) != 0) continue;
        p += 12;
        char *nl = strpbrk(p, "\r\n");
        if (nl) *nl = '\0';
        if (strcmp(p, "1") == 0) {
            g_datalog_enabled = true;
        } else if (strcmp(p, "0") == 0) {
            g_datalog_enabled = false;
        }
        break;
    }
    fclose(f);
}

static void bmw_settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open(BMW_SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }

    uint8_t pri_idx = 0, sec_idx = 1; // Fallback: Blau/Rot (Standardwerte)
    for (int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        if (COLOR_PALETTE[i].color.full == g_color_primary.full)   pri_idx = (uint8_t)i;
        if (COLOR_PALETTE[i].color.full == g_color_secondary.full) sec_idx = (uint8_t)i;
    }

    nvs_set_u8(h, "col_pri",    pri_idx);
    nvs_set_u8(h, "col_sec",    sec_idx);
    nvs_set_u8(h, "src",        (uint8_t)g_data_source);
    nvs_set_u8(h, "log_en",     g_datalog_enabled ? 1 : 0);
    nvs_set_u8(h, "gauge_mode", (uint8_t)g_multi_gauge_mode);
    nvs_set_u8(h, "bmw_style",  g_bmw_style_enabled ? 1 : 0);
    nvs_set_u8(h, "buzzer_en",  g_buzzer_enabled ? 1 : 0);
    nvs_set_u8(h, "led_en",     g_led[0].enabled ? 1 : 0);
    nvs_set_u8(h, "led_pat",    (uint8_t)g_led[0].pattern);
    nvs_set_u8(h, "led_cnt",    (uint8_t)g_led[0].count);
    nvs_set_u8(h, "led_spd",    g_led[0].speed);
    nvs_set_u8(h, "led_bri",    g_led[0].brightness);
    nvs_set_u8(h, "led_col_m",  g_led[0].color_main_idx);
    nvs_set_u8(h, "led_col_s",  g_led[0].color_second_idx);
    nvs_set_u8(h, "led2_en",    g_led[1].enabled ? 1 : 0);
    nvs_set_u8(h, "led2_pat",   (uint8_t)g_led[1].pattern);
    nvs_set_u8(h, "led2_cnt",   (uint8_t)g_led[1].count);
    nvs_set_u8(h, "led2_spd",   g_led[1].speed);
    nvs_set_u8(h, "led2_bri",   g_led[1].brightness);
    nvs_set_u8(h, "led2_col_m", g_led[1].color_main_idx);
    nvs_set_u8(h, "led2_col_s", g_led[1].color_second_idx);
    nvs_commit(h);
    nvs_close(h);
}

// Vor dem Aufbau der UI aufgerufen (BMW_UI_Init()), damit Farben, Nadelbild,
// Datenquelle, Anzeige-Umschalter und Schalterstellungen gleich beim ersten
// Aufbau den zuletzt gespeicherten Stand zeigen statt der Standardwerte.
static void bmw_settings_load(void)
{
    // READWRITE statt READONLY: der Bootloop-Schutz fuer "log_try" (siehe
    // unten) muss bei Bedarf noch in dieser Funktion zuruecksetzen/erhoehen.
    nvs_handle_t h;
    if (nvs_open(BMW_SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return; // noch nie gespeichert (frischer Flash) - Standardwerte bleiben
    }

    uint8_t v;
    if (nvs_get_u8(h, "col_pri", &v) == ESP_OK && v < COLOR_PALETTE_SIZE) {
        g_color_primary = COLOR_PALETTE[v].color;
    }
    if (nvs_get_u8(h, "col_sec", &v) == ESP_OK && v < COLOR_PALETTE_SIZE) {
        g_color_secondary = COLOR_PALETTE[v].color;
    }
    if (nvs_get_u8(h, "src", &v) == ESP_OK && v <= DATA_SRC_MCP2515) {
        g_data_source = (data_source_t)v;
    }
    if (nvs_get_u8(h, "log_en", &v) == ESP_OK) {
        g_datalog_enabled = (v != 0);
    }
    if (nvs_get_u8(h, "gauge_mode", &v) == ESP_OK && v <= MULTI_GAUGE_RPM) {
        g_multi_gauge_mode = (multi_gauge_mode_t)v;
    }
    if (nvs_get_u8(h, "bmw_style", &v) == ESP_OK) {
        g_bmw_style_enabled = (v != 0);
    }
    // Ueberschreibt den NVS-Wert, falls die SD-Karte eine explizite LAYOUT=
    // Zeile enthaelt (z.B. ueber die OTA-Weboberflaeche gesetzt) - siehe
    // Kommentar an load_startup_layout_from_sd().
    load_startup_layout_from_sd();
    // Dasselbe fuer DATALOGGING= - siehe load_startup_datalogging_from_sd().
    load_startup_datalogging_from_sd();
    if (nvs_get_u8(h, "buzzer_en", &v) == ESP_OK) {
        g_buzzer_enabled = (v != 0);
    }
    if (nvs_get_u8(h, "led_en", &v) == ESP_OK) {
        g_led[0].enabled = (v != 0);
    }
    if (nvs_get_u8(h, "led_pat", &v) == ESP_OK && v <= LED_PATTERN_CHASE) {
        g_led[0].pattern = (led_pattern_t)v;
    }
    if (nvs_get_u8(h, "led_cnt", &v) == ESP_OK && v >= 1 && v <= LED_CTRL_MAX_LEDS) {
        g_led[0].count = v;
    }
    if (nvs_get_u8(h, "led_spd", &v) == ESP_OK) {
        g_led[0].speed = v;
    }
    if (nvs_get_u8(h, "led_bri", &v) == ESP_OK && v <= 100) {
        g_led[0].brightness = v;
    }
    if (nvs_get_u8(h, "led_col_m", &v) == ESP_OK && v < COLOR_PALETTE_SIZE) {
        g_led[0].color_main_idx = v;
    }
    if (nvs_get_u8(h, "led_col_s", &v) == ESP_OK && v < COLOR_PALETTE_SIZE) {
        g_led[0].color_second_idx = v;
    }
    if (nvs_get_u8(h, "led2_en", &v) == ESP_OK) {
        g_led[1].enabled = (v != 0);
    }
    if (nvs_get_u8(h, "led2_pat", &v) == ESP_OK && v <= LED_PATTERN_CHASE) {
        g_led[1].pattern = (led_pattern_t)v;
    }
    if (nvs_get_u8(h, "led2_cnt", &v) == ESP_OK && v >= 1 && v <= LED_CTRL_MAX_LEDS) {
        g_led[1].count = v;
    }
    if (nvs_get_u8(h, "led2_spd", &v) == ESP_OK) {
        g_led[1].speed = v;
    }
    if (nvs_get_u8(h, "led2_bri", &v) == ESP_OK && v <= 100) {
        g_led[1].brightness = v;
    }
    if (nvs_get_u8(h, "led2_col_m", &v) == ESP_OK && v < COLOR_PALETTE_SIZE) {
        g_led[1].color_main_idx = v;
    }
    if (nvs_get_u8(h, "led2_col_s", &v) == ESP_OK && v < COLOR_PALETTE_SIZE) {
        g_led[1].color_second_idx = v;
    }

    // Bootloop-Schutz fuer das wiederhergestellte Datenlogging: In v1.0.39
    // ist bestaetigt (per ELF-Symbolaufloesung des Coredumps), dass
    // deferred_start_datalogging_cb() -> start_datalogging() -> fopen() per
    // newlib abort() abstuerzen kann (__retarget_lock_init_recursive ->
    // lock_init_generic -> abort, vermutlich Heap-Druck kurz nach dem Boot).
    // Da bmw_settings_save() das "an"-Flag unabhaengig vom Erfolg von
    // start_datalogging() speichert, wuerde ein solcher Absturz ohne diesen
    // Schutz bei JEDEM Neustart erneut ausgeloest -> Dauerbootloop, aus dem
    // man ohne Nochmal-Flashen nicht mehr herauskommt. "log_try" zaehlt
    // Boot-Versuche mit aktivem Logging, die NICHT bis zum erfolgreichen
    // fopen() in start_datalogging() ueberlebt haben (das loescht den
    // Zaehler wieder, siehe dort). Nach 2 gescheiterten Versuchen in Folge
    // wird Logging fuer diesen Boot automatisch deaktiviert und dauerhaft
    // ausgeschaltet, statt es erneut zu versuchen.
    uint8_t log_try = 0;
    nvs_get_u8(h, "log_try", &log_try);
    if (g_datalog_enabled) {
        if (log_try >= 2) {
            g_datalog_enabled = false;
            nvs_set_u8(h, "log_en", 0);
            nvs_set_u8(h, "log_try", 0);
            nvs_commit(h);
            SD_Log("WARNUNG: Datenlogging nach %d gescheiterten Boot-Versuchen automatisch "
                   "deaktiviert (Bootloop-Schutz)", (int)log_try);
        } else {
            nvs_set_u8(h, "log_try", (uint8_t)(log_try + 1));
            nvs_commit(h);
        }
    }
    nvs_close(h);

    SD_Log("Einstellungen aus NVS geladen: Quelle=%s Logging=%d Anzeige=%s Summer=%d "
           "(freier Heap=%u, groesster 8-Bit-Block=%u)",
           g_data_source == DATA_SRC_BLE_OBD ? "BLE" : "MCP", (int)g_datalog_enabled,
           g_multi_gauge_mode == MULTI_GAUGE_RPM ? "Drehzahl" : "Wasser", (int)g_buzzer_enabled,
           (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

// Loescht den Bootloop-Schutz-Zaehler ("log_try"), nachdem start_datalogging()
// tatsaechlich bis zum erfolgreichen fopen() durchgelaufen ist - erst dann
// gilt dieser Boot-Versuch als ueberlebt.
static void clear_datalog_boot_counter(void)
{
    nvs_handle_t h;
    if (nvs_open(BMW_SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, "log_try", 0);
    nvs_commit(h);
    nvs_close(h);
}

// 3-Sekunden-Halten in der Bildschirmmitte -> Demo-Seite anzeigen,
// Doppeltipp in der Mitte -> Einstellungs-Screen (Farben) anzeigen.
static lv_obj_t *center_hold;

static void center_touch_cb(lv_event_t *e)
{
    static uint32_t press_start = 0;
    static bool triggered = false;
    static bool long_press_fired = false;
    static uint32_t last_click_tick = 0;
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED) {
        press_start = lv_tick_get();
        triggered = false;
    } else if (code == LV_EVENT_PRESSING) {
        if (!triggered && scr_demo && lv_tick_elaps(press_start) >= 3000) {
            triggered = true;
            long_press_fired = true;
            lv_scr_load(scr_demo);
        }
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        triggered = false;
    } else if (code == LV_EVENT_CLICKED) {
        if (long_press_fired) {
            // Klick, der aus dem 3-Sekunden-Halten resultiert -> nicht als
            // (erster) Doppeltipp-Klick werten.
            long_press_fired = false;
        } else if (last_click_tick != 0 && lv_tick_elaps(last_click_tick) < 400) {
            last_click_tick = 0;
            if (scr_settings) {
                lv_scr_load(scr_settings);
            }
        } else {
            last_click_tick = lv_tick_get();
        }
    }
}

// "Zurueck"-Button auf der Demo-Seite -> zurueck zur BMW-Multi-Ansicht
static void back_to_multi_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load(scr_multi);
}

// --- Datenquellen-Auswahl (BLE-OBD2 / MCP2515) im gedrueckt-halten-Menue ---
static lv_obj_t *btn_src_ble;
static lv_obj_t *btn_src_mcp;

static void update_source_btn_styles(void)
{
    lv_obj_set_style_bg_color(btn_src_ble,
        g_data_source == DATA_SRC_BLE_OBD ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_color(btn_src_mcp,
        g_data_source == DATA_SRC_MCP2515 ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_GREY), 0);
}

static void src_ble_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    g_data_source = DATA_SRC_BLE_OBD;
    update_source_btn_styles();
    bmw_settings_save();
}

static void src_mcp_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    g_data_source = DATA_SRC_MCP2515;
    update_source_btn_styles();
    bmw_settings_save();
}

// --- Einstellungs-Screen (Farben) ---
typedef struct {
    int index;
    bool is_primary;
} ColorBtnCtx;

static ColorBtnCtx primary_color_ctx[sizeof(COLOR_PALETTE) / sizeof(COLOR_PALETTE[0])];
static ColorBtnCtx secondary_color_ctx[sizeof(COLOR_PALETTE) / sizeof(COLOR_PALETTE[0])];

static void color_btn_clicked_cb(lv_event_t *e)
{
    ColorBtnCtx *ctx = (ColorBtnCtx *)lv_event_get_user_data(e);
    lv_color_t chosen = COLOR_PALETTE[ctx->index].color;
    if (ctx->is_primary) {
        g_color_primary = chosen;
        lv_obj_set_style_text_color(multi_speed_label, g_color_primary, 0);
    } else {
        g_color_secondary = chosen;
    }
    bmw_settings_save();
}

// Baut eine vertikal scrollbare Spalte mit Farb-Buttons (mehr Auswahl als
// auf den Bildschirm passt, wie im C6-Projekt).
static void create_color_picker_column(lv_obj_t *parent, const char *title, int x_offset,
                                        bool is_primary, ColorBtnCtx *ctx_array)
{
    lv_obj_t *header = lv_label_create(parent);
    lv_label_set_text(header, title);
    lv_obj_align(header, LV_ALIGN_TOP_MID, x_offset, 45);

    lv_obj_t *list = lv_obj_create(parent);
    set_dark_bg(list);
    lv_obj_set_size(list, 150, 350);
    lv_obj_align(list, LV_ALIGN_TOP_MID, x_offset, 80);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(list, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_style_pad_row(list, 10, 0);

    for (int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        lv_obj_t *btn = lv_btn_create(list);
        lv_obj_set_size(btn, 70, 45);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_style_bg_color(btn, COLOR_PALETTE[i].color, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);

        ctx_array[i].index = i;
        ctx_array[i].is_primary = is_primary;
        lv_obj_add_event_cb(btn, color_btn_clicked_cb, LV_EVENT_CLICKED, &ctx_array[i]);
    }
}

static void back_from_settings_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load(scr_multi);
}

// --- Datenlogging auf SD-Karte (CSV, Excel-kompatibel: Semikolon-Trenner,
// CRLF-Zeilenenden, Dezimalpunkt) ---
#define LOG_INTERVAL_MS 500
static FILE *log_file = NULL;
static uint32_t log_last_tick = 0;
static lv_obj_t *log_status_label;
static lv_obj_t *log_switch; // auch von sd_format_btn_cb() gebraucht, um den Schalter beim Formatieren zurueckzusetzen

static void start_datalogging(void)
{
    // Karte war beim Boot evtl. nicht eingelegt/nicht gemountet (SDCard_Size
    // blieb 0) - z.B. weil sie erst nach dem Einschalten eingesteckt wurde.
    // Hier erneut versuchen, statt direkt "SD-Fehler!" zu melden, obwohl die
    // Karte jetzt sichtbar im Schacht steckt.
    if (!SD_EnsureMounted()) {
        lv_label_set_text(log_status_label, "SD-Fehler!");
        SD_Log("Datenlogging: SD_EnsureMounted() fehlgeschlagen (Karte fehlt/nicht lesbar)");
        return;
    }
    SD_Log_Init(); // no-op, falls schon beim Boot initialisiert

    datetime_t now;
    PCF85063_Read_Time(&now);

    char path[64];
    snprintf(path, sizeof(path), "/sdcard/log_%04d%02d%02d_%02d%02d%02d.csv",
             now.year, now.month, now.day, now.hour, now.minute, now.second);

    // Heap-Stand unmittelbar vor dem fopen() protokollieren: Genau dieser
    // Aufruf ist in v1.0.39 per Coredump/ELF-Symbolaufloesung als Absturzstelle
    // bestaetigt (fopen -> __sfp -> __retarget_lock_init_recursive ->
    // lock_init_generic -> abort). esp_get_free_heap_size()/MALLOC_CAP_8BIT
    // zaehlen das reichlich vorhandene PSRAM mit rein (siehe gleicher Fund in
    // sntp_sync.c) - fuer den internen Mutex, den fopen() hier allokiert, ist
    // aber nur MALLOC_CAP_INTERNAL relevant. Mit echten Zahlen aus dem Feld
    // statt zu raten.
    SD_Log("Datenlogging: oeffne %s (freier interner Heap=%u, groesster interner Block=%u)",
           path, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    log_file = fopen(path, "w");
    if (!log_file) {
        lv_label_set_text(log_status_label, "SD-Fehler!");
        SD_Log("Datenlogging: fopen(%s) fehlgeschlagen, errno=%d (%s)",
               path, errno, strerror(errno));
        return;
    }
    fprintf(log_file,
            "Zeit_ms;Geschwindigkeit_kmh;Drehzahl_U_min;Wassertemperatur_C;"
            "Gaspedal_pct;Batterie_OBD2_V;GKraft_Quer_g;GKraft_Laengs_g\r\n");
    fflush(log_file);
    // fflush() reicht den stdio-Puffer nur an FatFs weiter - das neue
    // Verzeichnis/die FAT-Tabellenaenderung fuer die frisch angelegte Datei
    // bleibt ohne fsync() im FatFs-internen Cache und wird nie auf die Karte
    // committet, wenn (wie im Auto normal) die Zuendung sofort stromlos
    // schaltet statt ein cleanes Unmount zuzulassen. Exakt dasselbe Problem
    // wurde in sd_log.c bereits per fsync(fileno(...)) behoben - hier bisher
    // vergessen, obwohl es dieselbe FatFs-Instanz ist.
    fsync(fileno(log_file));
    log_last_tick = lv_tick_get();
    lv_label_set_text(log_status_label, "Aktiv");

    // fopen() hat den abschuzgefaehrdeten Punkt diesmal ueberlebt - Bootloop-
    // Schutz-Zaehler fuer den naechsten Boot wieder auf 0 setzen.
    clear_datalog_boot_counter();
}

static void stop_datalogging(void)
{
    if (log_file) {
        // Abschluss-Markierung: Ein spaeter geoeffnetes Log ohne diese Zeile
        // am Ende wurde nicht sauber beendet (z.B. Zuendung aus waehrend des
        // Loggens) - rein diagnostisch, verhindert keine FAT-Inkonsistenz,
        // zeigt aber beim Nachsehen auf dem PC, ob die Datei regulaer
        // geschlossen wurde.
        fprintf(log_file, "# Ende sauber beendet\r\n");
        fflush(log_file);
        fsync(fileno(log_file));
        fclose(log_file);
        log_file = NULL;
    }
    lv_label_set_text(log_status_label, "");
}

// Einmaliger, verzoegerter Start fuer das beim Boot aus NVS wiederhergestellte
// Datenlogging: fopen() kann intern einen neuen FreeRTOS-Mutex fuer die
// FILE-Struktur allokieren (__retarget_lock_init_recursive); schlaegt das
// fehl, ruft newlib abort() -> Bootloop. 1,5s Verzoegerung (v1.0.35/36) hat
// den Absturz in v1.0.39 NICHT verhindert, nur verschoben (per Coredump
// bestaetigt: crasht weiterhin genau hier). Deshalb jetzt zusaetzlich zur
// laengeren Verzoegerung der Bootloop-Schutz in bmw_settings_load()/
// clear_datalog_boot_counter() - der verhindert zumindest, dass ein
// erneuter Absturz an dieser Stelle das Geraet dauerhaft unbrauchbar macht.
static void deferred_start_datalogging_cb(lv_timer_t *timer)
{
    lv_timer_del(timer);
    start_datalogging();
}

static void log_switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    g_datalog_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (g_datalog_enabled) {
        start_datalogging();
    } else {
        stop_datalogging();
    }
    bmw_settings_save();
    save_startup_datalogging_to_sd();
}

// --- SD-Karte formatieren: fuer den Fall, dass die Karte unformatiert/
// beschaedigt ist (SD_EnsureMounted() liefert dann false, "SD-Fehler!" bzw.
// "SD-Karte nicht gefunden" bei Datenlogging/Hotspot) oder einfach neu
// aufgesetzt werden soll. Die eigentliche Formatierung laeuft in einem
// eigenen Task, da esp_vfs_fat_sdcard_format() mehrere hundert ms blockiert
// und sonst den LVGL-Haupttask anhaelt - deshalb darf dieser Task selbst
// keine LVGL-Funktionen aufrufen (nicht threadsicher), genau wie beim
// bestehenden SNTP-Hotspot-Task. Die UI wird stattdessen aus
// BMW_UI_Update() heraus per Polling von s_sd_format_busy aktualisiert.
static lv_obj_t *sd_format_status_label;
static volatile bool s_sd_format_busy = false;
static volatile bool s_sd_format_ok = false;

static void sd_format_task(void *arg)
{
    (void)arg;

    // Der SNTP-Hotspot-Task (sntp_sync.c) liest/schreibt /sdcard/wifi-
    // einstellungen.txt auf demselben FatFs-Volume, das SD_Format() gleich
    // unmountet und neu anlegt - ein offener Handle dort waere danach
    // genauso ungueltig wie der Datenlogging-Handle unten. Laeuft gerade ein
    // Versuch, zuerst abbrechen und auf das Ende warten, statt parallel zu
    // formatieren. SNTP_Sync_Cancel() wirkt nicht sofort - steckt der
    // Versuch gerade im blockierenden esp_netif_sntp_sync_wait() (bis zu
    // SNTP_SYNC_TIMEOUT_MS=8s), greift der Abbruch erst danach - Wartezeit
    // hier daher grosszuegig bemessen.
    if (SNTP_Sync_IsBusy()) {
        SNTP_Sync_Cancel();
        for (int waited = 0; waited < 10000 && SNTP_Sync_IsBusy(); waited += 100) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Ein offener Log-Datei-Handle auf dem gerade neu formatierten
    // Dateisystem waere danach ungueltig - vorher schliessen. Das
    // Datenlogging selbst (log_file/g_datalog_enabled/UI) ist bereits in
    // sd_format_btn_cb() auf dem LVGL-Thread gestoppt worden.
    SD_Log_Deinit();

    esp_err_t err = SD_Format();
    if (err == ESP_OK) {
        SD_Log_Init();
        SD_Log("SD-Karte formatiert (SD_Format), neue Groesse=%lu MB", (unsigned long)SDCard_Size);
    }

    s_sd_format_ok = (err == ESP_OK);
    s_sd_format_busy = false;
    vTaskDelete(NULL);
}

static void sd_format_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_sd_format_busy) return;

    if (g_datalog_enabled) {
        g_datalog_enabled = false;
        lv_obj_clear_state(log_switch, LV_STATE_CHECKED);
        stop_datalogging();
        bmw_settings_save();
    }

    s_sd_format_busy = true;
    s_sd_format_ok = false;
    lv_label_set_text(sd_format_status_label, "Formatiere...");
    xTaskCreatePinnedToCore(sd_format_task, "sd_format", 4096, NULL, 2, NULL, 0);
}

static lv_obj_t *gauge_mode_label;

// Vorwaerts-Deklaration: Definition folgt weiter unten (dort zusammen mit
// den anderen Wisch-Zielen scr_dtc/scr_service dokumentiert), wird aber
// bereits in create_settings_screen()/create_settings_func_screen() fuer
// scr_settings/scr_settings_func gebraucht.
static void swipe_gesture_cb(lv_event_t *e);

// Umschalter fuer die grosse Ring+Nadel-Anzeige der Multi-Kachel: Wasser-
// temperatur (Standard, Skala 40-119 Grad) oder Drehzahl (Skala 0-8000 U/min).
// Der Farbring ist nur im Wasser-Modus relevant, siehe BMW_UI_Update().
static void gauge_mode_switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    bool rpm_mode = lv_obj_has_state(sw, LV_STATE_CHECKED);
    g_multi_gauge_mode = rpm_mode ? MULTI_GAUGE_RPM : MULTI_GAUGE_WATER;
    lv_label_set_text(gauge_mode_label, rpm_mode ? "Anzeige: Drehzahl" : "Anzeige: Wasser");
    if (rpm_mode) {
        lv_meter_set_scale_range(multi_meter, multi_needle_scale, 0, 8000, 270, 45);
        lv_meter_set_scale_range(multi_meter, multi_ring_scale, 0, 8000, 270, 135);
    } else {
        lv_meter_set_scale_range(multi_meter, multi_needle_scale,
                                  MULTI_WATER_SCALE_MIN, MULTI_WATER_SCALE_MAX, 270, 45);
        lv_meter_set_scale_range(multi_meter, multi_ring_scale,
                                  MULTI_WATER_SCALE_MIN, MULTI_WATER_SCALE_MAX, 270, 135);
    }
    bmw_settings_save();
}

// Wendet das BMW-Style-Layout auf die Multi-Kachel an (on=true), oder das
// normale Layout (on=false). Alle referenzierten Widgets existieren bereits,
// wenn dies aufgerufen wird: create_multi_tile() ruft es einmal am Ende auf
// (Startzustand aus NVS), bmw_style_switch_cb() danach bei jedem Umschalten.
static void apply_bmw_style(bool on)
{
    // Hintergrundbild: normaler Stil zeigt den alten synthetischen Hintergrund
    // (multi_bg_img_obj, Ring/Skala/Farbverlauf fest im Bild), BMW-Style zeigt
    // stattdessen das Foto des echten BMW-Kombiinstruments (multi_bg_bmw_img_obj,
    // enthaelt Skala/Teilstriche/Zahlen/Farbzonen bereits fertig eingebrannt -
    // die frueher hier zusaetzlich gezeichneten nativen Teilstriche
    // (ehem. multi_bmw_tick_scale) sind deshalb nicht mehr noetig und entfallen).
    if (multi_bg_img_obj) {
        if (on) lv_obj_add_flag(multi_bg_img_obj, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_clear_flag(multi_bg_img_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (multi_bg_bmw_img_obj) {
        if (on) lv_obj_clear_flag(multi_bg_bmw_img_obj, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_add_flag(multi_bg_bmw_img_obj, LV_OBJ_FLAG_HIDDEN);
    }

    // Beide Hintergrundbilder explizit nach hinten, beide Nadel-Meter
    // explizit nach vorne schieben - unabhaengig von der Erzeugungsreihenfolge
    // muss die Nadel immer ueber dem (je nach Stil sichtbaren) Hintergrundbild
    // liegen, sonst wuerde sie dahinter verschwinden.
    if (multi_bg_img_obj)     lv_obj_move_background(multi_bg_img_obj);
    if (multi_bg_bmw_img_obj) lv_obj_move_background(multi_bg_bmw_img_obj);
    if (multi_meter)          lv_obj_move_foreground(multi_meter);
    if (multi_bmw_water_meter) lv_obj_move_foreground(multi_bmw_water_meter);
    // Das unsichtbare Center-Overlay (3s-Halten/Doppeltipp) muss trotzdem
    // immer ganz oben bleiben - seit v1.0.68 lagen die Nadel-Meter (450x450,
    // in LVGL standardmaessig klickbar) darueber und haben alle Touches in
    // der Mitte abgefangen. Es ist transparent, verdeckt optisch also nichts.
    if (center_hold)          lv_obj_move_foreground(center_hold);

    // Geschwindigkeit: normal gross mittig, im BMW-Style klein oben (Stelle
    // des mittleren gruenen Kreises im Vorbildfoto) - "KM/H" dort ausgeblendet,
    // im Foto steht an dieser Stelle kein Einheitentext.
    if (multi_speed_label) {
        lv_obj_set_style_text_font(multi_speed_label, on ? &lv_font_montserrat_28 : &lv_font_montserrat_48, 0);
        lv_obj_align(multi_speed_label, LV_ALIGN_CENTER, 0, on ? -140 : -60);
    }
    if (multi_unit_label) {
        if (on) lv_obj_add_flag(multi_unit_label, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_clear_flag(multi_unit_label, LV_OBJ_FLAG_HIDDEN);
    }

    // Batterie (Index 0) und Wasser (Index 3) bleiben sichtbar, wandern aber
    // auf Nabenhoehe nach links/rechts (linker/rechter gruener Kreis im
    // Vorbildfoto). Gaspedal (1) und digitale Drehzahl (2) haben dort keine
    // Entsprechung und werden im BMW-Style ausgeblendet.
    static const lv_coord_t field_x_normal[4] = {-165, -55, 55, 165};
    static const lv_coord_t field_x_style[4]  = {-130, -55, 55, 130}; // [1]/[2] ungenutzt (ausgeblendet)
    static const lv_coord_t outline_off2[8][2] = {
        {-2, 0}, {2, 0}, {0, -2}, {0, 2}, {-2, -2}, {2, -2}, {-2, 2}, {2, 2},
    };
    lv_obj_t *field_labels[4]    = {multi_bat_label, multi_throttle_label, multi_rpm_label, multi_water_label};
    lv_obj_t **field_outlines[4] = {multi_bat_outline, multi_throttle_outline, multi_rpm_outline, multi_water_outline};
    for (int f = 0; f < 4; f++) {
        bool keep_visible = (f == 0 || f == 3);
        bool visible = !on || keep_visible;
        lv_coord_t x = on ? field_x_style[f] : field_x_normal[f];
        lv_coord_t y = on ? 0 : 60;
        lv_coord_t icon_y = on ? -28 : 32;

        if (field_labels[f]) {
            if (visible) lv_obj_clear_flag(field_labels[f], LV_OBJ_FLAG_HIDDEN);
            else         lv_obj_add_flag(field_labels[f], LV_OBJ_FLAG_HIDDEN);
            lv_obj_align(field_labels[f], LV_ALIGN_CENTER, x, y);
        }
        if (multi_field_icon[f]) {
            if (visible) lv_obj_clear_flag(multi_field_icon[f], LV_OBJ_FLAG_HIDDEN);
            else         lv_obj_add_flag(multi_field_icon[f], LV_OBJ_FLAG_HIDDEN);
            lv_obj_align(multi_field_icon[f], LV_ALIGN_CENTER, x, icon_y);
        }
        for (int i = 0; i < 8; i++) {
            lv_obj_t *o = field_outlines[f][i];
            if (!o) continue;
            if (visible) lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
            else         lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
            lv_obj_align(o, LV_ALIGN_CENTER, x + outline_off2[i][0], y + outline_off2[i][1]);
        }
    }

    // Schaltanzeige: normal unterhalb des Hintergrundbild-Kastens (y=172), im
    // BMW-Style zwischen Haupt-Nadel-Nabe und der zweiten Wasser-Nadel weiter
    // unten platziert (vom Nutzer so gewuenscht).
    for (int i = 0; i < 6; i++) {
        if (rpm_boxes[i]) {
            lv_obj_align(rpm_boxes[i], LV_ALIGN_CENTER, rpm_box_x[i], on ? 85 : 172);
        }
    }

    // Zweite, kleine Analognadel nur fuer Wassertemperatur (unterer blauer
    // Bereich im Vorbildfoto) - nur im BMW-Style sichtbar, unabhaengig davon,
    // worauf die grosse Ring/Nadel-Anzeige gerade eingestellt ist.
    if (multi_bmw_water_meter) {
        if (on) lv_obj_clear_flag(multi_bmw_water_meter, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_add_flag(multi_bmw_water_meter, LV_OBJ_FLAG_HIDDEN);
    }
}

// Es gibt zwei BMW-Style-Schalter (Funktionen-Screen + Langdruck-/Demo-Screen,
// vom Nutzer gewuenscht, damit der Umschalter auch dann erreichbar ist, wenn
// der BMW-Style-Screen gerade traege reagiert). Beide zeigen denselben Zustand
// g_bmw_style_enabled; nach jedem Umschalten werden beide synchronisiert.
static lv_obj_t *g_bmw_style_sw[2] = {NULL, NULL};

static void sync_bmw_style_switches(void)
{
    for (int i = 0; i < 2; i++) {
        if (!g_bmw_style_sw[i]) continue;
        if (g_bmw_style_enabled) lv_obj_add_state(g_bmw_style_sw[i], LV_STATE_CHECKED);
        else                     lv_obj_clear_state(g_bmw_style_sw[i], LV_STATE_CHECKED);
    }
}

static void bmw_style_switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    g_bmw_style_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    apply_bmw_style(g_bmw_style_enabled);
    sync_bmw_style_switches();
    bmw_settings_save();
    save_startup_layout_to_sd();
}

// Warnsummer bei Kuehlmitteltemperatur >= 120 Grad (siehe BMW_UI_Update()),
// hier nur ein-/ausschaltbar.
static void buzzer_switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    g_buzzer_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (!g_buzzer_enabled && g_buzzer_active) {
        Buzzer_Off();
        g_buzzer_active = false;
    }
    bmw_settings_save();
}

// "Hotspot verbinden"-Schalter: loest einen einmaligen SNTP-Zeitabgleich
// aus (siehe sntp_sync.h), ohne dauerhaft zu speichern - dieser Schalter
// ist bewusst NICHT Teil von bmw_settings_save()/_load(), nach jedem
// Neustart ist er also wieder aus und es wird nie unbeabsichtigt nach
// einem Hotspot gesucht. BMW_UI_Update() schaltet ihn automatisch wieder
// aus, sobald der (einmalige) Versuch fertig ist, siehe dort.
static lv_obj_t *hotspot_switch;
static lv_obj_t *hotspot_status_label;

static void hotspot_switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    if (lv_obj_has_state(sw, LV_STATE_CHECKED)) {
        // Waehrend sd_format_task() formatiert, ist /sdcard kurzzeitig
        // unmounted/neu angelegt - ein zeitgleicher Hotspot-Versuch wuerde
        // versuchen, wifi-einstellungen.txt genau in diesem Fenster zu
        // lesen/anzulegen. Lieber ablehnen als in die gleiche Race-
        // Situation laufen, die sd_format_task() umgekehrt schon vermeidet.
        if (s_sd_format_busy) {
            lv_obj_clear_state(sw, LV_STATE_CHECKED);
            lv_label_set_text(hotspot_status_label, "SD wird formatiert");
            return;
        }
        SNTP_Sync_Start();
    } else {
        SNTP_Sync_Cancel();
    }
}

// Einstellungs-Screen mit Primaer-/Sekundaerfarb-Auswahl, erreichbar per
// Doppeltipp in der Bildschirmmitte der Multi-Ansicht. Wisch nach rechts
// fuehrt zum zweiten Einstellungs-Screen "FUNKTIONEN" (Datenlogging,
// Anzeige-Umschalter, Warnsummer), Wisch nach links geht von dort zurueck.
static void create_settings_screen(void)
{
    scr_settings = lv_obj_create(NULL);
    set_dark_bg(scr_settings);
    lv_obj_clear_flag(scr_settings, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr_settings);
    lv_label_set_text(title, "FARBEN");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    create_color_picker_column(scr_settings, "Primaer", -115, true, primary_color_ctx);
    create_color_picker_column(scr_settings, "Sekundaer", 115, false, secondary_color_ctx);

    lv_obj_t *hint = lv_label_create(scr_settings);
    lv_label_set_text(hint, "Wisch -> Funktionen");
    lv_obj_set_style_text_color(hint, lv_color_make(150, 150, 150), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -58);

    lv_obj_t *btn_back = lv_btn_create(scr_settings);
    set_dark_blue_btn(btn_back);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_back, back_from_settings_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, "Zurueck");
    lv_obj_center(back_lbl);

    lv_obj_add_event_cb(scr_settings, swipe_gesture_cb, LV_EVENT_GESTURE, NULL);
}

// Zweiter Einstellungs-Screen "FUNKTIONEN": Datenlogging, Umschalter
// Wasser-/Drehzahlanzeige (beide aus dem Farben-Screen hierher verschoben,
// da sie keine Farbeinstellungen sind) sowie neu der Warnsummer-Schalter.
// Erreichbar per Wisch nach rechts vom Farben-Screen, Wisch nach links geht
// zurueck.
static void create_settings_func_screen(void)
{
    scr_settings_func = lv_obj_create(NULL);
    set_dark_bg(scr_settings_func);
    lv_obj_clear_flag(scr_settings_func, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr_settings_func);
    lv_label_set_text(title, "FUNKTIONEN");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 35);

    // TOP_MID mit Offset statt TOP_LEFT/TOP_RIGHT an der Kante: Bei diesem
    // y-Abstand von oben liegt die Kante auf dem runden Panel innerhalb der
    // sichtbaren Kreisflaeche zu weit aussen und wird abgeschnitten.
    lv_obj_t *log_label = lv_label_create(scr_settings_func);
    lv_label_set_text(log_label, "Datenlogging");
    lv_obj_align(log_label, LV_ALIGN_TOP_MID, -85, 75);

    log_switch = lv_switch_create(scr_settings_func);
    lv_obj_align(log_switch, LV_ALIGN_TOP_MID, -85, 97);
    lv_obj_add_event_cb(log_switch, log_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    log_status_label = lv_label_create(scr_settings_func);
    lv_label_set_text(log_status_label, "");
    lv_obj_align_to(log_status_label, log_switch, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);

    // Aus NVS geladener Zustand (bmw_settings_load(), vor dem UI-Aufbau
    // aufgerufen): Schalter entsprechend vorbelegen und Logging bei Bedarf
    // starten, statt immer mit "aus" zu beginnen. Der eigentliche fopen()
    // erfolgt verzoegert (siehe deferred_start_datalogging_cb) statt direkt
    // hier mitten im UI-Aufbau, das fuehrte deterministisch zum Bootloop.
    if (g_datalog_enabled) {
        lv_obj_add_state(log_switch, LV_STATE_CHECKED);
        lv_timer_create(deferred_start_datalogging_cb, 4000, NULL);
    }

    gauge_mode_label = lv_label_create(scr_settings_func);
    lv_label_set_text(gauge_mode_label, "Anzeige: Wasser");
    lv_obj_align(gauge_mode_label, LV_ALIGN_TOP_MID, 85, 75);

    lv_obj_t *gauge_mode_switch = lv_switch_create(scr_settings_func);
    lv_obj_align(gauge_mode_switch, LV_ALIGN_TOP_MID, 85, 97);
    lv_obj_add_event_cb(gauge_mode_switch, gauge_mode_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    if (g_multi_gauge_mode == MULTI_GAUGE_RPM) {
        lv_obj_add_state(gauge_mode_switch, LV_STATE_CHECKED);
        lv_label_set_text(gauge_mode_label, "Anzeige: Drehzahl");
        lv_meter_set_scale_range(multi_meter, multi_needle_scale, 0, 8000, 270, 45);
        lv_meter_set_scale_range(multi_meter, multi_ring_scale, 0, 8000, 270, 135);
    }

    // Eigener Schalter fuer das alternative "BMW Style"-Layout der Multi-
    // Kachel (siehe apply_bmw_style()) - unabhaengig vom Wasser-/Drehzahl-
    // Umschalter darueber, in die Luecke zwischen diesem und dem Warnsummer
    // platziert.
    lv_obj_t *bmw_style_label = lv_label_create(scr_settings_func);
    lv_label_set_text(bmw_style_label, "BMW Style");
    lv_obj_align(bmw_style_label, LV_ALIGN_TOP_MID, 0, 145);

    lv_obj_t *bmw_style_switch = lv_switch_create(scr_settings_func);
    lv_obj_align(bmw_style_switch, LV_ALIGN_TOP_MID, 0, 167);
    lv_obj_add_event_cb(bmw_style_switch, bmw_style_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);
    g_bmw_style_sw[0] = bmw_style_switch;
    if (g_bmw_style_enabled) {
        lv_obj_add_state(bmw_style_switch, LV_STATE_CHECKED);
    }
    apply_bmw_style(g_bmw_style_enabled); // aus NVS geladener Zustand, direkt beim Aufbau anwenden

    lv_obj_t *buzzer_label = lv_label_create(scr_settings_func);
    lv_label_set_text(buzzer_label, "Warnsummer ab 120\xC2\xB0""C");
    lv_obj_align(buzzer_label, LV_ALIGN_TOP_MID, 0, 190);

    lv_obj_t *buzzer_switch = lv_switch_create(scr_settings_func);
    lv_obj_align(buzzer_switch, LV_ALIGN_TOP_MID, 0, 212);
    if (g_buzzer_enabled) {
        lv_obj_add_state(buzzer_switch, LV_STATE_CHECKED); // aus NVS geladen, Standard: an
    }
    lv_obj_add_event_cb(buzzer_switch, buzzer_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Einmaliger SNTP-Zeitabgleich ueber den in /sdcard/wifi-einstellungen.txt
    // hinterlegten Hotspot - bewusst NICHT in NVS gespeichert (siehe
    // hotspot_switch_cb), startet bei jedem Boot wieder aus.
    lv_obj_t *hotspot_label = lv_label_create(scr_settings_func);
    lv_label_set_text(hotspot_label, "Hotspot verbinden");
    lv_obj_align(hotspot_label, LV_ALIGN_TOP_MID, 0, 280);

    hotspot_switch = lv_switch_create(scr_settings_func);
    lv_obj_align(hotspot_switch, LV_ALIGN_TOP_MID, 0, 302);
    lv_obj_add_event_cb(hotspot_switch, hotspot_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    hotspot_status_label = lv_label_create(scr_settings_func);
    lv_label_set_text(hotspot_status_label, "Inaktiv");
    lv_obj_set_style_text_color(hotspot_status_label, lv_color_make(150, 150, 150), 0);
    lv_obj_align(hotspot_status_label, LV_ALIGN_TOP_MID, 0, 330);

    // SD-Karte formatieren: Wenn die Karte unformatiert/beschaedigt ist
    // (z.B. "SD-Fehler!" beim Datenlogging oder "zu wenig Speicher" beim
    // Hotspot-Task, weil die Karte nie erfolgreich mountet), hilft weder
    // Datenlogging noch Hotspot - hier direkt neu formatieren.
    lv_obj_t *format_btn = lv_btn_create(scr_settings_func);
    lv_obj_set_size(format_btn, 170, 34);
    lv_obj_align(format_btn, LV_ALIGN_TOP_MID, 0, 358);
    set_dark_blue_btn(format_btn);
    lv_obj_add_event_cb(format_btn, sd_format_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *format_btn_label = lv_label_create(format_btn);
    lv_label_set_text(format_btn_label, "SD formatieren");
    lv_obj_center(format_btn_label);

    sd_format_status_label = lv_label_create(scr_settings_func);
    lv_label_set_text(sd_format_status_label, "");
    lv_obj_set_style_text_color(sd_format_status_label, lv_color_make(150, 150, 150), 0);
    lv_obj_align(sd_format_status_label, LV_ALIGN_TOP_MID, 0, 396);

    lv_obj_t *hint = lv_label_create(scr_settings_func);
    lv_label_set_text(hint, "<- Wisch: Farben");
    lv_obj_set_style_text_color(hint, lv_color_make(150, 150, 150), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);

    lv_obj_add_event_cb(scr_settings_func, swipe_gesture_cb, LV_EVENT_GESTURE, NULL);
}

// --- Fehlercode-Screen: erreichbar per Wisch nach links auf der Multi-
// Ansicht, per Wisch nach rechts geht es zurueck. Obere Haelfte zeigt die
// zuletzt ausgelesenen Fehlercodes, darunter Auslesen/Loeschen-Buttons und
// ein Service-Reset-Button.
static void swipe_gesture_cb(lv_event_t *e)
{
    lv_obj_t *scr = lv_event_get_current_target(e);
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    if (scr == scr_multi && dir == LV_DIR_LEFT) {
        lv_scr_load_anim(scr_dtc, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    } else if (scr == scr_dtc && dir == LV_DIR_RIGHT) {
        lv_scr_load_anim(scr_multi, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    } else if (scr == scr_multi && dir == LV_DIR_RIGHT) {
        lv_scr_load_anim(scr_service, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    } else if (scr == scr_service && dir == LV_DIR_LEFT) {
        lv_scr_load_anim(scr_multi, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    } else if (scr == scr_service && dir == LV_DIR_RIGHT) {
        // Lambda/MAP-Abfrage erst jetzt starten - kostet sonst unnoetig
        // Poll-Zeit/Buszeit, solange niemand auf den Screen schaut.
        if (g_data_source == DATA_SRC_BLE_OBD) BLE_OBD_set_sensors_active(true);
        else CAN_OBD2_set_sensors_active(true);
        lv_scr_load_anim(scr_sensors, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    } else if (scr == scr_sensors && dir == LV_DIR_LEFT) {
        // Beide Quellen stoppen (nicht nur die gerade aktive) - falls
        // zwischendurch umgeschaltet wurde, bleibt sonst eine Quelle aktiv.
        BLE_OBD_set_sensors_active(false);
        CAN_OBD2_set_sensors_active(false);
        lv_scr_load_anim(scr_service, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    } else if (scr == scr_settings && dir == LV_DIR_RIGHT) {
        lv_scr_load_anim(scr_settings_func, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    } else if (scr == scr_settings_func && dir == LV_DIR_LEFT) {
        lv_scr_load_anim(scr_settings, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    } else if (scr == scr_multi && dir == LV_DIR_TOP) {
        // Kein Slide (LV_SCR_LOAD_ANIM_NONE statt MOVE_TOP): Die Multi-Kachel
        // zeigt als einzige Ansicht einen detailreichen Ring mit feinen
        // Teilstrichen, auf dem das seit v1.0.9-1.0.14 bekannte, ungeloeste
        // RGB-Panel-Tearing (Doppel-Framebuffer ohne Vsync-Sync) am
        // deutlichsten auffaellt. Die 200ms-Animation erzeugt viele schnell
        // aufeinanderfolgende Teil-Redraws und war der wahrscheinlichste
        // zusaetzliche Ausloeser dafuer, dass es hier erstmals auffiel.
        // Ohne Animation (ein Frame direkt ersetzt das andere) sollte das
        // Risiko sinken - behebt das zugrunde liegende Tearing aber nicht,
        // das steckt tiefer im RGB-Treiber (siehe README/CLAUDE.md).
        lv_scr_load_anim(scr_led, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
    } else if (scr == scr_led && dir == LV_DIR_BOTTOM) {
        lv_scr_load_anim(scr_multi, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
    }
}

// Jeder Tastendruck wird sofort und unabhaengig vom BLE-/MCP-Zustand ins
// SD-Log geschrieben - bisherige Logs zeigten in mehreren langen Testfahrten
// keinen einzigen Hinweis auf einen DTC-/Service-Befehl, obwohl die Buttons
// laut Nutzer gedrueckt wurden. Damit laesst sich erstmals unterscheiden, ob
// der Touch ueberhaupt ankommt (diese Zeile fehlt) oder ob er ankommt, aber
// die BLE-/MCP-Seite danach stumm bleibt (diese Zeile steht da, aber keine
// TX-/Service-Zeile aus ble_obd.c/can_obd2.c folgt).
static void dtc_read_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    SD_Log("UI: Button 'Auslesen' gedrueckt (Quelle=%s)", g_data_source == DATA_SRC_BLE_OBD ? "BLE" : "MCP");
    lv_label_set_text(dtc_status_label, "Frage Fehlercodes an...");
    if (g_data_source == DATA_SRC_BLE_OBD) BLE_OBD_read_dtc();
    else CAN_OBD2_read_dtc();
}

static void dtc_clear_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    SD_Log("UI: Button 'Loeschen' gedrueckt (Quelle=%s)", g_data_source == DATA_SRC_BLE_OBD ? "BLE" : "MCP");
    lv_label_set_text(dtc_status_label, "Loesche Fehlercodes...");
    if (g_data_source == DATA_SRC_BLE_OBD) BLE_OBD_clear_dtc();
    else CAN_OBD2_clear_dtc();
}

static void dtc_service_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    SD_Log("UI: Button 'Service Reset' gedrueckt (Quelle=%s)", g_data_source == DATA_SRC_BLE_OBD ? "BLE" : "MCP");
    lv_label_set_text(dtc_status_label, "Sende Service-Reset...");
    if (g_data_source == DATA_SRC_BLE_OBD) BLE_OBD_reset_service_oil();
    else CAN_OBD2_reset_service_oil();
}

static void create_dtc_screen(void)
{
    scr_dtc = lv_obj_create(NULL);
    set_dark_bg(scr_dtc);
    lv_obj_clear_flag(scr_dtc, LV_OBJ_FLAG_SCROLLABLE);

    // Titel weiter von der oberen Rundung des Panels weggerueckt (y 15 -> 35):
    // bei y=15 faellt ein Teil des Textes in die abgeschnittene Kante des
    // runden Displays und verschwindet dort.
    lv_obj_t *title = lv_label_create(scr_dtc);
    lv_label_set_text(title, "FEHLERCODES");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 35);

    // Liste der ausgelesenen Fehlercodes: deutlich mehr Platz (Hoehe 190 -> 225)
    lv_obj_t *list_box = lv_obj_create(scr_dtc);
    set_dark_bg(list_box);
    lv_obj_set_size(list_box, 420, 225);
    lv_obj_align(list_box, LV_ALIGN_TOP_MID, 0, 65);
    lv_obj_set_scroll_dir(list_box, LV_DIR_VER);

    dtc_list_label = lv_label_create(list_box);
    lv_label_set_long_mode(dtc_list_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(dtc_list_label, &lv_font_montserrat_28, 0);
    lv_obj_set_width(dtc_list_label, 390);
    lv_obj_align(dtc_list_label, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_label_set_text(dtc_list_label, "Noch nicht ausgelesen");

    dtc_status_label = lv_label_create(scr_dtc);
    lv_obj_set_style_text_font(dtc_status_label, &lv_font_montserrat_28, 0);
    lv_label_set_text(dtc_status_label, "");
    lv_obj_align(dtc_status_label, LV_ALIGN_TOP_MID, 0, 300);

    // Auslesen / Loeschen nebeneinander, schmaler als vorher (200 -> 170),
    // dadurch rutschen alle drei Buttons weiter nach unten (325 statt 270)
    // und die Schrift (28pt statt Standard) hat trotzdem genug Platz.
    lv_obj_t *btn_read = lv_btn_create(scr_dtc);
    set_dark_blue_btn(btn_read);
    lv_obj_set_size(btn_read, 170, 64);
    lv_obj_align(btn_read, LV_ALIGN_TOP_MID, -90, 325);
    lv_obj_add_event_cb(btn_read, dtc_read_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *read_lbl = lv_label_create(btn_read);
    lv_obj_set_style_text_font(read_lbl, &lv_font_montserrat_28, 0);
    lv_label_set_text(read_lbl, "Auslesen");
    lv_obj_center(read_lbl);

    lv_obj_t *btn_clear = lv_btn_create(scr_dtc);
    set_dark_blue_btn(btn_clear);
    lv_obj_set_size(btn_clear, 170, 64);
    lv_obj_align(btn_clear, LV_ALIGN_TOP_MID, 90, 325);
    lv_obj_add_event_cb(btn_clear, dtc_clear_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *clear_lbl = lv_label_create(btn_clear);
    lv_obj_set_style_text_font(clear_lbl, &lv_font_montserrat_28, 0);
    lv_label_set_text(clear_lbl, "Loeschen");
    lv_obj_center(clear_lbl);

    // Service-Reset darunter
    lv_obj_t *btn_service = lv_btn_create(scr_dtc);
    set_dark_blue_btn(btn_service);
    lv_obj_set_size(btn_service, 260, 64);
    lv_obj_align(btn_service, LV_ALIGN_TOP_MID, 0, 400);
    lv_obj_add_event_cb(btn_service, dtc_service_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *service_lbl = lv_label_create(btn_service);
    lv_obj_set_style_text_font(service_lbl, &lv_font_montserrat_28, 0);
    lv_label_set_text(service_lbl, "Service Reset");
    lv_obj_center(service_lbl);

    lv_obj_add_event_cb(scr_dtc, swipe_gesture_cb, LV_EVENT_GESTURE, NULL);
}

// --- Service-Screen: erreichbar per Wisch nach rechts auf der Multi-Ansicht,
// Wisch nach links geht zurueck. Ein Button je Eintrag in SERVICE_FUNCS[].
static void service_btn_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    SD_Log("UI: Service-Button '%s' gedrueckt (Quelle=%s)", SERVICE_FUNCS[idx].label,
           g_data_source == DATA_SRC_BLE_OBD ? "BLE" : "MCP");
    bool ok = (g_data_source == DATA_SRC_BLE_OBD) ? BLE_OBD_service_func(idx)
                                                  : CAN_OBD2_service_func(idx);
    lv_label_set_text_fmt(service_status_label, ok ? "Gesendet: %s" : "Nicht hinterlegt: %s",
                          SERVICE_FUNCS[idx].label);
}

static void create_service_screen(void)
{
    scr_service = lv_obj_create(NULL);
    set_dark_bg(scr_service);
    lv_obj_clear_flag(scr_service, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr_service);
    lv_label_set_text(title, "SERVICE");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 25);

    lv_obj_t *ver = lv_label_create(scr_service);
    lv_label_set_text_fmt(ver, "v%s", esp_app_get_description()->version);
    lv_obj_set_style_text_color(ver, lv_color_make(150, 150, 150), 0);
    lv_obj_align(ver, LV_ALIGN_TOP_MID, 0, 48);

    for (int i = 0; i < SERVICE_FUNC_COUNT; i++) {
        lv_obj_t *btn = lv_btn_create(scr_service);
        set_dark_blue_btn(btn);
        lv_obj_set_size(btn, 420, 70);
        lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 80 + i * 80);
        lv_obj_add_event_cb(btn, service_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_48, 0);
        lv_label_set_text(lbl, SERVICE_FUNCS[i].label);
        lv_obj_center(lbl);
    }

    service_status_label = lv_label_create(scr_service);
    lv_label_set_text(service_status_label, "");
    lv_obj_align(service_status_label, LV_ALIGN_BOTTOM_MID, 0, -30);

    lv_obj_add_event_cb(scr_service, swipe_gesture_cb, LV_EVENT_GESTURE, NULL);
}

// --- Sensoren-Screen: reine Digitalwert-Anzeige fuer Werte, die auf keiner
// der 6 Hauptkacheln Platz haben. Erreichbar per Wisch nach rechts auf dem
// Service-Screen (also "hinter" Multi-Ansicht -> Service), Wisch nach links
// geht zurueck. Lambda/Ansaugdruck/Ladedruck/Oel-Temp/MIL kommen live per
// OBD2 von der eingestellten Datenquelle (siehe BMW_UI_Update()); Ladedruck
// = MAP - Baro (siehe can_obd2.h/ble_obd.h) - der N43B20A hat laut CLAUDE.md
// keinen Turbo, daher sind hier nur Werte um/leicht unter 0 kPa zu erwarten,
// kein echter Ladedruck. Oel-Temp nutzt die BMW-spezifische Service-0x22-PID
// 0x4402 (kein Standard-Mode-01-PID vorhanden), siehe can_obd2.h/ble_obd.h -
// Quelle: github.com/TomWis97/bmw-obd2-display. Die VANOS-Nockenwellenposition
// zeigt weiterhin immer "n/v": dafuer existiert ueberhaupt kein Standard-
// OBD2-PID - das waere nur per BMW-spezifischem UDS-Identifier auslesbar,
// was hier bewusst nicht blind nachgebaut wurde (vgl. die ungeklaerten
// NOx-Regeneration-/Bremsenentlueften-Service-Funktionen).
static lv_obj_t *create_sensor_cell(lv_obj_t *parent, const char *caption, int x, int y)
{
    lv_obj_t *cap = lv_label_create(parent);
    lv_label_set_text(cap, caption);
    lv_obj_set_style_text_color(cap, lv_color_make(150, 150, 150), 0);
    lv_obj_align(cap, LV_ALIGN_TOP_MID, x, y);

    lv_obj_t *val = lv_label_create(parent);
    lv_obj_set_style_text_font(val, &lv_font_montserrat_28, 0);
    lv_label_set_text(val, "---");
    lv_obj_align(val, LV_ALIGN_TOP_MID, x, y + 22);
    return val;
}

static void create_sensors_screen(void)
{
    scr_sensors = lv_obj_create(NULL);
    set_dark_bg(scr_sensors);
    lv_obj_clear_flag(scr_sensors, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr_sensors);
    lv_label_set_text(title, "SENSOREN");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 25);

    sens_lambda1_val      = create_sensor_cell(scr_sensors, "Lambda B1",     -110, 70);
    sens_lambda2_val      = create_sensor_cell(scr_sensors, "Lambda B2",      110, 70);
    sens_lambda1_volt_val = create_sensor_cell(scr_sensors, "Lambda B1 Sp.", -110, 150);
    sens_lambda2_volt_val = create_sensor_cell(scr_sensors, "Lambda B2 Sp.",  110, 150);
    sens_map_val          = create_sensor_cell(scr_sensors, "Ansaugdruck",  -110, 230);
    sens_boost_val        = create_sensor_cell(scr_sensors, "Ladedruck",     110, 230);
    sens_cam_in_val       = create_sensor_cell(scr_sensors, "NW Einlass",   -110, 310);
    sens_cam_ex_val       = create_sensor_cell(scr_sensors, "NW Auslass",    110, 310);
    sens_oil_val          = create_sensor_cell(scr_sensors, "Oel-Temp",     -110, 390);
    sens_mil_val          = create_sensor_cell(scr_sensors, "Motorlampe",    110, 390);

    // Werden nie per OBD2 beantwortet (siehe Kommentar oben) - einmalig fest
    // auf "n/v" setzen statt bei jedem BMW_UI_Update()-Tick neu zu schreiben.
    lv_label_set_text(sens_cam_in_val, "n/v");
    lv_label_set_text(sens_cam_ex_val, "n/v");

    lv_obj_t *hint = lv_label_create(scr_sensors);
    lv_label_set_text(hint, "<- Wisch: Service");
    lv_obj_set_style_text_color(hint, lv_color_make(150, 150, 150), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);

    lv_obj_add_event_cb(scr_sensors, swipe_gesture_cb, LV_EVENT_GESTURE, NULL);
}

// --- LED-Control-Screen: Ansteuerung von bis zu zwei WS2812B/SK6812-
// kompatiblen Streifen - Streifen 1 an GPIO0 (immer frei, siehe led_ctrl.c/
// mcp2515.h), Streifen 2 an GPIO19 (teilt sich den Pin mit MCP2515-MISO,
// siehe docs/led_wiring.png - fuer Streifen 2 muss die Datenquelle auf
// BLE-OBD stehen). Ein Tab-Umschalter oben waehlt, welcher der zwei
// unabhaengig laufenden Streifen gerade angezeigt/editiert wird; beide
// Backends (LED_Ctrl_*/LED_Ctrl2_*) laufen davon unberuehrt parallel weiter.
// Erreichbar per Wisch nach oben auf der Multi-Ansicht, Wisch nach unten
// geht zurueck. Zwei Muster: Solid (eine Farbe fuer alle LEDs) und Chase
// (Hauptfarbe als Hintergrund, Zweitfarbe laeuft mit kurzem Schweif von
// LED 0 bis zum Ende im Loop, Geschwindigkeit per Schieberegler).
static void update_led_pattern_btn_styles(void)
{
    led_pattern_t pat = g_led[g_led_active_strip].pattern;
    lv_obj_set_style_bg_color(led_pattern_solid_btn,
        pat == LED_PATTERN_SOLID ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_color(led_pattern_chase_btn,
        pat == LED_PATTERN_CHASE ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_GREY), 0);
    // Zweitfarbe/Geschwindigkeit sind nur im Chase-Muster relevant.
    if (pat == LED_PATTERN_CHASE) {
        lv_obj_clear_flag(led_speed_row, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(led_speed_row, LV_OBJ_FLAG_HIDDEN);
    }
}

// Streifen 2 teilt sich GPIO19 mit MCP2515-MISO (siehe led_ctrl.h) - wird
// Streifen 2 betrachtet, waehrend die Datenquelle auf MCP2515 steht, zeigt
// ein Warnhinweis den Konflikt an. Wird sowohl beim Tab-Wechsel als auch
// periodisch aus BMW_UI_Update() aufgerufen (Datenquelle kann sich auf
// einem anderen Screen aendern, waehrend dieser Screen schon sichtbar ist).
static void update_led_conflict_warning(void)
{
    if (!led_conflict_label) return;
    bool conflict = (g_led_active_strip == 1 && g_data_source == DATA_SRC_MCP2515);
    if (conflict) {
        lv_label_set_text(led_conflict_label, "Achtung: MCP2515 aktiv - blockiert GPIO19!");
        lv_obj_clear_flag(led_conflict_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(led_conflict_label, LV_OBJ_FLAG_HIDDEN);
    }
}

// Aktualisiert alle gemeinsam genutzten Widgets (Schalter/Muster-Buttons/
// Slider) auf den Stand des gerade per Tab ausgewaehlten Streifens - reiner
// UI-Refresh, die Backends beider Streifen laufen unabhaengig im Hintergrund
// weiter und werden hier nicht angefasst.
static void refresh_led_screen_controls(void)
{
    int idx = g_led_active_strip;
    lv_label_set_text_fmt(led_title_label, "LED CONTROL – Streifen %d", idx + 1);
    lv_obj_set_style_bg_color(led_strip1_tab_btn,
        idx == 0 ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_color(led_strip2_tab_btn,
        idx == 1 ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_GREY), 0);

    if (g_led[idx].enabled) {
        lv_obj_add_state(led_enable_switch, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(led_enable_switch, LV_STATE_CHECKED);
    }

    lv_slider_set_value(led_count_slider, g_led[idx].count, LV_ANIM_OFF);
    lv_label_set_text_fmt(led_count_label, "Anzahl: %d", g_led[idx].count);

    lv_slider_set_value(led_brightness_slider, g_led[idx].brightness, LV_ANIM_OFF);
    lv_label_set_text_fmt(led_brightness_label, "Helligkeit: %d%%", g_led[idx].brightness);

    lv_slider_set_value(led_speed_slider, g_led[idx].speed, LV_ANIM_OFF);
    lv_label_set_text_fmt(led_speed_label, "Tempo: %d%%", g_led[idx].speed);

    lv_obj_set_style_bg_color(led_main_color_swatch, COLOR_PALETTE[g_led[idx].color_main_idx].color, 0);
    lv_obj_set_style_bg_color(led_second_color_swatch, COLOR_PALETTE[g_led[idx].color_second_idx].color, 0);

    update_led_pattern_btn_styles();
    update_led_conflict_warning();
}

static void led_strip_tab_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    g_led_active_strip = idx;
    refresh_led_screen_controls();
}

static void led_enable_switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    int idx = g_led_active_strip;
    g_led[idx].enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    led_apply_enabled(idx, g_led[idx].enabled);
    bmw_settings_save();
}

static void led_pattern_solid_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    int idx = g_led_active_strip;
    g_led[idx].pattern = LED_PATTERN_SOLID;
    led_apply_pattern(idx, LED_PATTERN_SOLID);
    update_led_pattern_btn_styles();
    bmw_settings_save();
}

static void led_pattern_chase_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    int idx = g_led_active_strip;
    g_led[idx].pattern = LED_PATTERN_CHASE;
    led_apply_pattern(idx, LED_PATTERN_CHASE);
    update_led_pattern_btn_styles();
    bmw_settings_save();
}

static void led_count_slider_changed_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int idx = g_led_active_strip;
    g_led[idx].count = lv_slider_get_value(slider);
    led_apply_count(idx, g_led[idx].count);
    lv_label_set_text_fmt(led_count_label, "Anzahl: %d", g_led[idx].count);
}

// Erst beim Loslassen in NVS schreiben, nicht bei jedem Zwischenschritt
// waehrend des Ziehens - sonst unnoetig viele Flash-Schreibzyklen.
static void led_count_slider_released_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    bmw_settings_save();
}

static void led_brightness_slider_changed_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int idx = g_led_active_strip;
    g_led[idx].brightness = (uint8_t)lv_slider_get_value(slider);
    led_apply_brightness(idx, g_led[idx].brightness);
    lv_label_set_text_fmt(led_brightness_label, "Helligkeit: %d%%", g_led[idx].brightness);
}

static void led_brightness_slider_released_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    bmw_settings_save();
}

static void led_speed_slider_changed_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int idx = g_led_active_strip;
    g_led[idx].speed = (uint8_t)lv_slider_get_value(slider);
    led_apply_speed(idx, g_led[idx].speed);
    lv_label_set_text_fmt(led_speed_label, "Tempo: %d%%", g_led[idx].speed);
}

static void led_speed_slider_released_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    bmw_settings_save();
}

// Ersetzt die fruehere 2-spaltige Farbraster-Spalte (musste in nur 115px
// Hoehe passen, Gruen war dadurch kaum erreichbar) durch einen einzelnen
// Farbkasten je Hauptfarbe/Zweitfarbe, der nur die aktuell gewaehlte Farbe
// zeigt (siehe refresh_led_screen_controls()). Tippen auf den Kasten oeffnet
// ein vollflaechiges Overlay mit allen Paletten-Farben in grosszuegigen
// Kacheln - deutlich mehr Platz als die alte feste Spalte erlaubte.
static bool led_popup_is_second = false; // welcher der beiden Kaesten das Popup geoeffnet hat

static void led_color_popup_close(void)
{
    if (led_color_popup_bg) {
        lv_obj_del(led_color_popup_bg);
        led_color_popup_bg = NULL;
    }
}

// Tippen auf eine Farbkachel: uebernehmen, Kasten-Vorschau aktualisieren,
// Popup schliessen - wie vom Nutzer gewuenscht ("Tippt man auf eine Farbe,
// wird diese uebernommen und das Fenster schliesst sich").
static void led_popup_color_btn_cb(lv_event_t *e)
{
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    const RgbColor *c = &COLOR_PALETTE_RGB[i];
    int idx = g_led_active_strip;
    if (led_popup_is_second) {
        g_led[idx].color_second_idx = (uint8_t)i;
        led_apply_second_color(idx, c->r, c->g, c->b);
        lv_obj_set_style_bg_color(led_second_color_swatch, COLOR_PALETTE[i].color, 0);
    } else {
        g_led[idx].color_main_idx = (uint8_t)i;
        led_apply_main_color(idx, c->r, c->g, c->b);
        lv_obj_set_style_bg_color(led_main_color_swatch, COLOR_PALETTE[i].color, 0);
    }
    bmw_settings_save();
    led_color_popup_close();
}

// Tippen auf den abgedunkelten Hintergrund ausserhalb der Kachel-Flaeche
// schliesst das Popup ohne Auswahl (Flaeche selbst ist nicht klickbar -
// Events von Kindern bubbeln aber nicht bis hierher durch, da die Kachel-
// Flaeche die Klicks selbst konsumiert).
static void led_color_popup_bg_cb(lv_event_t *e)
{
    if (lv_event_get_target(e) == led_color_popup_bg) {
        led_color_popup_close();
    }
}

static void led_open_color_popup(lv_obj_t *parent_screen, bool is_second)
{
    led_color_popup_close(); // Sicherheitsnetz, falls schon eines offen war
    led_popup_is_second = is_second;

    led_color_popup_bg = lv_obj_create(parent_screen);
    lv_obj_remove_style_all(led_color_popup_bg);
    lv_obj_set_size(led_color_popup_bg, 480, 480);
    lv_obj_set_pos(led_color_popup_bg, 0, 0);
    lv_obj_set_style_bg_color(led_color_popup_bg, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(led_color_popup_bg, LV_OPA_70, 0);
    lv_obj_clear_flag(led_color_popup_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(led_color_popup_bg, led_color_popup_bg_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *panel = lv_obj_create(led_color_popup_bg);
    set_dark_bg(panel);
    lv_obj_set_size(panel, 360, 340);
    lv_obj_center(panel);
    lv_obj_set_style_pad_all(panel, 10, 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(panel, LV_DIR_VER);
    lv_obj_set_style_pad_row(panel, 10, 0);
    lv_obj_set_style_pad_column(panel, 10, 0);

    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text(title, is_second ? "Zweitfarbe waehlen" : "Hauptfarbe waehlen");
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);

    for (int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        lv_obj_t *btn = lv_btn_create(panel);
        lv_obj_set_size(btn, 95, 60);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_bg_color(btn, COLOR_PALETTE[i].color, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_add_event_cb(btn, led_popup_color_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

static void led_main_swatch_cb(lv_event_t *e)
{
    led_open_color_popup(lv_obj_get_screen(lv_event_get_target(e)), false);
}

static void led_second_swatch_cb(lv_event_t *e)
{
    led_open_color_popup(lv_obj_get_screen(lv_event_get_target(e)), true);
}

static void create_led_screen(void)
{
    scr_led = lv_obj_create(NULL);
    set_dark_bg(scr_led);
    lv_obj_clear_flag(scr_led, LV_OBJ_FLAG_SCROLLABLE);

    led_title_label = lv_label_create(scr_led);
    lv_label_set_text(led_title_label, "LED CONTROL – Streifen 1");
    lv_obj_align(led_title_label, LV_ALIGN_TOP_MID, 0, 15);

    // Tab-Umschalter: waehlt, welcher der zwei unabhaengigen Streifen in den
    // Controls darunter angezeigt/editiert wird (siehe refresh_led_screen_controls()).
    led_strip1_tab_btn = lv_btn_create(scr_led);
    lv_obj_set_size(led_strip1_tab_btn, 140, 32);
    lv_obj_align(led_strip1_tab_btn, LV_ALIGN_TOP_MID, -75, 40);
    lv_obj_add_event_cb(led_strip1_tab_btn, led_strip_tab_cb, LV_EVENT_CLICKED, (void *)(intptr_t)0);
    lv_obj_t *tab1_lbl = lv_label_create(led_strip1_tab_btn);
    lv_label_set_text(tab1_lbl, "Streifen 1");
    lv_obj_center(tab1_lbl);

    led_strip2_tab_btn = lv_btn_create(scr_led);
    lv_obj_set_size(led_strip2_tab_btn, 140, 32);
    lv_obj_align(led_strip2_tab_btn, LV_ALIGN_TOP_MID, 75, 40);
    lv_obj_add_event_cb(led_strip2_tab_btn, led_strip_tab_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);
    lv_obj_t *tab2_lbl = lv_label_create(led_strip2_tab_btn);
    lv_label_set_text(tab2_lbl, "Streifen 2");
    lv_obj_center(tab2_lbl);

    lv_obj_t *enable_label = lv_label_create(scr_led);
    lv_label_set_text(enable_label, "Streifen aktiv");
    lv_obj_align(enable_label, LV_ALIGN_TOP_MID, 0, 72);

    led_enable_switch = lv_switch_create(scr_led);
    lv_obj_align(led_enable_switch, LV_ALIGN_TOP_MID, 0, 94);
    lv_obj_add_event_cb(led_enable_switch, led_enable_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    led_pattern_solid_btn = lv_btn_create(scr_led);
    lv_obj_set_size(led_pattern_solid_btn, 130, 40);
    lv_obj_align(led_pattern_solid_btn, LV_ALIGN_TOP_MID, -75, 137);
    lv_obj_add_event_cb(led_pattern_solid_btn, led_pattern_solid_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *solid_lbl = lv_label_create(led_pattern_solid_btn);
    lv_label_set_text(solid_lbl, "Solid");
    lv_obj_center(solid_lbl);

    led_pattern_chase_btn = lv_btn_create(scr_led);
    lv_obj_set_size(led_pattern_chase_btn, 130, 40);
    lv_obj_align(led_pattern_chase_btn, LV_ALIGN_TOP_MID, 75, 137);
    lv_obj_add_event_cb(led_pattern_chase_btn, led_pattern_chase_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *chase_lbl = lv_label_create(led_pattern_chase_btn);
    lv_label_set_text(chase_lbl, "Chase");
    lv_obj_center(chase_lbl);

    led_count_label = lv_label_create(scr_led);
    lv_label_set_text(led_count_label, "Anzahl: 30");
    lv_obj_align(led_count_label, LV_ALIGN_TOP_MID, 0, 182);

    led_count_slider = lv_slider_create(scr_led);
    lv_obj_set_size(led_count_slider, 300, 14);
    lv_obj_align(led_count_slider, LV_ALIGN_TOP_MID, 0, 207);
    lv_slider_set_range(led_count_slider, 1, LED_CTRL_MAX_LEDS);
    lv_obj_add_event_cb(led_count_slider, led_count_slider_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(led_count_slider, led_count_slider_released_cb, LV_EVENT_RELEASED, NULL);

    // Zwischen Anzahl- und Tempo-Regler platziert (vom Nutzer so gewuenscht).
    led_brightness_label = lv_label_create(scr_led);
    lv_label_set_text(led_brightness_label, "Helligkeit: 100%");
    lv_obj_align(led_brightness_label, LV_ALIGN_TOP_MID, 0, 237);

    led_brightness_slider = lv_slider_create(scr_led);
    lv_obj_set_size(led_brightness_slider, 300, 14);
    lv_obj_align(led_brightness_slider, LV_ALIGN_TOP_MID, 0, 261);
    lv_slider_set_range(led_brightness_slider, 0, 100);
    lv_obj_add_event_cb(led_brightness_slider, led_brightness_slider_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(led_brightness_slider, led_brightness_slider_released_cb, LV_EVENT_RELEASED, NULL);

    // Nur im Chase-Muster relevant (Geschwindigkeit des Komet-Schweifs) -
    // Sichtbarkeit wird in update_led_pattern_btn_styles() gesteuert.
    led_speed_row = lv_obj_create(scr_led);
    lv_obj_set_size(led_speed_row, 320, 55);
    lv_obj_align(led_speed_row, LV_ALIGN_TOP_MID, 0, 291);
    lv_obj_set_style_bg_opa(led_speed_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(led_speed_row, 0, 0);
    lv_obj_clear_flag(led_speed_row, LV_OBJ_FLAG_SCROLLABLE);

    led_speed_label = lv_label_create(led_speed_row);
    lv_label_set_text(led_speed_label, "Tempo: 50%");
    lv_obj_align(led_speed_label, LV_ALIGN_TOP_MID, 0, 0);

    led_speed_slider = lv_slider_create(led_speed_row);
    lv_obj_set_size(led_speed_slider, 300, 14);
    lv_obj_align(led_speed_slider, LV_ALIGN_TOP_MID, 0, 26);
    lv_slider_set_range(led_speed_slider, 0, 100);
    lv_obj_add_event_cb(led_speed_slider, led_speed_slider_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(led_speed_slider, led_speed_slider_released_cb, LV_EVENT_RELEASED, NULL);

    // Kompakte Farbvorschau-Kaesten statt der frueheren festen Farbraster-
    // Spalten - Tippen oeffnet das vollflaechige Auswahl-Popup (siehe
    // led_open_color_popup()).
    lv_obj_t *main_hdr = lv_label_create(scr_led);
    lv_label_set_text(main_hdr, "Hauptfarbe");
    lv_obj_align(main_hdr, LV_ALIGN_TOP_MID, -90, 356);

    led_main_color_swatch = lv_btn_create(scr_led);
    lv_obj_set_size(led_main_color_swatch, 130, 50);
    lv_obj_align(led_main_color_swatch, LV_ALIGN_TOP_MID, -90, 380);
    lv_obj_set_style_radius(led_main_color_swatch, 10, 0);
    lv_obj_set_style_border_width(led_main_color_swatch, 2, 0);
    lv_obj_set_style_border_color(led_main_color_swatch, lv_color_white(), 0);
    lv_obj_add_event_cb(led_main_color_swatch, led_main_swatch_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *second_hdr = lv_label_create(scr_led);
    lv_label_set_text(second_hdr, "Zweitfarbe");
    lv_obj_align(second_hdr, LV_ALIGN_TOP_MID, 90, 356);

    led_second_color_swatch = lv_btn_create(scr_led);
    lv_obj_set_size(led_second_color_swatch, 130, 50);
    lv_obj_align(led_second_color_swatch, LV_ALIGN_TOP_MID, 90, 380);
    lv_obj_set_style_radius(led_second_color_swatch, 10, 0);
    lv_obj_set_style_border_width(led_second_color_swatch, 2, 0);
    lv_obj_set_style_border_color(led_second_color_swatch, lv_color_white(), 0);
    lv_obj_add_event_cb(led_second_color_swatch, led_second_swatch_cb, LV_EVENT_CLICKED, NULL);

    led_conflict_label = lv_label_create(scr_led);
    lv_label_set_text(led_conflict_label, "");
    lv_obj_set_style_text_color(led_conflict_label, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(led_conflict_label, LV_ALIGN_BOTTOM_MID, 0, -38);
    lv_obj_add_flag(led_conflict_label, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *hint = lv_label_create(scr_led);
    lv_label_set_text(hint, "Wisch runter: Zurueck");
    lv_obj_set_style_text_color(hint, lv_color_make(150, 150, 150), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -15);

    // Aktuellen (ggf. aus NVS geladenen) Zustand beider Streifen auf ihr
    // jeweiliges Backend anwenden, bevor der Screen zum ersten Mal sichtbar
    // wird - analog zum Muster bei Datenlogging/Buzzer/Gauge-Modus in
    // create_settings_func_screen(). LED_Ctrl[2]_SetEnabled() gibt bei
    // Bedarf nur ein Semaphor frei, das den jeweiligen led_task aufweckt
    // (siehe led_ctrl.c) - kein synchroner I/O mitten im UI-Aufbau wie beim
    // fruehen fopen()-Bootloop des Datenloggings.
    for (int i = 0; i < 2; i++) {
        led_apply_count(i, g_led[i].count);
        led_apply_speed(i, g_led[i].speed);
        led_apply_brightness(i, g_led[i].brightness);
        led_apply_pattern(i, g_led[i].pattern);
        led_apply_main_color(i, COLOR_PALETTE_RGB[g_led[i].color_main_idx].r,
                                 COLOR_PALETTE_RGB[g_led[i].color_main_idx].g,
                                 COLOR_PALETTE_RGB[g_led[i].color_main_idx].b);
        led_apply_second_color(i, COLOR_PALETTE_RGB[g_led[i].color_second_idx].r,
                                   COLOR_PALETTE_RGB[g_led[i].color_second_idx].g,
                                   COLOR_PALETTE_RGB[g_led[i].color_second_idx].b);
        if (g_led[i].enabled) {
            led_apply_enabled(i, true);
        }
    }
    refresh_led_screen_controls();

    lv_obj_add_event_cb(scr_led, swipe_gesture_cb, LV_EVENT_GESTURE, NULL);
}

// Rampe fuer die Start-Testanimation: 0 -> max (ANIM_UP_MS) -> 0 (ANIM_DOWN_MS),
// erst nach ANIM_DELAY_MS Wartezeit
static float anim_ramp(float max_val, uint32_t elapsed)
{
    if (elapsed < ANIM_DELAY_MS) {
        return 0.0f;
    } else if (elapsed < ANIM_DELAY_MS + ANIM_UP_MS) {
        float ratio = (float)(elapsed - ANIM_DELAY_MS) / (float)ANIM_UP_MS;
        return max_val * ratio;
    } else if (elapsed < ANIM_TOTAL_MS) {
        float ratio = (float)(elapsed - ANIM_DELAY_MS - ANIM_UP_MS) / (float)ANIM_DOWN_MS;
        return max_val * (1.0f - ratio);
    }
    return 0.0f;
}

static void update_timer_cb(lv_timer_t *t)
{
    BMW_UI_Update();
}

void BMW_UI_Init(lv_obj_t *demo_screen)
{
    // Vor jedem UI-Aufbau zuerst die zuletzt gespeicherten Einstellungen aus
    // NVS laden (Farben, Datenquelle, Logging, Anzeige-Umschalter, Summer) -
    // alle folgenden Konstruktionsschritte lesen direkt die betroffenen
    // Globals, es muss hier nichts weiter angewendet werden.
    bmw_settings_load();

    scr_demo = demo_screen;

    scr_multi = lv_obj_create(NULL);
    set_dark_bg(scr_multi);
    lv_obj_clear_flag(scr_multi, LV_OBJ_FLAG_SCROLLABLE);

    // Statisches Hintergrundbild (RPM-Ring, Skala, Farbverlauf) - fest im Bild.
    // Im BMW-Style ausgeblendet (siehe apply_bmw_style()), deshalb als
    // statischer Zeiger gehalten statt als lokale Variable.
    multi_bg_img_obj = lv_img_create(scr_multi);
    lv_img_set_src(multi_bg_img_obj, &multi_bg_img);
    lv_obj_center(multi_bg_img_obj);

    // Foto-Hintergrundbild fuer den BMW-Style (echtes Kombiinstrument) - nur im
    // BMW-Style sichtbar (siehe apply_bmw_style()), sonst ausgeblendet.
    multi_bg_bmw_img_obj = lv_img_create(scr_multi);
    lv_img_set_src(multi_bg_bmw_img_obj, &multi_bg_bmw_img);
    // WICHTIG (Performance): KEIN lv_img_set_zoom() hier. Das Foto ist bereits
    // exakt 466x466 (= volle Bildschirmgroesse), ein Zoom ist also gar nicht
    // noetig. Zuvor stand hier zoom=262 (~+2.3%, rein kosmetische ~5px-
    // Vergroesserung) - das zwang LVGL aber bei JEDEM Teil-Redraw ueber dem
    // Hintergrund in den teuren Transformations-Renderpfad (Pixel einzeln
    // skalieren + wahlfreier Flash-Zugriff auf die Quelldaten) statt in den
    // schnellen 1:1-Blit. Da die Nadel-Updates (100 ms-Timer) staendig kleine
    // Bereiche ueber diesem Bild neu zeichnen, wurde der gesamte BMW-Style-
    // Screen dadurch spuerbar traege (Wischen kaum moeglich, Drehzahl-/Nadel-
    // Aenderungen stark verzoegert) - der normale Multiview-Hintergrund
    // (multi_bg_img) wird ohne Zoom gezeichnet und ist deshalb fluessig. Ohne
    // Zoom fuellt das Foto den Schirm 1:1 und alle Redraws nehmen den schnellen
    // Pfad. Kleiner y-Versatz bleibt (kostet nichts, verschiebt nur die Position).
    lv_obj_align(multi_bg_bmw_img_obj, LV_ALIGN_CENTER, 0, 4);
    lv_obj_add_flag(multi_bg_bmw_img_obj, LV_OBJ_FLAG_HIDDEN);

    // Transparentes Meter nur fuer die Nadel-Winkelberechnung
    multi_meter = lv_meter_create(scr_multi);
    lv_obj_set_style_bg_opa(multi_meter, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(multi_meter, 0, 0);
    lv_obj_center(multi_meter);
    lv_obj_set_size(multi_meter, 450, 450);
    lv_obj_clear_flag(multi_meter, LV_OBJ_FLAG_SCROLLABLE);
    // Reine Anzeige - nicht klickbar, damit Touches/Wischgesten nie vom
    // Meter abgefangen werden (siehe center_hold / apply_bmw_style()).
    lv_obj_clear_flag(multi_meter, LV_OBJ_FLAG_CLICKABLE);

    // Multi-Nadel in der eingestellten Sekundaerfarbe (Standard: Rot) + weisses Hub-Cap
    const lv_img_dsc_t *init_needle_img;
    lv_coord_t init_pivot_x, init_pivot_y;
    get_base_needle_img(&init_needle_img, &init_pivot_x, &init_pivot_y);
    multi_needle = add_image_needle(multi_meter, MULTI_WATER_SCALE_MIN, MULTI_WATER_SCALE_MAX,
                                     init_needle_img, init_pivot_x, init_pivot_y, &multi_needle_scale);

    // Farbring: eigene Skala statt der Nadel-Hilfsskala. Die Nadel ist ein
    // Bild (LV_METER_INDICATOR_TYPE_NEEDLE_IMG): LVGL nutzt den Skalenwinkel
    // direkt als Bild-Rotationswinkel, und das Bild ruht bei Rotation 0 nach
    // 6 Uhr (90 Grad in der "0=rechts"-Skalenkonvention) ausgerichtet - die
    // sichtbare Nadelrichtung ist also Skalenwinkel+90. Die Nadel-Hilfsskala
    // gleicht das mit rotation=45 (statt 135) aus, damit ihre Ruhestellung
    // zur Skalenluecke im Hintergrundbild passt. Bogen-Indikatoren (Arcs)
    // bekommen diese +90-Korrektur nicht, sie nutzen den Skalenwinkel direkt
    // - mit rotation=45 lag der Ring deshalb um 90 Grad gegenueber der Nadel
    // verschoben. Eigene Skala mit rotation=135 (=45+90) kompensiert das, der
    // Ring beginnt jetzt exakt dort, wo die Nadel bei ihrem Minimalwert steht.
    // r_mod schiebt den Ring nach aussen. War zuletzt auf 24 (volle Ringbreite
    // von 8px plus Puffer), das lag zu weit aussen - um die halbe Ringbreite
    // (4px) wieder nach innen auf 20 korrigiert.
    multi_ring_scale = lv_meter_add_scale(multi_meter);
    lv_meter_set_scale_range(multi_meter, multi_ring_scale,
                              MULTI_WATER_SCALE_MIN, MULTI_WATER_SCALE_MAX, 270, 135);
    multi_ring_yellow = lv_meter_add_arc(multi_meter, multi_ring_scale, 8, lv_color_make(224, 255, 0), 20);
    multi_ring_orange = lv_meter_add_arc(multi_meter, multi_ring_scale, 8, lv_palette_main(LV_PALETTE_ORANGE), 20);
    multi_ring_red     = lv_meter_add_arc(multi_meter, multi_ring_scale, 8, lv_palette_main(LV_PALETTE_RED), 20);

    // Innere Leuchtskala (Kuehlmitteltemperatur, immer aktiv - siehe
    // Deklaration oben): anders als der volle Haupt-Ring nur noch ein 110 Grad
    // breiter Bogen unten (Vorbild: "HIGH TEMP"-Bogen im Foto-Hintergrund
    // multi_bg_bmw_img), der mit steigender Temperatur gegen den Uhrzeigersinn
    // von unten-links nach unten-rechts waechst (siehe MULTI_WATER_ARC_* und
    // WMIR() oben sowie BMW_UI_Update()). r_mod=-60 (Radius ~165 statt vormals
    // ~115 bei der fruiheren vollen Kreis-Variante) platziert den Bogen knapp
    // unterhalb der Schaltanzeige-Kaestchen, oberhalb der zweiten Wasser-Nadel.
    // Blau als Grundfarbe statt Neongelb (Vorbild: Foto der BMW-Segmentanzeige),
    // dieselben Schwellen wie beim Haupt-Ring (ab 100 Grad Orange, ab 115 Grad Rot).
    multi_water_inner_scale = lv_meter_add_scale(multi_meter);
    lv_meter_set_scale_range(multi_meter, multi_water_inner_scale,
                              MULTI_WATER_SCALE_MIN, MULTI_WATER_SCALE_MAX,
                              MULTI_WATER_ARC_RANGE, MULTI_WATER_ARC_ROTATION);
    // r_mod -48 (vormals -60): Bogen ~12px weiter nach aussen/unten, damit er
    // naeher an der unteren gelben "HIGH TEMP"-Skala des Foto-Hintergrunds
    // liegt (Nutzer-Rueckmeldung Foto v1.0.68).
    multi_water_inner_blue   = lv_meter_add_arc(multi_meter, multi_water_inner_scale, 10, lv_palette_main(LV_PALETTE_BLUE), -48);
    multi_water_inner_orange = lv_meter_add_arc(multi_meter, multi_water_inner_scale, 10, lv_palette_main(LV_PALETTE_ORANGE), -48);
    multi_water_inner_red     = lv_meter_add_arc(multi_meter, multi_water_inner_scale, 10, lv_palette_main(LV_PALETTE_RED), -48);

    lv_obj_t *hub = lv_obj_create(multi_meter);
    lv_obj_set_size(hub, 14, 14);
    lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hub, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(hub, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hub, 0, 0);
    lv_obj_center(hub);
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_CLICKABLE);

    // Geschwindigkeit gross in Primaerfarbe
    multi_speed_label = lv_label_create(scr_multi);
    lv_obj_set_style_text_font(multi_speed_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(multi_speed_label, g_color_primary, 0);
    lv_obj_align(multi_speed_label, LV_ALIGN_CENTER, 0, -50);
    lv_label_set_text(multi_speed_label, "0");

    multi_unit_label = lv_label_create(scr_multi);
    lv_label_set_text(multi_unit_label, "KM/H");
    lv_obj_set_style_text_color(multi_unit_label, lv_color_white(), 0);
    lv_obj_align(multi_unit_label, LV_ALIGN_CENTER, 0, -15);

    // RX (gruen) / TX (rot) Aktivitaetspunkte unter KM/H, nur bei BLE-OBD
    multi_rx_dot = lv_obj_create(scr_multi);
    multi_tx_dot = lv_obj_create(scr_multi);
    lv_obj_t *dots[2] = {multi_rx_dot, multi_tx_dot};
    for (int i = 0; i < 2; i++) {
        lv_obj_set_size(dots[i], 12, 12);
        lv_obj_set_style_radius(dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(dots[i], 0, 0);
        lv_obj_set_style_bg_opa(dots[i], LV_OPA_COVER, 0);
        lv_obj_clear_flag(dots[i], LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_align(dots[i], LV_ALIGN_CENTER, i == 0 ? -14 : 14, 12);
        lv_obj_add_flag(dots[i], LV_OBJ_FLAG_HIDDEN);
    }

    // 4 Zusatzfelder (Batterie/Gaspedal/Drehzahl/Wasser)
    const int field_x[4] = {-165, -55, 55, 165};

    // Schwarze Umrandungs-Kopien VOR den eigentlichen Labels erzeugen, damit
    // sie im Z-Order dahinter liegen (2px-Versatz in 8 Richtungen) - fuer
    // alle 4 Zusatzfelder (Batterie/Gaspedal/Drehzahl/Wasser)
    static const lv_coord_t outline_off[8][2] = {
        {-2, 0}, {2, 0}, {0, -2}, {0, 2}, {-2, -2}, {2, -2}, {-2, 2}, {2, 2},
    };
    lv_obj_t **outline_fields[4] = {
        multi_bat_outline, multi_throttle_outline, multi_rpm_outline, multi_water_outline,
    };
    for (int f = 0; f < 4; f++) {
        for (int i = 0; i < 8; i++) {
            lv_obj_t *lbl = lv_label_create(scr_multi);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
            lv_obj_set_style_text_color(lbl, lv_color_black(), 0);
            lv_obj_align(lbl, LV_ALIGN_CENTER,
                         field_x[f] + outline_off[i][0], 60 + outline_off[i][1]);
            lv_label_set_text(lbl, "-");
            outline_fields[f][i] = lbl;
        }
    }

    multi_bat_label      = lv_label_create(scr_multi);
    multi_throttle_label = lv_label_create(scr_multi);
    multi_rpm_label      = lv_label_create(scr_multi);
    multi_water_label    = lv_label_create(scr_multi);
    lv_obj_t *fields[4] = {multi_bat_label, multi_throttle_label, multi_rpm_label, multi_water_label};
    for (int i = 0; i < 4; i++) {
        lv_obj_set_style_text_color(fields[i], lv_color_white(), 0);
        lv_obj_align(fields[i], LV_ALIGN_CENTER, field_x[i], 60);
        lv_label_set_text(fields[i], "-");
    }

    // Symbol-Icons mittig ueber jedem der 4 Zusatzfelder (Batterie/Gaspedal/
    // Drehzahl/Wasser), passend zur jeweiligen Anzeige
    const lv_img_dsc_t *field_icons[4] = {&icon_battery_img, &icon_pedal_img, &icon_tacho_img, &icon_water_img};
    for (int i = 0; i < 4; i++) {
        multi_field_icon[i] = lv_img_create(scr_multi);
        lv_img_set_src(multi_field_icon[i], field_icons[i]);
        lv_obj_align(multi_field_icon[i], LV_ALIGN_CENTER, field_x[i], 32);
        lv_obj_clear_flag(multi_field_icon[i], LV_OBJ_FLAG_CLICKABLE);
    }
    // Batterie-, Gaspedal-, Drehzahl- und Wasser-Feld doppelt so gross (Font 28 statt Standard 14)
    lv_obj_set_style_text_font(multi_bat_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_font(multi_throttle_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_font(multi_rpm_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_font(multi_water_label, &lv_font_montserrat_28, 0);

    // Schaltanzeige-Kaestchen (initial leer/transparent, ueber den
    // Hintergrund-Kaesten positioniert)
    for (int i = 0; i < 6; i++) {
        rpm_boxes[i] = lv_obj_create(scr_multi);
        lv_obj_set_size(rpm_boxes[i], 32, 16);
        lv_obj_align(rpm_boxes[i], LV_ALIGN_CENTER, rpm_box_x[i], 172);
        lv_obj_set_style_radius(rpm_boxes[i], 3, 0);
        lv_obj_set_style_border_width(rpm_boxes[i], 0, 0);
        lv_obj_set_style_bg_color(rpm_boxes[i], rpm_box_col[i], 0);
        lv_obj_set_style_bg_opa(rpm_boxes[i], LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(rpm_boxes[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(rpm_boxes[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // Zweite, kleine Analognadel ausschliesslich fuer Wassertemperatur
    // (BMW-Style, siehe apply_bmw_style()) - eigenstaendiges Meter-Widget,
    // da LVGL-Nadeln immer um den Mittelpunkt ihres eigenen Meter-Objekts
    // rotieren und sich nicht an einer beliebigen Stelle innerhalb von
    // multi_meter platzieren lassen. Nutzt eigene, kleine BMW-Style-Nadel-
    // Bilder (needle_small_*_img, aus Fotos des echten BMW-Tachozeigers
    // generiert, siehe get_small_needle_img()) statt der grossen
    // needle_*_img-Rundinstrument-Nadeln, die hier zuvor stark ueberdimensioniert
    // waren (87x276px in einem 110px-Widget) und den BMW-Style-Screen spuerbar
    // ausgebremst hatten. Standardmaessig ausgeblendet - apply_bmw_style()
    // schaltet sie frei. y-Offset von vormals 155 auf 135 verringert (Zentrum
    // etwas hoeher), naeher am "HIGH TEMP"-Bogen im Foto.
    multi_bmw_water_meter = lv_meter_create(scr_multi);
    lv_obj_set_style_bg_opa(multi_bmw_water_meter, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(multi_bmw_water_meter, 0, 0);
    lv_obj_set_size(multi_bmw_water_meter, MULTI_BMW_WATER_METER_SIZE, MULTI_BMW_WATER_METER_SIZE);
    lv_obj_align(multi_bmw_water_meter, LV_ALIGN_CENTER, 0, 120);
    lv_obj_clear_flag(multi_bmw_water_meter, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(multi_bmw_water_meter, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(multi_bmw_water_meter, LV_OBJ_FLAG_HIDDEN);

    // rotation=MULTI_WATER_NEEDLE_ROTATION + range=MULTI_WATER_ARC_RANGE (110
    // statt 270): dieselbe "+90"-Bildkorrektur wie bei der grossen Nadel
    // (sichtbare Richtung = Skalenwinkel+90), aber auf den schmalen 110-Grad-
    // Bogen unten zugeschnitten, damit die sichtbare Nadelrichtung exakt dem
    // blauen Bogen (multi_water_inner_*) folgt. BMW_UI_Update() speist dafuer
    // den gespiegelten Temperaturwert (WMIR()) statt des Rohwerts ein, damit
    // die Nadel wie der Bogen gegen den Uhrzeigersinn waechst.
    const lv_img_dsc_t *small_needle_img;
    lv_coord_t small_pivot_x, small_pivot_y;
    get_small_needle_img(&small_needle_img, &small_pivot_x, &small_pivot_y);
    multi_bmw_water_needle = add_image_needle_ex(multi_bmw_water_meter, MULTI_WATER_SCALE_MIN, MULTI_WATER_SCALE_MAX,
                                               small_needle_img, small_pivot_x, small_pivot_y,
                                               MULTI_WATER_ARC_RANGE, MULTI_WATER_NEEDLE_ROTATION,
                                               &multi_bmw_water_needle_scale);

    lv_obj_t *bmw_water_hub = lv_obj_create(multi_bmw_water_meter);
    lv_obj_set_size(bmw_water_hub, 10, 10);
    lv_obj_set_style_radius(bmw_water_hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(bmw_water_hub, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(bmw_water_hub, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bmw_water_hub, 0, 0);
    lv_obj_center(bmw_water_hub);
    lv_obj_clear_flag(bmw_water_hub, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(bmw_water_hub, LV_OBJ_FLAG_CLICKABLE);

    // Unsichtbares Center-Overlay: 3-Sekunden-Halten -> Demo-Seite,
    // Doppeltipp -> Einstellungs-Screen (Farben). Vergroessert (160 -> 260
    // -> 360), da der Treffbereich dem Nutzer zu klein war. Wischgesten
    // funktionieren weiterhin, da sie zu scr_multi hochblubbern.
    center_hold = lv_obj_create(scr_multi);
    lv_obj_set_size(center_hold, 360, 360);
    lv_obj_center(center_hold);
    lv_obj_set_style_bg_opa(center_hold, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(center_hold, 0, 0);
    lv_obj_clear_flag(center_hold, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(center_hold, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(center_hold, center_touch_cb, LV_EVENT_ALL, NULL);
    lv_obj_move_foreground(center_hold);

    // Wisch nach links -> Fehlercode-Screen
    lv_obj_add_event_cb(scr_multi, swipe_gesture_cb, LV_EVENT_GESTURE, NULL);

    // "Zurueck"-Button + Datenquellen-Auswahl (BLE-OBD2/MCP2515) oben auf
    // dem Demo-Screen (ueber dem Tabview, da als letzte Kinder von scr_demo
    // erzeugt -> liegen im Z-Order oben). Tiefer gesetzt (y=55) und kleiner
    // als zuvor, damit sie nicht mit der Tab-Leiste des Waveshare-Demos
    // ueberlappen. Kraeftige rote Einfaerbung beim Zurueck-Button, damit er
    // sich sichtbar von der Tab-Leiste abhebt.
    // Alle drei Buttons mittig (statt rechtsbuendig) ueber der Tab-Leiste
    // platziert: Bei TOP_RIGHT-Ausrichtung nahe der oberen rechten Ecke
    // wurden sie von der runden Displayabdeckung angeschnitten (Foto des
    // Nutzers: "BLE OBD" halb, "Zurueck" fast komplett abgeschnitten). Bei
    // y=55 betraegt die sichtbare Breite innerhalb der runden Aussparung nur
    // noch ca. 300px um die Bildschirmmitte (x=240) - die Gruppe (70+70+78
    // plus 2x8px Abstand = 234px) passt dort zentriert bequem hinein.
    lv_obj_t *back_btn = lv_btn_create(scr_demo);
    lv_obj_set_size(back_btn, 78, 30);
    lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 279, 55);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0xCC2222), 0);
    lv_obj_set_style_bg_opa(back_btn, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(back_btn, back_to_multi_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, "Zurueck");
    lv_obj_set_style_text_color(back_lbl, lv_color_white(), 0);
    lv_obj_center(back_lbl);

    btn_src_mcp = lv_btn_create(scr_demo);
    lv_obj_set_size(btn_src_mcp, 70, 30);
    lv_obj_align(btn_src_mcp, LV_ALIGN_TOP_LEFT, 201, 55);
    lv_obj_set_style_bg_opa(btn_src_mcp, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(btn_src_mcp, src_mcp_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *src_mcp_lbl = lv_label_create(btn_src_mcp);
    lv_label_set_text(src_mcp_lbl, "MCP");
    lv_obj_set_style_text_color(src_mcp_lbl, lv_color_white(), 0);
    lv_obj_center(src_mcp_lbl);

    btn_src_ble = lv_btn_create(scr_demo);
    lv_obj_set_size(btn_src_ble, 70, 30);
    lv_obj_align(btn_src_ble, LV_ALIGN_TOP_LEFT, 123, 55);
    lv_obj_set_style_bg_opa(btn_src_ble, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(btn_src_ble, src_ble_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *src_ble_lbl = lv_label_create(btn_src_ble);
    lv_label_set_text(src_ble_lbl, "BLE OBD");
    lv_obj_set_style_text_color(src_ble_lbl, lv_color_white(), 0);
    lv_obj_center(src_ble_lbl);

    update_source_btn_styles(); // Standard: BLE OBD (gruen) aktiv markieren

    // Zweiter BMW-Style-Umschalter direkt auf dem Langdruck-/Demo-Screen (vom
    // Nutzer gewuenscht): der Demo-Screen reagiert zuverlaessig, auch wenn der
    // BMW-Style-Hauptscreen gerade traege ist - so laesst sich der Stil immer
    // wieder abschalten, ohne erst per Doppeltipp in die Einstellungen zu muessen.
    // Mittig unter der Button-Reihe, innerhalb der runden Displayaussparung.
    lv_obj_t *demo_style_label = lv_label_create(scr_demo);
    lv_label_set_text(demo_style_label, "BMW Style");
    lv_obj_set_style_text_color(demo_style_label, lv_color_white(), 0);
    lv_obj_align(demo_style_label, LV_ALIGN_TOP_MID, -34, 95);

    lv_obj_t *demo_style_switch = lv_switch_create(scr_demo);
    lv_obj_align(demo_style_switch, LV_ALIGN_TOP_MID, 45, 93);
    lv_obj_add_event_cb(demo_style_switch, bmw_style_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);
    g_bmw_style_sw[1] = demo_style_switch;
    if (g_bmw_style_enabled) {
        lv_obj_add_state(demo_style_switch, LV_STATE_CHECKED);
    }

    create_settings_screen();
    create_settings_func_screen();
    create_dtc_screen();
    create_service_screen();
    create_sensors_screen();
    create_led_screen();

    anim_start_tick = lv_tick_get();
    anim_done = false;

    BMW_UI_Update();
    lv_scr_load(scr_multi);

    // Zyklische Aktualisierung der Live-Werte (100 ms)
    lv_timer_create(update_timer_cb, 100, NULL);
}

void BMW_UI_Update(void)
{
    // Start-Testanimation: 2s warten, dann alle Anzeigen in 4s auf Anschlag
    // hochfahren und in 2s wieder auf 0 zurueck - danach Live-/Platzhalterwerte
    if (!anim_done) {
        uint32_t elapsed = lv_tick_elaps(anim_start_tick);
        if (elapsed >= ANIM_TOTAL_MS) {
            anim_done = true;
        } else {
            current_speed_kmh    = anim_ramp(ANIM_SPEED_MAX, elapsed);
            current_water_temp   = anim_ramp(ANIM_WATER_MAX, elapsed);
            current_bat_voltage  = anim_ramp(ANIM_BAT_MAX, elapsed);
            current_throttle_pct = anim_ramp(ANIM_THROTTLE_MAX, elapsed);
            current_rpm          = anim_ramp(ANIM_RPM_MAX, elapsed);
        }
    }

    // Datenquelle (BLE-OBD2 oder MCP2515-CAN) fuer Motordaten UND
    // Batteriespannung - beide kommen live vom Fahrzeug, nicht vom
    // Board-ADC (der misst nur die interne Versorgungsspannung des
    // Displays, nicht die Bordnetzspannung des Autos).
    bool use_ble_src = (g_data_source == DATA_SRC_BLE_OBD);
    float obd2_bat = use_ble_src ? BLE_OBD_bat_voltage() : CAN_OBD2_bat_voltage();

    if (anim_done) {
        // Motordaten live von der eingestellten Datenquelle, falls online -
        // sonst bleiben die Platzhalter stehen
        bool source_online = use_ble_src ? BLE_OBD_online() : CAN_OBD2_online();
        if (source_online) {
            current_speed_kmh    = use_ble_src ? BLE_OBD_speed_kmh()    : CAN_OBD2_speed_kmh();
            current_rpm          = use_ble_src ? BLE_OBD_rpm()          : CAN_OBD2_rpm();
            current_water_temp   = use_ble_src ? BLE_OBD_water_temp()   : CAN_OBD2_water_temp();
            current_throttle_pct = use_ble_src ? BLE_OBD_throttle_pct() : CAN_OBD2_throttle_pct();
        }
        if (obd2_bat > 0.0f) current_bat_voltage = obd2_bat;
    }

    // Grosse Ring+Nadel-Anzeige: je nach Einstellungs-Umschalter Wasser-
    // temperatur (Standard) oder Drehzahl
    float big_gauge_val = (g_multi_gauge_mode == MULTI_GAUGE_RPM) ? current_rpm : current_water_temp;
    lv_meter_set_indicator_value(multi_meter, multi_needle, (int32_t)big_gauge_val);
    lv_label_set_text_fmt(multi_speed_label, "%d", (int)current_speed_kmh);
    {
        // lv_label_set_text_fmt nutzt LVGLs eigenen (v)snprintf, der ohne
        // CONFIG_LV_SPRINTF_USE_FLOAT kein "%f" versteht und dann nur die
        // literalen Format-Reste ("fV") ausgibt - deshalb hier echtes
        // snprintf (newlib, mit Float-Unterstuetzung) in einen Puffer.
        // Solange noch keine OBD2-Antwort da war (und die Startanimation
        // vorbei ist), zeigt das Feld den BLE-Verbindungsstatus statt einer
        // erfundenen Spannung.
        char buf[16];
        if (!anim_done || obd2_bat > 0.0f) {
            snprintf(buf, sizeof(buf), "%.1fV", current_bat_voltage);
            lv_label_set_text(multi_bat_label, buf);
            for (int i = 0; i < 8; i++) lv_label_set_text(multi_bat_outline[i], buf);
        } else if (use_ble_src) {
            lv_label_set_text_fmt(multi_bat_label, "BLE: %s", BLE_OBD_status());
            for (int i = 0; i < 8; i++) {
                lv_label_set_text_fmt(multi_bat_outline[i], "BLE: %s", BLE_OBD_status());
            }
        } else {
            lv_label_set_text(multi_bat_label, "-");
            for (int i = 0; i < 8; i++) lv_label_set_text(multi_bat_outline[i], "-");
        }
    }
    lv_label_set_text_fmt(multi_throttle_label, "%d%%", (int)current_throttle_pct);
    lv_label_set_text_fmt(multi_rpm_label, "%d", (int)current_rpm);
    lv_label_set_text_fmt(multi_water_label, "%d\xC2\xB0""C", (int)current_water_temp);
    lv_obj_set_style_text_color(multi_water_label,
        current_water_temp >= 110.0f ? lv_palette_main(LV_PALETTE_ORANGE) : lv_color_white(), 0);

    // Warnsummer ab 120 Grad Kuehlmitteltemperatur, im Einstellungs-Screen
    // "Funktionen" ein-/ausschaltbar. g_buzzer_active verhindert wiederholte
    // Buzzer_On()/_Off()-Aufrufe (I2C-Schreibzugriff auf den TCA9554-IO-
    // Expander) bei jedem 100-ms-Update-Tick, solange der Zustand gleich bleibt.
    bool want_buzzer = g_buzzer_enabled && current_water_temp >= 120.0f;
    if (want_buzzer && !g_buzzer_active) {
        Buzzer_On();
        g_buzzer_active = true;
    } else if (!want_buzzer && g_buzzer_active) {
        Buzzer_Off();
        g_buzzer_active = false;
    }

    // "Hotspot verbinden": Statustext live aus sntp_sync.c, Schalter faellt
    // automatisch wieder ab, sobald der einmalige Versuch fertig ist (Erfolg,
    // Fehler oder Abbruch) - er ist kein dauerhafter "verbunden"-Zustand.
    {
        static bool last_sntp_busy = false;
        bool sntp_busy = SNTP_Sync_IsBusy();
        lv_label_set_text(hotspot_status_label, SNTP_Sync_Status());
        if (last_sntp_busy && !sntp_busy) {
            lv_obj_clear_state(hotspot_switch, LV_STATE_CHECKED);
        }
        last_sntp_busy = sntp_busy;
    }

    // "SD formatieren": Statustext erst nach Abschluss des Hintergrund-Tasks
    // setzen (der Task selbst ruft keine LVGL-Funktionen auf, siehe
    // sd_format_task()).
    {
        static bool last_format_busy = false;
        bool format_busy = s_sd_format_busy;
        if (last_format_busy && !format_busy) {
            lv_label_set_text(sd_format_status_label, s_sd_format_ok ? "Fertig" : "Fehler!");
        }
        last_format_busy = format_busy;
    }

    for (int i = 0; i < 8; i++) {
        lv_label_set_text_fmt(multi_throttle_outline[i], "%d%%", (int)current_throttle_pct);
        lv_label_set_text_fmt(multi_rpm_outline[i], "%d", (int)current_rpm);
        lv_label_set_text_fmt(multi_water_outline[i], "%d\xC2\xB0""C", (int)current_water_temp);
    }

    // RX/TX-Punkte: erscheinen nach dem ersten empfangenen Datum, leuchten
    // kurz bei jedem Senden (rot) bzw. Empfangen (gruen)
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t rx_ms = BLE_OBD_last_rx_ms();
    uint32_t tx_ms = BLE_OBD_last_tx_ms();
    bool show_dots = use_ble_src && rx_ms != 0;
    for (int i = 0; i < 2; i++) {
        lv_obj_t *dot = i == 0 ? multi_rx_dot : multi_tx_dot;
        if (!show_dots) { lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN); continue; }
        uint32_t t = i == 0 ? rx_ms : tx_ms;
        bool lit = t != 0 && (now_ms - t) < ACT_DOT_FLASH_MS;
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(dot, lit
            ? lv_palette_main(i == 0 ? LV_PALETTE_GREEN : LV_PALETTE_RED)
            : lv_palette_darken(i == 0 ? LV_PALETTE_GREEN : LV_PALETTE_RED, 4), 0);
    }

    // Datenlogging auf SD-Karte (CSV-Zeile alle LOG_INTERVAL_MS)
    if (log_file && lv_tick_elaps(log_last_tick) >= LOG_INTERVAL_MS) {
        log_last_tick = lv_tick_get();
        float gx = use_ble_src ? BLE_OBD_gforce_x() : CAN_OBD2_gforce_x();
        float gy = use_ble_src ? BLE_OBD_gforce_y() : CAN_OBD2_gforce_y();
        fprintf(log_file, "%lu;%.1f;%.0f;%.1f;%.0f;%.2f;%.2f;%.2f\r\n",
                (unsigned long)lv_tick_get(), current_speed_kmh, current_rpm,
                current_water_temp, current_throttle_pct, obd2_bat, gx, gy);
        fflush(log_file);
        // Siehe Kommentar in start_datalogging(): ohne fsync() bleibt jede
        // Zeile im FatFs-Cache, bis das Zuendungs-Abschalten den Strom ohne
        // Vorwarnung wegnimmt - dann ist sie verloren, auch wenn fopen() hier
        // als erfolgreich geloggt wurde.
        if (fsync(fileno(log_file)) != 0) {
            // Karte waehrend der Fahrt verschwunden/defekt - Handle ist tot.
            // Weiter alle 500ms blind draufschreiben wuerde nur Zeit kosten
            // und eine danach neu eingesteckte Karte nie erkennen lassen
            // (SD_HandleIoError() setzt SDCard_Size zurueck, siehe SD_MMC.c).
            SD_Log("Datenlogging: fsync() fehlgeschlagen, errno=%d (%s) - Logging gestoppt",
                   errno, strerror(errno));
            fclose(log_file);
            log_file = NULL;
            SD_HandleIoError();
            lv_label_set_text(log_status_label, "SD-Fehler!");
            if (log_switch) lv_obj_clear_state(log_switch, LV_STATE_CHECKED);
            g_datalog_enabled = false;
            bmw_settings_save();
        }
    }

    // Fehlercode-Liste auf dem DTC-Screen aktualisieren
    int dtc_count = use_ble_src ? BLE_OBD_dtc_count() : CAN_OBD2_dtc_count();
    if (dtc_count < 0) {
        lv_label_set_text(dtc_list_label, "Noch nicht ausgelesen");
    } else if (dtc_count == 0) {
        lv_label_set_text(dtc_list_label, "Keine Fehler gespeichert");
    } else {
        char buf[8 * 7 + 1] = {0};
        int pos = 0;
        for (int i = 0; i < dtc_count; i++) {
            const char *code = use_ble_src ? BLE_OBD_dtc_code(i) : CAN_OBD2_dtc_code(i);
            if (!code) break;
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%s%s", i > 0 ? "\n" : "", code);
        }
        lv_label_set_text(dtc_list_label, buf);
    }

    // Sensoren-Screen: Lambda/Ansaugdruck/Ladedruck/Oel-Temp/MIL leben,
    // solange die Quelle online ist, sonst bleiben die Felder auf "---"
    // stehen. Nockenwellen-Position ist bereits einmalig in
    // create_sensors_screen() auf "n/v" gesetzt und wird hier nicht mehr
    // angefasst (sie aendert sich nie).
    {
        bool sensors_online = use_ble_src ? BLE_OBD_online() : CAN_OBD2_online();
        char sbuf[16];
        if (sensors_online) {
            float l1r = use_ble_src ? BLE_OBD_lambda1_ratio()   : CAN_OBD2_lambda1_ratio();
            float l1v = use_ble_src ? BLE_OBD_lambda1_voltage() : CAN_OBD2_lambda1_voltage();
            float l2r = use_ble_src ? BLE_OBD_lambda2_ratio()   : CAN_OBD2_lambda2_ratio();
            float l2v = use_ble_src ? BLE_OBD_lambda2_voltage() : CAN_OBD2_lambda2_voltage();
            float map_kpa = use_ble_src ? BLE_OBD_intake_pressure() : CAN_OBD2_intake_pressure();
            float boost_kpa = use_ble_src ? BLE_OBD_boost_pressure() : CAN_OBD2_boost_pressure();
            float oil_c = use_ble_src ? BLE_OBD_oil_temp() : CAN_OBD2_oil_temp();
            bool mil_on = use_ble_src ? BLE_OBD_mil_on() : CAN_OBD2_mil_on();

            snprintf(sbuf, sizeof(sbuf), "%.2f", l1r);
            lv_label_set_text(sens_lambda1_val, sbuf);
            snprintf(sbuf, sizeof(sbuf), "%.2f", l2r);
            lv_label_set_text(sens_lambda2_val, sbuf);
            snprintf(sbuf, sizeof(sbuf), "%.2fV", l1v);
            lv_label_set_text(sens_lambda1_volt_val, sbuf);
            snprintf(sbuf, sizeof(sbuf), "%.2fV", l2v);
            lv_label_set_text(sens_lambda2_volt_val, sbuf);
            snprintf(sbuf, sizeof(sbuf), "%.0f kPa", map_kpa);
            lv_label_set_text(sens_map_val, sbuf);

            if (boost_kpa <= -1.0f) {
                lv_label_set_text(sens_boost_val, "n/v");
            } else {
                snprintf(sbuf, sizeof(sbuf), "%+.0f kPa", boost_kpa);
                lv_label_set_text(sens_boost_val, sbuf);
            }

            if (oil_c <= -900.0f) {
                lv_label_set_text(sens_oil_val, "n/v");
            } else {
                snprintf(sbuf, sizeof(sbuf), "%d\xC2\xB0""C", (int)oil_c);
                lv_label_set_text(sens_oil_val, sbuf);
            }

            lv_label_set_text(sens_mil_val, mil_on ? "AN" : "AUS");
            lv_obj_set_style_text_color(sens_mil_val,
                mil_on ? lv_color_make(230, 50, 50) : lv_color_make(80, 200, 100), 0);
        } else {
            lv_label_set_text(sens_lambda1_val, "---");
            lv_label_set_text(sens_lambda2_val, "---");
            lv_label_set_text(sens_lambda1_volt_val, "---");
            lv_label_set_text(sens_lambda2_volt_val, "---");
            lv_label_set_text(sens_map_val, "---");
            lv_label_set_text(sens_boost_val, "---");
            lv_label_set_text(sens_oil_val, "---");
            lv_label_set_text(sens_mil_val, "---");
            lv_obj_set_style_text_color(sens_mil_val, lv_color_white(), 0);
        }
    }

    // Schaltanzeige: jedes Kaestchen fuellt sich (in seiner Umrandungsfarbe)
    // erst ab seiner eigenen Drehzahlschwelle (rpm_box_thr). Ab RPM_SHIFT_BLINK
    // (6800 U/min) leuchten alle gemeinsam blinkend im 250-ms-Takt auf wie eine
    // digitale Schaltanzeige.
    bool shift_blink = (current_rpm >= RPM_SHIFT_BLINK);
    lv_opa_t blink_opa = LV_OPA_COVER;
    if (shift_blink) {
        static bool box_blink_state = true;
        static uint32_t box_last_blink = 0;
        if (lv_tick_elaps(box_last_blink) > 250) {
            box_blink_state = !box_blink_state;
            box_last_blink = lv_tick_get();
        }
        blink_opa = box_blink_state ? LV_OPA_COVER : LV_OPA_TRANSP;
    }
    for (int i = 0; i < 6; i++) {
        lv_opa_t opa;
        if (shift_blink) {
            opa = blink_opa;
        } else {
            opa = (current_rpm >= (float)rpm_box_thr[i]) ? LV_OPA_COVER : LV_OPA_TRANSP;
        }
        lv_obj_set_style_bg_opa(rpm_boxes[i], opa, 0);
    }

    // Nadelfarbe + Farbring: nur relevant im Wasser-Modus der grossen Anzeige
    // (Schwellen sind auf Kuehlmitteltemperatur zugeschnitten, nicht auf
    // Drehzahl). Basis = eingestellte Sekundaerfarbe (Standard Rot) bis
    // 95 Grad, ab 95 Grad neongelb, ab 112 Grad blinkt die Nadel (250ms-Takt,
    // wie das Schaltpunkt-Blinken der Drehzahl-LEDs im C6-Projekt)
    const lv_img_dsc_t *base_needle_img;
    lv_coord_t base_pivot_x, base_pivot_y;
    get_base_needle_img(&base_needle_img, &base_pivot_x, &base_pivot_y);
    LV_UNUSED(base_pivot_x);
    LV_UNUSED(base_pivot_y);
    const void *needle_src = base_needle_img;
    lv_opa_t needle_opa = LV_OPA_COVER;
    if (g_multi_gauge_mode == MULTI_GAUGE_WATER && current_water_temp >= 95.0f) {
        needle_src = &multi_needle_yellow_img;
        if (current_water_temp >= 112.0f) {
            static bool blink_state = false;
            static uint32_t last_blink = 0;
            if (lv_tick_elaps(last_blink) > 250) {
                blink_state = !blink_state;
                last_blink = lv_tick_get();
            }
            needle_opa = blink_state ? LV_OPA_COVER : LV_OPA_TRANSP;
        }
    }
    if (multi_needle->type_data.needle_img.src != needle_src || multi_needle->opa != needle_opa) {
        multi_needle->type_data.needle_img.src = needle_src;
        multi_needle->opa = needle_opa;
        lv_obj_invalidate(multi_meter);
    }

    // Zweite Wasser-Nadel (BMW-Style, siehe apply_bmw_style()): zeigt immer
    // die Kuehlmitteltemperatur, unabhaengig von g_multi_gauge_mode - gleiche
    // Farb-/Blink-Schwellen wie die grosse Nadel im Wasser-Modus, nur mit den
    // kleinen Nadel-Bildern. Wird nur berechnet, solange der Stil aktiv ist.
    if (g_bmw_style_enabled) {
        // WMIR() spiegelt den Wert, damit die Nadel auf der bestehenden Skala gegen den Uhrzeigersinn wandert
        lv_meter_set_indicator_value(multi_bmw_water_meter, multi_bmw_water_needle, WMIR(current_water_temp));

        const lv_img_dsc_t *small_base_img;
        lv_coord_t small_base_px, small_base_py;
        get_small_needle_img(&small_base_img, &small_base_px, &small_base_py);
        LV_UNUSED(small_base_px);
        LV_UNUSED(small_base_py);
        const void *small_needle_src = small_base_img;
        lv_opa_t small_needle_opa = LV_OPA_COVER;
        if (current_water_temp >= 95.0f) {
            small_needle_src = &needle_small_yellow_img;
            if (current_water_temp >= 112.0f) {
                static bool small_blink_state = false;
                static uint32_t small_last_blink = 0;
                if (lv_tick_elaps(small_last_blink) > 250) {
                    small_blink_state = !small_blink_state;
                    small_last_blink = lv_tick_get();
                }
                small_needle_opa = small_blink_state ? LV_OPA_COVER : LV_OPA_TRANSP;
            }
        }
        if (multi_bmw_water_needle->type_data.needle_img.src != small_needle_src ||
            multi_bmw_water_needle->opa != small_needle_opa) {
            multi_bmw_water_needle->type_data.needle_img.src = small_needle_src;
            multi_bmw_water_needle->opa = small_needle_opa;
            lv_obj_invalidate(multi_bmw_water_meter);
        }
    }

    // Innere Leuchtskala (Kuehlmitteltemperatur): immer aktiv, unabhaengig
    // von g_multi_gauge_mode - zeigt die Wassertemperatur auch dann, wenn die
    // grosse Ring/Nadel-Anzeige gerade auf Drehzahl steht. Gleiche
    // Schwellen-Logik wie der Haupt-Ring im Wasser-Modus, nur auf die drei
    // blau/orange/rot-Indikatoren der inneren Skala angewendet. Start-/End-Werte
    // mit WMIR() gespiegelt und vertauscht (WMIR() kehrt die Reihenfolge um),
    // damit der Bogen wie die Skala (MULTI_WATER_ARC_ROTATION) gegen den
    // Uhrzeigersinn von unten-links (kalt) nach unten-rechts (heiss) waechst.
    {
        float t = current_water_temp;
        float blue_end = t < (float)MULTI_WATER_RING_ORANGE_AT ? t : (float)MULTI_WATER_RING_ORANGE_AT;
        if (blue_end < MULTI_WATER_SCALE_MIN) blue_end = MULTI_WATER_SCALE_MIN;
        lv_meter_set_indicator_start_value(multi_meter, multi_water_inner_blue, WMIR(blue_end));
        lv_meter_set_indicator_end_value(multi_meter, multi_water_inner_blue, WMIR(MULTI_WATER_SCALE_MIN));

        float orange_end = t < (float)MULTI_WATER_RING_RED_AT ? t : (float)MULTI_WATER_RING_RED_AT;
        if (orange_end < (float)MULTI_WATER_RING_ORANGE_AT) orange_end = (float)MULTI_WATER_RING_ORANGE_AT;
        lv_meter_set_indicator_start_value(multi_meter, multi_water_inner_orange, WMIR(orange_end));
        lv_meter_set_indicator_end_value(multi_meter, multi_water_inner_orange, WMIR(MULTI_WATER_RING_ORANGE_AT));

        float red_end = t > MULTI_WATER_SCALE_MAX ? MULTI_WATER_SCALE_MAX : t;
        if (red_end < (float)MULTI_WATER_RING_RED_AT) red_end = (float)MULTI_WATER_RING_RED_AT;
        lv_meter_set_indicator_start_value(multi_meter, multi_water_inner_red, WMIR(red_end));
        lv_meter_set_indicator_end_value(multi_meter, multi_water_inner_red, WMIR(MULTI_WATER_RING_RED_AT));
    }

    // Farbring: waechst von aussen entlang der Skalenteilstriche mit.
    // Wasser-Modus: Neongelb bis 100 Grad, ab 100 Grad Orange (das
    // Orange-Segment beginnt erst genau an dieser Stelle, kein Verblassen
    // davor), ab 115 Grad Rot. Drehzahl-Modus: dieselbe Logik auf die
    // 0-8000 U/min-Skala uebertragen, Orange ab 6500, Rot ab 6900 U/min.
    if (g_multi_gauge_mode == MULTI_GAUGE_WATER) {
        float t = current_water_temp;
        float yellow_end = t < (float)MULTI_WATER_RING_ORANGE_AT ? t : (float)MULTI_WATER_RING_ORANGE_AT;
        if (yellow_end < MULTI_WATER_SCALE_MIN) yellow_end = MULTI_WATER_SCALE_MIN;
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_yellow, MULTI_WATER_SCALE_MIN);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_yellow, (int32_t)yellow_end);

        float orange_end = t < (float)MULTI_WATER_RING_RED_AT ? t : (float)MULTI_WATER_RING_RED_AT;
        if (orange_end < (float)MULTI_WATER_RING_ORANGE_AT) orange_end = (float)MULTI_WATER_RING_ORANGE_AT;
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_orange, MULTI_WATER_RING_ORANGE_AT);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_orange, (int32_t)orange_end);

        float red_end = t > MULTI_WATER_SCALE_MAX ? MULTI_WATER_SCALE_MAX : t;
        if (red_end < (float)MULTI_WATER_RING_RED_AT) red_end = (float)MULTI_WATER_RING_RED_AT;
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_red, MULTI_WATER_RING_RED_AT);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_red, (int32_t)red_end);
    } else if (g_multi_gauge_mode == MULTI_GAUGE_RPM) {
        float t = current_rpm;
        float yellow_end = t < (float)MULTI_RPM_RING_ORANGE_AT ? t : (float)MULTI_RPM_RING_ORANGE_AT;
        if (yellow_end < 0.0f) yellow_end = 0.0f;
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_yellow, 0);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_yellow, (int32_t)yellow_end);

        float orange_end = t < (float)MULTI_RPM_RING_RED_AT ? t : (float)MULTI_RPM_RING_RED_AT;
        if (orange_end < (float)MULTI_RPM_RING_ORANGE_AT) orange_end = (float)MULTI_RPM_RING_ORANGE_AT;
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_orange, MULTI_RPM_RING_ORANGE_AT);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_orange, (int32_t)orange_end);

        float red_end = t > 8000.0f ? 8000.0f : t;
        if (red_end < (float)MULTI_RPM_RING_RED_AT) red_end = (float)MULTI_RPM_RING_RED_AT;
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_red, MULTI_RPM_RING_RED_AT);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_red, (int32_t)red_end);
    } else {
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_yellow, MULTI_WATER_SCALE_MIN);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_yellow, MULTI_WATER_SCALE_MIN);
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_orange, MULTI_WATER_RING_ORANGE_AT);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_orange, MULTI_WATER_RING_ORANGE_AT);
        lv_meter_set_indicator_start_value(multi_meter, multi_ring_red, MULTI_WATER_RING_RED_AT);
        lv_meter_set_indicator_end_value(multi_meter, multi_ring_red, MULTI_WATER_RING_RED_AT);
    }

    // Streifen-2/MCP2515-Konflikt kann sich aendern, waehrend der LED-Screen
    // schon offen ist (Datenquelle wird auf scr_service umgeschaltet) -
    // deshalb hier periodisch statt nur beim Tab-Wechsel aktualisiert.
    update_led_conflict_warning();
}
