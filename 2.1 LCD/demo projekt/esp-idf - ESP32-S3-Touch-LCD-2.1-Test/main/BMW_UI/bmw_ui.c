#include "bmw_ui.h"
#include "multi_bg_img.h"
#include "needle_imgs.h"
#include "BAT_Driver.h"
#include "can_obd2.h"
#include <string.h>

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

// Primaerfarbe (Geschwindigkeitsanzeige) / Sekundaerfarbe (Nadel-Basisfarbe,
// solange die Wassertemperatur unter 95 Grad liegt) - per Einstellungs-Screen
// veraenderbar, Standardwerte wie im C6-Projekt.
static lv_color_t g_color_primary   = LV_COLOR_MAKE(0, 162, 255);  // Blau
static lv_color_t g_color_secondary = LV_COLOR_MAKE(255, 0, 0);    // Rot

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

static uint32_t anim_start_tick;
static bool anim_done = false;

// --- UI-Objekte ---
static lv_obj_t *scr_multi;
static lv_obj_t *scr_demo;
static lv_obj_t *scr_settings;

static lv_obj_t *multi_meter;
static lv_meter_indicator_t *multi_needle;
static lv_obj_t *multi_speed_label;
static lv_obj_t *multi_bat_label;
static lv_obj_t *multi_throttle_label;
static lv_obj_t *multi_rpm_label;
static lv_obj_t *multi_water_label;

// --- Schaltanzeige (6 Fuell-Kaestchen ueber den im Hintergrundbild
// gezeichneten Kaesten): fuellen sich mit steigender Drehzahl (je Kaestchen
// eine eigene Schwelle), ab RPM_SHIFT_BLINK blinken alle gemeinsam wie eine
// digitale Schaltanzeige. Fuellfarben = Umrandungsfarben aus dem
// Hintergrundbild (1-4 weiss/grau, 5 hell-lila, 6 blau). ---
#define RPM_SHIFT_BLINK 6800
static lv_obj_t *rpm_boxes[6];
static const lv_coord_t rpm_box_x[6]  = {-117, -73, -28, 17, 62, 109};
static const int32_t    rpm_box_thr[6] = {4000, 4560, 5120, 5680, 6240, 6800};
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

// Bild-Nadel um den Meter-Mittelpunkt rotieren (in Ruhestellung nach 6 Uhr).
// Eigene, unsichtbare Hilfsskala mit um -90 versetzter rotation (270 range,
// 45 rotation), damit die 6-Uhr-Ruhestellung mit der Skalenluecke uebereinstimmt
// (identische Logik wie im C6-Projekt, addImageNeedle()).
static lv_meter_indicator_t *add_image_needle(lv_obj_t *meter, int32_t min_val, int32_t max_val,
                                              const lv_img_dsc_t *img, lv_coord_t pivot_x, lv_coord_t pivot_y)
{
    lv_meter_scale_t *needle_scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, needle_scale, min_val, max_val, 270, 45);
    return lv_meter_add_needle_img(meter, needle_scale, img, pivot_x, pivot_y);
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

// 3-Sekunden-Halten in der Bildschirmmitte -> Demo-Seite anzeigen,
// Doppeltipp in der Mitte -> Einstellungs-Screen (Farben) anzeigen.
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

// Einstellungs-Screen mit Primaer-/Sekundaerfarb-Auswahl, erreichbar per
// Doppeltipp in der Bildschirmmitte der Multi-Ansicht.
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

    lv_obj_t *btn_back = lv_btn_create(scr_settings);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_back, back_from_settings_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, "Zurueck");
    lv_obj_center(back_lbl);
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
    scr_demo = demo_screen;

    scr_multi = lv_obj_create(NULL);
    set_dark_bg(scr_multi);
    lv_obj_clear_flag(scr_multi, LV_OBJ_FLAG_SCROLLABLE);

    // Statisches Hintergrundbild (RPM-Ring, Skala, Farbverlauf) - fest im Bild
    lv_obj_t *bg_img = lv_img_create(scr_multi);
    lv_img_set_src(bg_img, &multi_bg_img);
    lv_obj_center(bg_img);

    // Transparentes Meter nur fuer die Nadel-Winkelberechnung
    multi_meter = lv_meter_create(scr_multi);
    lv_obj_set_style_bg_opa(multi_meter, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(multi_meter, 0, 0);
    lv_obj_center(multi_meter);
    lv_obj_set_size(multi_meter, 450, 450);
    lv_obj_clear_flag(multi_meter, LV_OBJ_FLAG_SCROLLABLE);

    // Multi-Nadel in der eingestellten Sekundaerfarbe (Standard: Rot) + weisses Hub-Cap
    const lv_img_dsc_t *init_needle_img;
    lv_coord_t init_pivot_x, init_pivot_y;
    get_base_needle_img(&init_needle_img, &init_pivot_x, &init_pivot_y);
    multi_needle = add_image_needle(multi_meter, 0, 8000, init_needle_img, init_pivot_x, init_pivot_y);

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
    lv_obj_align(multi_speed_label, LV_ALIGN_CENTER, 0, -60);
    lv_label_set_text(multi_speed_label, "0");

    lv_obj_t *unit_label = lv_label_create(scr_multi);
    lv_label_set_text(unit_label, "KM/H");
    lv_obj_set_style_text_color(unit_label, lv_color_white(), 0);
    lv_obj_align(unit_label, LV_ALIGN_CENTER, 0, -15);

    // 4 Zusatzfelder (Batterie/Gaspedal/Drehzahl/Wasser)
    const int field_x[4] = {-165, -55, 55, 165};
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
    // Gaspedal- und Drehzahl-Feld doppelt so gross (Font 28 statt Standard 14)
    lv_obj_set_style_text_font(multi_throttle_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_font(multi_rpm_label, &lv_font_montserrat_28, 0);

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

    // Unsichtbares Center-Overlay: 3-Sekunden-Halten -> Demo-Seite,
    // Doppeltipp -> Einstellungs-Screen (Farben)
    lv_obj_t *center_hold = lv_obj_create(scr_multi);
    lv_obj_set_size(center_hold, 160, 160);
    lv_obj_center(center_hold);
    lv_obj_set_style_bg_opa(center_hold, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(center_hold, 0, 0);
    lv_obj_clear_flag(center_hold, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(center_hold, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(center_hold, center_touch_cb, LV_EVENT_ALL, NULL);

    // "Zurueck"-Button oben auf dem Demo-Screen (ueber dem Tabview, da als
    // letztes Kind von scr_demo erzeugt -> liegt im Z-Order oben)
    lv_obj_t *back_btn = lv_btn_create(scr_demo);
    lv_obj_set_size(back_btn, 110, 40);
    lv_obj_align(back_btn, LV_ALIGN_TOP_RIGHT, -10, 5);
    lv_obj_add_event_cb(back_btn, back_to_multi_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, "Zurueck");
    lv_obj_center(back_lbl);

    create_settings_screen();

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

    if (anim_done) {
        // Batteriespannung live vom Sensor
        current_bat_voltage = BAT_analogVolts;

        // Motordaten live vom MCP2515-CAN (falls online), sonst Platzhalter
        if (CAN_OBD2_online()) {
            current_speed_kmh    = CAN_OBD2_speed_kmh();
            current_rpm          = CAN_OBD2_rpm();
            current_water_temp   = CAN_OBD2_water_temp();
            current_throttle_pct = CAN_OBD2_throttle_pct();
        }
    }

    lv_meter_set_indicator_value(multi_meter, multi_needle, (int32_t)current_rpm);
    lv_label_set_text_fmt(multi_speed_label, "%d", (int)current_speed_kmh);
    lv_label_set_text_fmt(multi_bat_label, "%.1fV", current_bat_voltage);
    lv_label_set_text_fmt(multi_throttle_label, "%d%%", (int)current_throttle_pct);
    lv_label_set_text_fmt(multi_rpm_label, "%d", (int)current_rpm);
    lv_label_set_text_fmt(multi_water_label, "%d\xC2\xB0""C", (int)current_water_temp);
    lv_obj_set_style_text_color(multi_water_label,
        current_water_temp >= 110.0f ? lv_palette_main(LV_PALETTE_ORANGE) : lv_color_white(), 0);

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

    // Nadelfarbe: Basis = eingestellte Sekundaerfarbe (Standard Rot) bis
    // 95 Grad, ab 95 Grad neongelb, ab 106 Grad blinkt die Nadel (250ms-Takt,
    // wie das Schaltpunkt-Blinken der Drehzahl-LEDs im C6-Projekt)
    const lv_img_dsc_t *base_needle_img;
    lv_coord_t base_pivot_x, base_pivot_y;
    get_base_needle_img(&base_needle_img, &base_pivot_x, &base_pivot_y);
    LV_UNUSED(base_pivot_x);
    LV_UNUSED(base_pivot_y);
    const void *needle_src = base_needle_img;
    lv_opa_t needle_opa = LV_OPA_COVER;
    if (current_water_temp >= 95.0f) {
        needle_src = &multi_needle_yellow_img;
        if (current_water_temp >= 106.0f) {
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
}
