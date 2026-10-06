#include "bmw_ui.h"
#include "multi_bg_img.h"
#include "needle_imgs.h"
#include "field_icons.h"
#include "can_obd2.h"
#include "ble_obd.h"
#include "sntp_sync.h"
#include "service_funcs.h"
#include "PCF85063.h"
#include "sd_log.h"
#include "SD_MMC.h"
#include "Buzzer.h"
#include <string.h>
#include <stdio.h>
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

static lv_obj_t *multi_meter;
static lv_meter_scale_t *multi_needle_scale;
static lv_meter_scale_t *multi_ring_scale;
static lv_meter_indicator_t *multi_needle;
static lv_meter_indicator_t *multi_ring_yellow;
static lv_meter_indicator_t *multi_ring_orange;
static lv_meter_indicator_t *multi_ring_red;

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
static lv_obj_t *multi_speed_label;
static lv_obj_t *multi_bat_label;
static lv_obj_t *multi_throttle_label;
static lv_obj_t *multi_rpm_label;
static lv_obj_t *multi_water_label;
// Schwarze Umrandung (8 versetzte Kopien) hinter jedem der 4 Zusatzfelder,
// damit die weisse Schrift auch auf hellem Hintergrund gut lesbar bleibt
static lv_obj_t *multi_bat_outline[8];
static lv_obj_t *multi_throttle_outline[8];
static lv_obj_t *multi_rpm_outline[8];
static lv_obj_t *multi_water_outline[8];

static lv_obj_t *multi_rx_dot;
static lv_obj_t *multi_tx_dot;
#define ACT_DOT_FLASH_MS 120

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
// Eigene, unsichtbare Hilfsskala mit um -90 versetzter rotation (270 range,
// 45 rotation), damit die 6-Uhr-Ruhestellung mit der Skalenluecke uebereinstimmt
// (identische Logik wie im C6-Projekt, addImageNeedle()).
static lv_meter_indicator_t *add_image_needle(lv_obj_t *meter, int32_t min_val, int32_t max_val,
                                              const lv_img_dsc_t *img, lv_coord_t pivot_x, lv_coord_t pivot_y,
                                              lv_meter_scale_t **out_scale)
{
    lv_meter_scale_t *needle_scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, needle_scale, min_val, max_val, 270, 45);
    if (out_scale) *out_scale = needle_scale;
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

// --- Einstellungen dauerhaft speichern (NVS-Flash, ueberlebt Stromverluste,
// unabhaengig von der SD-Karte): Primaer-/Sekundaerfarbe, Datenquelle (BLE/
// MCP), Datenlogging an/aus, Anzeige-Umschalter Wasser/Drehzahl, Warnsummer
// an/aus. nvs_flash_init() laeuft bereits in Wireless_Init() (main.c), lange
// bevor BMW_UI_Init() aufgerufen wird. ---
#define BMW_SETTINGS_NVS_NS "bmw_set"

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
    nvs_set_u8(h, "buzzer_en",  g_buzzer_enabled ? 1 : 0);
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
    if (nvs_get_u8(h, "buzzer_en", &v) == ESP_OK) {
        g_buzzer_enabled = (v != 0);
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

static void start_datalogging(void)
{
    // Karte war beim Boot evtl. nicht eingelegt/nicht gemountet (SDCard_Size
    // blieb 0) - z.B. weil sie erst nach dem Einschalten eingesteckt wurde.
    // Hier erneut versuchen, statt direkt "SD-Fehler!" zu melden, obwohl die
    // Karte jetzt sichtbar im Schacht steckt.
    if (!SD_EnsureMounted()) {
        lv_label_set_text(log_status_label, "SD-Fehler!");
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
    // lock_init_generic -> abort). Ob das an knappem/fragmentiertem Heap
    // liegt, zeigt sich erst mit echten Zahlen aus dem Feld statt zu raten.
    SD_Log("Datenlogging: oeffne %s (freier Heap=%u, groesster 8-Bit-Block=%u)",
           path, (unsigned)esp_get_free_heap_size(),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    log_file = fopen(path, "w");
    if (!log_file) {
        lv_label_set_text(log_status_label, "SD-Fehler!");
        return;
    }
    fprintf(log_file,
            "Zeit_ms;Geschwindigkeit_kmh;Drehzahl_U_min;Wassertemperatur_C;"
            "Gaspedal_pct;Batterie_OBD2_V;GKraft_Quer_g;GKraft_Laengs_g\r\n");
    fflush(log_file);
    log_last_tick = lv_tick_get();
    lv_label_set_text(log_status_label, "Aktiv");

    // fopen() hat den abschuzgefaehrdeten Punkt diesmal ueberlebt - Bootloop-
    // Schutz-Zaehler fuer den naechsten Boot wieder auf 0 setzen.
    clear_datalog_boot_counter();
}

static void stop_datalogging(void)
{
    if (log_file) {
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

    lv_obj_t *log_switch = lv_switch_create(scr_settings_func);
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
        lv_scr_load_anim(scr_sensors, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    } else if (scr == scr_sensors && dir == LV_DIR_LEFT) {
        lv_scr_load_anim(scr_service, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    } else if (scr == scr_settings && dir == LV_DIR_RIGHT) {
        lv_scr_load_anim(scr_settings_func, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    } else if (scr == scr_settings_func && dir == LV_DIR_LEFT) {
        lv_scr_load_anim(scr_settings, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
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
// geht zurueck. Lambda/Ansaugdruck kommen live per Standard-OBD2-PID von der
// eingestellten Datenquelle (siehe BMW_UI_Update()). Ladedruck und
// Nockenwellen-Position zeigen immer "n/v": Fuer Ladedruck gibt es keine
// standardisierte Mode-01-PID (und der N43B20A hat laut CLAUDE.md ohnehin
// keinen Turbo), fuer die VANOS-Nockenwellenposition existiert ueberhaupt
// kein Standard-OBD2-PID - das waere nur per BMW-spezifischem UDS-Identifier
// auslesbar, was hier bewusst nicht blind nachgebaut wurde (vgl. die
// ungeklaerten NOx-Regeneration-/Bremsenentlueften-Service-Funktionen).
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

    // Werden nie per OBD2 beantwortet (siehe Kommentar oben) - einmalig fest
    // auf "n/v" setzen statt bei jedem BMW_UI_Update()-Tick neu zu schreiben.
    lv_label_set_text(sens_boost_val, "n/v");
    lv_label_set_text(sens_cam_in_val, "n/v");
    lv_label_set_text(sens_cam_ex_val, "n/v");

    lv_obj_t *hint = lv_label_create(scr_sensors);
    lv_label_set_text(hint, "<- Wisch: Service");
    lv_obj_set_style_text_color(hint, lv_color_make(150, 150, 150), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);

    lv_obj_add_event_cb(scr_sensors, swipe_gesture_cb, LV_EVENT_GESTURE, NULL);
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
        lv_obj_t *icon = lv_img_create(scr_multi);
        lv_img_set_src(icon, field_icons[i]);
        lv_obj_align(icon, LV_ALIGN_CENTER, field_x[i], 32);
        lv_obj_clear_flag(icon, LV_OBJ_FLAG_CLICKABLE);
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

    // Unsichtbares Center-Overlay: 3-Sekunden-Halten -> Demo-Seite,
    // Doppeltipp -> Einstellungs-Screen (Farben). Vergroessert (160 -> 260),
    // da der Treffbereich dem Nutzer zu klein war.
    lv_obj_t *center_hold = lv_obj_create(scr_multi);
    lv_obj_set_size(center_hold, 260, 260);
    lv_obj_center(center_hold);
    lv_obj_set_style_bg_opa(center_hold, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(center_hold, 0, 0);
    lv_obj_clear_flag(center_hold, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(center_hold, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(center_hold, center_touch_cb, LV_EVENT_ALL, NULL);

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

    create_settings_screen();
    create_settings_func_screen();
    create_dtc_screen();
    create_service_screen();
    create_sensors_screen();

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

    // Sensoren-Screen: Lambda/Ansaugdruck leben, solange die Quelle online
    // ist, sonst bleiben die Felder auf "---" stehen. Ladedruck/Nockenwellen-
    // Position sind bereits einmalig in create_sensors_screen() auf "n/v"
    // gesetzt und werden hier nicht mehr angefasst (sie aendern sich nie).
    {
        bool sensors_online = use_ble_src ? BLE_OBD_online() : CAN_OBD2_online();
        char sbuf[16];
        if (sensors_online) {
            float l1r = use_ble_src ? BLE_OBD_lambda1_ratio()   : CAN_OBD2_lambda1_ratio();
            float l1v = use_ble_src ? BLE_OBD_lambda1_voltage() : CAN_OBD2_lambda1_voltage();
            float l2r = use_ble_src ? BLE_OBD_lambda2_ratio()   : CAN_OBD2_lambda2_ratio();
            float l2v = use_ble_src ? BLE_OBD_lambda2_voltage() : CAN_OBD2_lambda2_voltage();
            float map_kpa = use_ble_src ? BLE_OBD_intake_pressure() : CAN_OBD2_intake_pressure();

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
        } else {
            lv_label_set_text(sens_lambda1_val, "---");
            lv_label_set_text(sens_lambda2_val, "---");
            lv_label_set_text(sens_lambda1_volt_val, "---");
            lv_label_set_text(sens_lambda2_volt_val, "---");
            lv_label_set_text(sens_map_val, "---");
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
}
