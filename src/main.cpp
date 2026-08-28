#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TJpg_Decoder.h>
#include <lvgl.h>
#include <driver/twai.h>
#include <Arduino_GFX_Library.h>

// --- HARDWARE & DISPLAY CONFIGURATION (Waveshare 1.43" AMOLED ESP32-C6) ---
#define SD_CS_PIN       13
#define TWAI_TX_PIN     GPIO_NUM_19
#define TWAI_RX_PIN     GPIO_NUM_20

#define DISPLAY_WIDTH   466
#define DISPLAY_HEIGHT  466

// Display-Bus setup für CO5300 QSPI
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    6 /* CS */, 10 /* SCK */, 0 /* D0 */, 1 /* D1 */, 2 /* D2 */, 3 /* D3 */
);
Arduino_GFX *gfx = new Arduino_CO5300(bus, 7 /* RST */, 0 /* Rotation */, true /* IPS */, DISPLAY_WIDTH, DISPLAY_HEIGHT);

// --- LVGL BUFFER (Partieller Line-Buffer im SRAM) ---
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[DISPLAY_WIDTH * 30];

// --- EINSTELLUNGEN & KONFIGURATION ---
struct Settings {
    float warn_temp = 105.0f;
    float max_temp  = 120.0f;
    float warn_bat  = 11.5f;
    float min_bat   = 10.5f;
    lv_color_t color_primary   = LV_COLOR_MAKE(0, 162, 255);   // Cyan/Blau
    lv_color_t color_secondary = LV_COLOR_MAKE(255, 0, 0);     // Rot (Nadel)
    char brand[16]             = "bmw";
} currentSettings;

// --- UI OBJEKTE ---
lv_obj_t *main_screen;
lv_obj_t *settings_screen;
lv_obj_t *dtc_screen; // Fehlercode-Übersicht (per Doppeltipp erreichbar)
lv_obj_t *tileview;

// Tiles
lv_obj_t *tile_water;
lv_obj_t *tile_bat;

// Gauges
lv_obj_t *water_meter;
lv_meter_indicator_t *water_needle;
lv_obj_t *water_label;

lv_obj_t *bat_meter;
lv_meter_indicator_t *bat_needle;
lv_obj_t *bat_label;

// Diagnose UI Elemente
lv_obj_t *dtc_status_label;
lv_obj_t *cbs_status_label;

// Dynamische Live-Werte
float current_water_temp = 90.0f;
float current_bat_voltage = 12.4f;
char last_dtc_text[64] = "Keine Fehler im Speicher";
char last_cbs_text[64] = "CBS Status: OK";

// Touch Handling (3s Long Press)
unsigned long touch_start = 0;
bool is_touching_center = false;

// Touch Handling (Doppeltipp -> Fehlercode-Übersicht)
unsigned long last_tap_time = 0;
bool was_pressed = false;

// --- TJPGDEC DRAW CALLBACK ---
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
    gfx->draw16bitRGBBitmap(x, y, bitmap, w, h);
    return true;
}

// --- COLOR INTERPOLATION (SMOOTH FADING) ---
lv_color_t mix_colors(lv_color_t c1, lv_color_t c2, float ratio) {
    if (ratio <= 0.0f) return c1;
    if (ratio >= 1.0f) return c2;
    uint8_t r = LV_COLOR_GET_R(c1) + (int)((LV_COLOR_GET_R(c2) - LV_COLOR_GET_R(c1)) * ratio);
    uint8_t g = LV_COLOR_GET_G(c1) + (int)((LV_COLOR_GET_G(c2) - LV_COLOR_GET_G(c1)) * ratio);
    uint8_t b = LV_COLOR_GET_B(c1) + (int)((LV_COLOR_GET_B(c2) - LV_COLOR_GET_B(c1)) * ratio);
    return lv_color_make(r, g, b);
}

// --- LVGL FLUSH DISPLAY ---
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    lv_disp_flush_ready(disp);
}

// --- BOOT-ANIMATION (15 FPS JPEGs VON SD) ---
void playBootAnimation(const char* brand) {
    TJpgDec.setJpgScale(1);
    TJpgDec.setCallback(tft_output);

    char path[64];
    const int totalFrames = 150;
    const int frameDelayMs = 66; // 15 FPS

    for (int i = 0; i < totalFrames; i++) {
        unsigned long start = millis();
        snprintf(path, sizeof(path), "/anim_%s/frame_%03d.jpg", brand, i);

        if (SD.exists(path)) {
            TJpgDec.drawSdJpg(0, 0, path);
        } else {
            break;
        }

        int elapsed = millis() - start;
        if (elapsed < frameDelayMs) delay(frameDelayMs - elapsed);
    }
}

// --- CAN BUS (TWAI) & DIAGNOSE FUNKTIONEN ---
void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // BMW E90 500kbit/s D-CAN
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
    }
}

// Hilfsfunktion zum Senden von CAN-Frames
bool sendCANFrame(uint32_t id, uint8_t* data, uint8_t len) {
    twai_message_t message;
    message.identifier = id;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.data_length_code = len;
    for (int i = 0; i < len; i++) {
        message.data[i] = data[i];
    }
    return (twai_transmit(&message, pdMS_TO_TICKS(50)) == ESP_OK);
}

// Fehlercodes (DTCs) über Standard OBD2 Mode 03 auslesen
void readDiagnosticTroubleCodes() {
    uint8_t req[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (sendCANFrame(0x7DF, req, 8)) {
        snprintf(last_dtc_text, sizeof(last_dtc_text), "DTC Request gesendet...");
    } else {
        snprintf(last_dtc_text, sizeof(last_dtc_text), "CAN Fehler beim Lesen!");
    }
    lv_label_set_text(dtc_status_label, last_dtc_text);
}

// Fehlercodes löschen (OBD2 Mode 04)
void clearDiagnosticTroubleCodes() {
    uint8_t req[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (sendCANFrame(0x7DF, req, 8)) {
        snprintf(last_dtc_text, sizeof(last_dtc_text), "Fehler erfolgreich geloescht!");
    } else {
        snprintf(last_dtc_text, sizeof(last_dtc_text), "Fehler beim Loeschen!");
    }
    lv_label_set_text(dtc_status_label, last_dtc_text);
}

// BMW Condition Based Service (CBS) Intervall Reset Routine
void resetServiceInterval(uint8_t serviceType) {
    // serviceType: 0x01 = Motoroel, 0x02 = Bremsflüssigkeit, 0x03 = Fahrzeug-Check
    // Routine Control (Service 31) an das Kombiinstrument (Target ID 0x6F1 / 0x611)
    uint8_t req[8] = {0x04, 0x31, 0x01, 0xFF, serviceType, 0x00, 0x00, 0x00};
    if (sendCANFrame(0x611, req, 8)) {
        snprintf(last_cbs_text, sizeof(last_cbs_text), "CBS Reset (Typ %d) gesendet!", serviceType);
    } else {
        snprintf(last_cbs_text, sizeof(last_cbs_text), "CBS Reset fehlgeschlagen!");
    }
    lv_label_set_text(cbs_status_label, last_cbs_text);
}

void processCAN() {
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
        // E90 Kühlmittel PIDs abgreifen (Beispiel ID 0x1D0)
        if (message.identifier == 0x1D0) {
            current_water_temp = message.data[0] - 40;
        }
    }
}

// --- GAUGE UPDATE & COLOR FADE / BLINK ENGINE ---
void updateGaugeUI() {
    static bool blink_state = false;
    static uint32_t last_blink = 0;
    if (millis() - last_blink > 250) {
        blink_state = !blink_state;
        last_blink = millis();
    }

    // 1. WASSERTEMPERATUR
    lv_meter_set_indicator_value(water_meter, water_needle, current_water_temp);
    float pre_warn = currentSettings.warn_temp - (currentSettings.warn_temp * 0.10f); // 10% vor Warnbereich
    lv_color_t water_color;

    if (current_water_temp < pre_warn) {
        water_color = currentSettings.color_primary;
    } else if (current_water_temp >= pre_warn && current_water_temp < currentSettings.warn_temp) {
        float ratio = (current_water_temp - pre_warn) / (currentSettings.warn_temp - pre_warn);
        water_color = mix_colors(currentSettings.color_primary, lv_palette_main(LV_PALETTE_YELLOW), ratio);
    } else if (current_water_temp >= currentSettings.warn_temp && current_water_temp < currentSettings.max_temp) {
        float ratio = (current_water_temp - currentSettings.warn_temp) / (currentSettings.max_temp - currentSettings.warn_temp);
        water_color = mix_colors(lv_palette_main(LV_PALETTE_YELLOW), lv_palette_main(LV_PALETTE_RED), ratio);
    } else {
        water_color = blink_state ? lv_palette_main(LV_PALETTE_RED) : lv_color_black();
    }

    lv_label_set_text_fmt(water_label, "%d °C", (int)current_water_temp);
    lv_obj_set_style_text_color(water_label, water_color, 0);

    // 2. BATTERIESPANNUNG
    lv_meter_set_indicator_value(bat_meter, bat_needle, current_bat_voltage * 10);
    lv_color_t bat_color = (current_bat_voltage <= currentSettings.warn_bat) ?
                           (blink_state ? lv_palette_main(LV_PALETTE_RED) : currentSettings.color_primary) :
                           currentSettings.color_primary;

    lv_label_set_text_fmt(bat_label, "%.1f V", current_bat_voltage);
    lv_obj_set_style_text_color(bat_label, bat_color, 0);
}

// --- EINSTELLUNGS-SCREEN ---
void createSettingsUI() {
    settings_screen = lv_obj_create(NULL);

    lv_obj_t *title = lv_label_create(settings_screen);
    lv_label_set_text(title, "EINSTELLUNGEN");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t *btn_save = lv_btn_create(settings_screen);
    lv_obj_align(btn_save, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_t *lbl = lv_label_create(btn_save);
    lv_label_set_text(lbl, "Speichern & Beenden");

    lv_obj_add_event_cb(btn_save, [](lv_event_t * e) {
        lv_scr_load_anim(main_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
    }, LV_EVENT_CLICKED, NULL);
}

// --- HAUPT GAUGE UI (TILEVIEW: WASSER / BATTERIE) ---
void createGaugesUI() {
    main_screen = lv_obj_create(NULL);

    tileview = lv_tileview_create(main_screen);
    tile_water = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_RIGHT);
    tile_bat   = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_LEFT);

    // TILE 1: WASSERTEMP
    water_meter = lv_meter_create(tile_water);
    lv_obj_center(water_meter);
    lv_obj_set_size(water_meter, 420, 420);
    lv_meter_scale_t *scale_w = lv_meter_add_scale(water_meter);
    lv_meter_set_scale_range(water_meter, scale_w, 40, 130, 240, 150);
    water_needle = lv_meter_add_needle_line(water_meter, scale_w, 5, currentSettings.color_secondary, -10);
    water_label = lv_label_create(tile_water);
    lv_obj_align(water_label, LV_ALIGN_CENTER, 0, 90);

    // TILE 2: BATTERIE
    bat_meter = lv_meter_create(tile_bat);
    lv_obj_center(bat_meter);
    lv_obj_set_size(bat_meter, 420, 420);
    lv_meter_scale_t *scale_b = lv_meter_add_scale(bat_meter);
    lv_meter_set_scale_range(bat_meter, scale_b, 100, 160, 240, 150);
    bat_needle = lv_meter_add_needle_line(bat_meter, scale_b, 5, currentSettings.color_secondary, -10);
    bat_label = lv_label_create(tile_bat);
    lv_obj_align(bat_label, LV_ALIGN_CENTER, 0, 90);

    lv_scr_load(main_screen);
}

// --- FEHLERCODE-ÜBERSICHT (per Doppeltipp erreichbar) ---
void createDtcUI() {
    dtc_screen = lv_obj_create(NULL);

    lv_obj_t *diag_title = lv_label_create(dtc_screen);
    lv_label_set_text(diag_title, "FEHLERCODE-UEBERSICHT");
    lv_obj_align(diag_title, LV_ALIGN_TOP_MID, 0, 30);

    dtc_status_label = lv_label_create(dtc_screen);
    lv_label_set_text(dtc_status_label, "Status: Bereit");
    lv_obj_align(dtc_status_label, LV_ALIGN_CENTER, 0, -60);

    // CBS Öl-Reset Button
    lv_obj_t *btn_cbs_oil = lv_btn_create(dtc_screen);
    lv_obj_set_size(btn_cbs_oil, 180, 40);
    lv_obj_align(btn_cbs_oil, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *lbl_oil = lv_label_create(btn_cbs_oil);
    lv_label_set_text(lbl_oil, "Oel-Service Reset");
    lv_obj_center(lbl_oil);
    lv_obj_add_event_cb(btn_cbs_oil, [](lv_event_t *e) { resetServiceInterval(0x01); }, LV_EVENT_CLICKED, NULL);

    cbs_status_label = lv_label_create(dtc_screen);
    lv_label_set_text(cbs_status_label, "CBS: OK");
    lv_obj_align(cbs_status_label, LV_ALIGN_CENTER, 0, 50);

    // Löschen-Button (unten links)
    lv_obj_t *btn_clr_dtc = lv_btn_create(dtc_screen);
    lv_obj_set_size(btn_clr_dtc, 140, 45);
    lv_obj_align(btn_clr_dtc, LV_ALIGN_BOTTOM_MID, -80, -30);
    lv_obj_t *lbl_cl = lv_label_create(btn_clr_dtc);
    lv_label_set_text(lbl_cl, "Loeschen");
    lv_obj_center(lbl_cl);
    lv_obj_add_event_cb(btn_clr_dtc, [](lv_event_t *e) { clearDiagnosticTroubleCodes(); }, LV_EVENT_CLICKED, NULL);

    // Zurück-Button (unten rechts)
    lv_obj_t *btn_back = lv_btn_create(dtc_screen);
    lv_obj_set_size(btn_back, 140, 45);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 80, -30);
    lv_obj_t *lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Zurueck");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        lv_scr_load_anim(main_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
    }, LV_EVENT_CLICKED, NULL);
}

// --- 3 SEKUNDEN CENTER TOUCH PRÜFUNG ---
void checkCenterTouch() {
    lv_indev_t *indev = lv_indev_get_next(NULL);
    if (!indev) return;

    lv_indev_data_t data;
    lv_indev_get_point(indev, &data.point);

    bool center = (data.point.x >= 183 && data.point.x <= 283 && data.point.y >= 183 && data.point.y <= 283);

    if (indev->proc.state == LV_INDEV_STATE_PR) {
        if (!is_touching_center && center) {
            touch_start = millis();
            is_touching_center = true;
        } else if (is_touching_center && !center) {
            is_touching_center = false;
        }

        if (is_touching_center && (millis() - touch_start >= 3000)) {
            is_touching_center = false;
            lv_scr_load_anim(settings_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
        }
    } else {
        is_touching_center = false;
    }
}

// --- DOPPELTIPP PRÜFUNG (öffnet Fehlercode-Übersicht) ---
void checkDoubleTap() {
    lv_indev_t *indev = lv_indev_get_next(NULL);
    if (!indev) return;

    bool pressed = (indev->proc.state == LV_INDEV_STATE_PR);

    if (pressed && !was_pressed) {
        unsigned long now = millis();
        if (lv_scr_act() == main_screen) {
            if (now - last_tap_time < 400) {
                last_tap_time = 0;
                readDiagnosticTroubleCodes();
                lv_scr_load_anim(dtc_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
            } else {
                last_tap_time = now;
            }
        }
    }
    was_pressed = pressed;
}

// --- SETUP & MAIN LOOP ---
void setup() {
    Serial.begin(115200);

    gfx->begin();
    gfx->fillScreen(BLACK);

    SPI.begin();
    SD.begin(SD_CS_PIN);

    // 1. Boot-Animation abspielen
    playBootAnimation(currentSettings.brand);

    // 2. LVGL initialisieren
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, DISPLAY_WIDTH * 30);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISPLAY_WIDTH;
    disp_drv.ver_res = DISPLAY_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // 3. UI & CAN aufbauen
    createSettingsUI();
    createGaugesUI();
    createDtcUI();
    initCAN();
}

void loop() {
    lv_timer_handler();
    processCAN();
    updateGaugeUI();
    checkCenterTouch();
    checkDoubleTap();
    delay(5);
}
