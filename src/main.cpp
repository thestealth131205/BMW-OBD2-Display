#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Preferences.h>
#include <TJpg_Decoder.h>
#include <lvgl.h>
#include <driver/twai.h>
#include <Arduino_GFX_Library.h>
#include <math.h>
#include "multi_bg_img.h"

// --- HARDWARE & DISPLAY CONFIGURATION (Waveshare 1.43" AMOLED ESP32-C6) ---
// Pinbelegung laut offiziellem Waveshare-SDK
// (waveshareteam/ESP32-C6-Touch-AMOLED-1.43, user_config.h) verifiziert.
// SD-Karte teilt sich den QSPI-Bus mit dem Display (SCK=11, MOSI=D0=4,
// MISO=D1=5), nur die CS-Leitung ist eigen (GPIO15) – laut offiziellem
// Waveshare-SDK (sdcard_bsp.h: cs = 15, host = display SPI2_HOST).
#define SD_CS_PIN       15
#define TWAI_TX_PIN     GPIO_NUM_19
#define TWAI_RX_PIN     GPIO_NUM_20

#define TOUCH_SCL_PIN   8
#define TOUCH_SDA_PIN   18
#define TOUCH_I2C_ADDR  0x38

// TCA9554 I2C-IO-Expander (Adresse 0x20). Auf der Waveshare-1.43"-Platine
// haengen an IO6 der Power-Hold-Latch (haelt das Board nach dem Einschalten
// unter Strom) und an IO7 eine weitere Enable-Leitung. Werden beide nach dem
// Boot nicht auf HIGH gezogen, schaltet sich das Board nach dem Einschalt-
// Impuls selbst wieder aus -> Bootloop. Das offizielle SDK setzt sie in
// Tca9554_Init(). Register: 0x01 = Output-Port, 0x03 = Konfiguration/Richtung.
#define IO_EXPANDER_ADDR 0x20

#define DISPLAY_WIDTH   466
#define DISPLAY_HEIGHT  466

// Display-Bus setup für SH8601 QSPI
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    10 /* CS */, 11 /* SCK */, 4 /* D0 */, 5 /* D1 */, 6 /* D2 */, 7 /* D3 */
);

// Eigene SH8601-Unterklasse: Kommandoreihenfolge 1:1 aus dem offiziellen
// Waveshare-Factory-Treiber uebernommen (esp_lcd_sh8601.c: panel_sh8601_init()
// + display_bsp.cpp: lcd_init_cmds[]). Zwei Abweichungen zur vorherigen
// eigenen Sequenz, die dort nachweislich zu gruen verfaelschten Schwarztoenen
// fuehrten:
// 1. MADCTL (0x36) und COLMOD (0x3A) werden im Werks-Treiber VOR SLPOUT
//    gesendet (direkt nach dem Reset), nicht erst danach.
// 2. Nach DISPON folgt als allerletzter Init-Schritt nochmal
//    WDBRIGHTNESSVALNOR (0x51) = 0xFF (volle Helligkeit), noch INNERHALB
//    derselben Init-Sequenz.
// NORON/INVOFF kommen im Werks-Init nicht vor und wurden entfernt.
// 3. col_offset1=6: Das offizielle Waveshare-Arduino-LVGL-Beispiel
//    (Example/Arduino/09_LVGL_V8_Test/lvgl_port.c, example_lvgl_flush_cb())
//    addiert beim Pixel-Schreiben "+ 0x06" auf x1/x2 (CASET-Fenster) - das
//    GRAM des SH8601 ist 480x480, das sichtbare 466x466-Fenster beginnt bei
//    Spalte 6. Fehlt dieser Versatz, landen alle Pixel 6 Spalten zu weit
//    links im GRAM.
// 4. Reset-Puls: Werks-Treiber (panel_sh8601_reset()) zieht RST OHNE
//    vorherigen HIGH-Puls direkt auf LOW (10ms), dann HIGH (150ms Settle).
//    Die generische GFX-Library macht vorher zusaetzlich 10ms HIGH und
//    nutzt 200ms/200ms statt 10ms/150ms - hier auf die Werks-Timings
//    angeglichen.
class Arduino_SH8601W : public Arduino_SH8601 {
public:
    Arduino_SH8601W(Arduino_DataBus *bus, int8_t rst, uint8_t r, int16_t w, int16_t h)
        : Arduino_SH8601(bus, rst, r, w, h, 6 /* col_offset1 */) {}

protected:
    void tftInit() override {
        if (_rst != GFX_NOT_DEFINED) {
            // Werks-Reset (esp_lcd_sh8601.c: panel_sh8601_reset()): kein
            // vorheriger HIGH-Puls, direkt LOW fuer 10ms, dann HIGH fuer
            // 150ms Settle-Zeit (nicht die generischen 200ms/200ms der
            // GFX-Library).
            pinMode(_rst, OUTPUT);
            digitalWrite(_rst, LOW);
            delay(10);
            digitalWrite(_rst, HIGH);
            delay(150);
        } else {
            _bus->sendCommand(SH8601_C_SWRESET);
            delay(SH8601_RST_DELAY);
        }

        _bus->beginWrite();
        _bus->writeC8D8(SH8601_W_MADCTL, 0x00); // RGB-Farbreihenfolge
        _bus->writeC8D8(SH8601_W_PIXFMT, 0x55);  // COLMOD: 16 bit/pixel RGB565
        _bus->endWrite();

        _bus->beginWrite();
        _bus->writeCommand(SH8601_C_SLPOUT);
        _bus->endWrite();
        delay(80);

        _bus->beginWrite();
        _bus->writeC8D8(SH8601_W_SPIMODECTL, 0x80);
        _bus->writeC8D8(SH8601_W_WCTRLD1, 0x20); // Brightness Control On
        _bus->endWrite();
        delay(1);

        _bus->beginWrite();
        _bus->writeC8D8(SH8601_W_WDBRIGHTNESSVALHBM, 0xFF);
        _bus->endWrite();
        delay(1);

        _bus->beginWrite();
        _bus->writeC8D8(SH8601_W_WDBRIGHTNESSVALNOR, 0x00);
        _bus->endWrite();
        delay(1);

        _bus->beginWrite();
        _bus->writeCommand(SH8601_C_DISPON);
        _bus->endWrite();
        delay(10);

        _bus->beginWrite();
        _bus->writeC8D8(SH8601_W_WDBRIGHTNESSVALNOR, 0xFF);
        _bus->endWrite();
    }
};

Arduino_SH8601W *gfx = new Arduino_SH8601W(bus, 3 /* RST */, 0 /* Rotation */, DISPLAY_WIDTH, DISPLAY_HEIGHT);

// --- LVGL BUFFER (Partieller Line-Buffer im SRAM) ---
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[DISPLAY_WIDTH * 30];

// --- EINSTELLUNGEN & KONFIGURATION ---
struct Settings {
    float warn_temp = 105.0f;
    float max_temp  = 120.0f;
    float warn_bat  = 11.5f;
    float min_bat   = 10.5f;
    float throttle_warn_pct = 70.0f;
    float throttle_max_pct  = 90.0f;
    float rpm_warn  = 5000.0f;   // Schwelle 1: Beginn der gelben Schaltpunkt-LEDs
    float rpm_limit = 7000.0f;   // Schwelle 2: rote LED (Redline)
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
lv_obj_t *tile_multi;
lv_obj_t *tile_water;
lv_obj_t *tile_bat;
lv_obj_t *tile_throttle;
lv_obj_t *tile_rpm;
lv_obj_t *tile_gforce;

// Gauges
// Jede Nadel besteht aus 2 Indikatoren (breite kurze Basis + duenne lange
// Spitze), die zusammen einen tacho-typischen, spitz zulaufenden
// "M3-Style"-Zeiger ergeben (siehe createStyledMeter()/createRpmTile()).
lv_obj_t *water_meter;
lv_meter_indicator_t *water_needle;
lv_meter_indicator_t *water_needle_tip;
lv_obj_t *water_label;
lv_obj_t *water_secondary_label;   // zeigt Batteriespannung

lv_obj_t *bat_meter;
lv_meter_indicator_t *bat_needle;
lv_meter_indicator_t *bat_needle_tip;
lv_obj_t *bat_label;
lv_obj_t *bat_secondary_label;     // zeigt Kühlmitteltemperatur

lv_obj_t *throttle_meter;
lv_meter_indicator_t *throttle_needle;
lv_meter_indicator_t *throttle_needle_tip;
lv_obj_t *throttle_label;
lv_obj_t *throttle_secondary_label; // zeigt Kühlmitteltemperatur

lv_obj_t *rpm_meter;
lv_meter_indicator_t *rpm_needle;
lv_meter_indicator_t *rpm_needle_tip;
lv_obj_t *rpm_leds[6]; // Schaltpunktanzeige statt digitaler Anzeige: 4x Gelb, 1x Gruen, 1x Rot

lv_obj_t *gforce_blob;         // Punkt, der Richtung/Betrag der G-Kraft anzeigt (Sekundärfarbe)
lv_obj_t *gforce_value_label;  // Betrag in G (z.B. "0.56G")
lv_obj_t *gforce_speed_label;  // Geschwindigkeit (z.B. "80 KM/H")

// Multi-Daten-Anzeige (erste Kachel): RPM-Ring-Skala im Hintergrund (Nadel
// als weißer Strich mit dunkel eingefärbtem "Schweif"), mittig groß die
// Geschwindigkeit, darunter 4 Zusatzfelder (Batterie/Gaspedal/Drehzahl/Wasser).
lv_obj_t *multi_meter;
lv_meter_indicator_t *multi_needle_trail;
lv_meter_indicator_t *multi_needle_tip;
lv_obj_t *multi_speed_label;
lv_obj_t *multi_bat_label;
lv_obj_t *multi_throttle_label;
lv_obj_t *multi_rpm_label;
lv_obj_t *multi_water_label;   // einziges farbig reagierendes Feld: ab 110°C orange

// Diagnose UI Elemente
lv_obj_t *dtc_status_label;
lv_obj_t *cbs_status_label;

// Dynamische Live-Werte - Startwerte werden angezeigt, solange keine OBD2/CAN-Verbindung
// besteht (kein Blockieren beim Booten ohne CAN-Daten, siehe initCAN()/processCAN())
float current_water_temp = 105.0f;
float current_bat_voltage = 12.6f;
float current_throttle_position = 0.0f;
float current_rpm = 800.0f; // Leerlaufdrehzahl als Platzhalter-Startwert
float current_gforce_x = 0.0f;   // seitliche Beschleunigung (links/rechts), in g
float current_gforce_y = 0.0f;   // Längsbeschleunigung (Bremsen/Beschleunigen), in g
float current_speed_kmh = 0.0f;  // Fahrzeuggeschwindigkeit
bool gforce_calibrated = false;  // erster CAN-Frame nach Boot wird als 0-Referenz übernommen
float gforce_offset_x = 0.0f;
float gforce_offset_y = 0.0f;
char last_dtc_text[64] = "Keine Fehler im Speicher";
char last_cbs_text[64] = "CBS Status: OK";

// Touch Handling (3s Long Press)
unsigned long touch_start = 0;
bool is_touching_center = false;

// Touch Handling (Doppeltipp -> Fehlercode-Übersicht)
unsigned long last_tap_time = 0;
bool was_pressed = false;

// --- STROMLOS GESPEICHERTE START-ANZEIGE (NVS/Preferences) ---
// Index der Kachel, die nach dem Booten angezeigt wird: 0=Multi, 1=Wasser,
// 2=Batterie, 3=Gaspedal, 4=Drehzahl, 5=G-Force. Default bleibt Wasser (1),
// damit ohne OBD2-Verbindung weiterhin der Wasser-Screen mit den Startwerten
// (105°C/12,6V) angezeigt wird.
Preferences prefs;
uint8_t startup_gauge = 1;

void loadStartupGauge() {
    prefs.begin("bmw_disp", true);
    startup_gauge = prefs.getUChar("startup_gauge", 1);
    prefs.end();
}

void saveStartupGauge(uint8_t idx) {
    startup_gauge = idx;
    prefs.begin("bmw_disp", false);
    prefs.putUChar("startup_gauge", idx);
    prefs.end();
}

// --- TCA9554 IO-EXPANDER: POWER-HOLD (IO6) + IO7 + LCD_EN (IO2) AUF HIGH ---
// Muss VOR der Display-Init laufen, damit das Board unter Strom bleibt.
void initIoExpander() {
    // Output-Latch zuerst setzen (IO2+IO6+IO7 = HIGH), dann Richtung auf
    // Ausgang, um einen kurzen LOW-Glitch beim Umschalten zu vermeiden.
    // IO2 = LCD_EN (Display-Enable, laut offizieller Waveshare-Pin-Tabelle) –
    // ohne dieses Signal läuft das Panel in einem undefinierten
    // Power-Zustand (mögliche Ursache des hartnäckigen Grünstichs bei Schwarz).
    Wire.beginTransmission(IO_EXPANDER_ADDR);
    Wire.write(0x01);       // Output Port Register
    Wire.write(0xC4);       // Bit2 (LCD_EN) + Bit6 + Bit7 = HIGH
    Wire.endTransmission();

    Wire.beginTransmission(IO_EXPANDER_ADDR);
    Wire.write(0x03);       // Configuration Register (1 = Eingang, 0 = Ausgang)
    Wire.write(0x3B);       // IO2 + IO6 + IO7 als Ausgang, Rest Eingang
    Wire.endTransmission();
}

// --- I2C TOUCH TREIBER (FT-kompatibler Touch-Controller, Adresse 0x38) ---
// Registerprotokoll laut offiziellem Waveshare-SDK
// (display_bsp.cpp: Get_TouchCoords) verifiziert: Register 0x02 liefert
// 5 Bytes [Anzahl Touchpunkte, X_high(4bit)|X_low, Y_high(4bit)|Y_low].
bool readTouch(uint16_t &x, uint16_t &y) {
    uint8_t data[5];
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)TOUCH_I2C_ADDR, 5) != 5) return false;
    for (int i = 0; i < 5; i++) data[i] = Wire.read();
    if (data[0] != 1) return false;
    x = ((uint16_t)(data[1] & 0x0F) << 8) | data[2];
    y = ((uint16_t)(data[3] & 0x0F) << 8) | data[4];
    return true;
}

void touchpadReadCb(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t x, y;
    if (readTouch(x, y)) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

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

// Farbverlauf Primärfarbe -> Gelb/Orange (ab 10% vor der Warnschwelle) -> Rot
// (Warn- bis Alarmschwelle), oberhalb der Alarmschwelle blinkt der Wert rot/schwarz.
// invert=true: niedrige Werte sind kritisch (z.B. Batteriespannung) - der Verlauf
// läuft dann spiegelverkehrt (Rot am unteren Ende, Primärfarbe am oberen Ende).
lv_color_t computeGaugeColor(float value, float warn_val, float alert_val, bool invert, bool blink_state) {
    lv_color_t warn_color = lv_palette_main(LV_PALETTE_ORANGE);
    lv_color_t alert_color = lv_palette_main(LV_PALETTE_RED);

    if (!invert) {
        float pre_warn = warn_val - warn_val * 0.10f;
        if (value < pre_warn) {
            return currentSettings.color_primary;
        } else if (value < warn_val) {
            float ratio = (value - pre_warn) / (warn_val - pre_warn);
            return mix_colors(currentSettings.color_primary, warn_color, ratio);
        } else if (value < alert_val) {
            float ratio = (value - warn_val) / (alert_val - warn_val);
            return mix_colors(warn_color, alert_color, ratio);
        }
        return blink_state ? alert_color : lv_color_black();
    } else {
        float pre_warn = warn_val + warn_val * 0.10f;
        if (value > pre_warn) {
            return currentSettings.color_primary;
        } else if (value > warn_val) {
            float ratio = (pre_warn - value) / (pre_warn - warn_val);
            return mix_colors(currentSettings.color_primary, warn_color, ratio);
        } else if (value > alert_val) {
            float ratio = (warn_val - value) / (warn_val - alert_val);
            return mix_colors(warn_color, alert_color, ratio);
        }
        return blink_state ? alert_color : lv_color_black();
    }
}

// Farbe einer Skala-Position (nicht des Live-Werts) für Zonen-Bögen/Skala-Striche -
// oberhalb der Alarmschwelle immer durchgehend Rot (kein Blinken für statische Zonen).
lv_color_t computeZoneColor(float pos, float warn_val, float alert_val, bool invert) {
    return computeGaugeColor(pos, warn_val, alert_val, invert, true);
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

    char dirPath[48];
    snprintf(dirPath, sizeof(dirPath), "/anim_%s", brand);
    if (!SD.exists(dirPath)) {
        return; // Kein Animationsordner vorhanden -> ueberspringen
    }

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
        // Gaspedalstellung (Fahrpedalwert) - CAN-ID/Byte-Position sind ein
        // Platzhalter und müssen am Fahrzeug per CAN-Sniffer/Diagnosetool
        // verifiziert werden (analog zu 0x1D0)
        if (message.identifier == 0x1F0) {
            current_throttle_position = message.data[0] * (100.0f / 255.0f);
        }
        // Drehzahl (RPM) - CAN-ID/Byte-Position sind ein Platzhalter und
        // müssen am Fahrzeug per CAN-Sniffer/Diagnosetool verifiziert werden
        // (analog zu 0x1D0 und 0x1F0)
        if (message.identifier == 0x0AA) {
            current_rpm = (message.data[2] | (message.data[3] << 8)) * 0.25f;
        }
        // Quer-/Längsbeschleunigung (G-Kraft) sowie Geschwindigkeit - CAN-ID/
        // Byte-Position sind ein Platzhalter und müssen am Fahrzeug per
        // CAN-Sniffer/Diagnosetool verifiziert werden (analog zu 0x1D0/0x1F0/0x0AA)
        if (message.identifier == 0x0C4) {
            float raw_x = (int8_t)message.data[0] * 0.01f;
            float raw_y = (int8_t)message.data[1] * 0.01f;
            // Erster Frame nach dem Boot legt die aktuelle Lage/Vibration als
            // 0-Referenz fest, damit die Anzeige beim Start immer bei 0G startet
            // (unabhängig von Einbaulage/statischem Sensor-Offset).
            if (!gforce_calibrated) {
                gforce_offset_x = raw_x;
                gforce_offset_y = raw_y;
                gforce_calibrated = true;
            }
            current_gforce_x = raw_x - gforce_offset_x;
            current_gforce_y = raw_y - gforce_offset_y;
            current_speed_kmh = (message.data[2] | (message.data[3] << 8)) * 0.1f;
        }
    }
}

// Schwellenwerte (RPM) für die 6 LEDs der Schaltpunktanzeige: 4x Gelb
// (gleichmäßig zwischen Warnschwelle und Gruen-Schwelle verteilt), 1x Gruen
// (200 U/min vor dem Limit) und 1x Rot (Limit/Redline).
void computeRpmLedThresholds(float thresholds[6]) {
    float green_threshold = currentSettings.rpm_limit - 200.0f;
    float step = (green_threshold - currentSettings.rpm_warn) / 4.0f;
    for (int i = 0; i < 4; i++) {
        thresholds[i] = currentSettings.rpm_warn + step * i;
    }
    thresholds[4] = green_threshold;
    thresholds[5] = currentSettings.rpm_limit;
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
    lv_meter_set_indicator_value(water_meter, water_needle, (int32_t)current_water_temp);
    lv_meter_set_indicator_value(water_meter, water_needle_tip, (int32_t)current_water_temp);
    lv_color_t water_color = computeGaugeColor(current_water_temp, currentSettings.warn_temp, currentSettings.max_temp, false, blink_state);

    lv_label_set_text_fmt(water_label, "%d°C", (int)current_water_temp);
    lv_obj_set_style_text_color(water_label, water_color, 0);
    lv_label_set_text_fmt(water_secondary_label, "%.1fV", current_bat_voltage);

    // 2. BATTERIESPANNUNG
    lv_meter_set_indicator_value(bat_meter, bat_needle, (int32_t)(current_bat_voltage * 10));
    lv_meter_set_indicator_value(bat_meter, bat_needle_tip, (int32_t)(current_bat_voltage * 10));
    lv_color_t bat_color = computeGaugeColor(current_bat_voltage, currentSettings.warn_bat, currentSettings.min_bat, true, blink_state);

    lv_label_set_text_fmt(bat_label, "%.1fV", current_bat_voltage);
    lv_obj_set_style_text_color(bat_label, bat_color, 0);
    lv_label_set_text_fmt(bat_secondary_label, "%d°C", (int)current_water_temp);

    // 3. GASPEDALSTELLUNG
    lv_meter_set_indicator_value(throttle_meter, throttle_needle, (int32_t)current_throttle_position);
    lv_meter_set_indicator_value(throttle_meter, throttle_needle_tip, (int32_t)current_throttle_position);
    lv_color_t throttle_color = computeGaugeColor(current_throttle_position, currentSettings.throttle_warn_pct, currentSettings.throttle_max_pct, false, blink_state);

    lv_label_set_text_fmt(throttle_label, "%d%%", (int)current_throttle_position);
    lv_obj_set_style_text_color(throttle_label, throttle_color, 0);
    lv_label_set_text_fmt(throttle_secondary_label, "%d°C", (int)current_water_temp);

    // 4. DREHZAHL (Schaltpunktanzeige mit 6 LEDs statt digitaler Anzeige)
    lv_meter_set_indicator_value(rpm_meter, rpm_needle, (int32_t)current_rpm);
    lv_meter_set_indicator_value(rpm_meter, rpm_needle_tip, (int32_t)current_rpm);

    float rpm_thresholds[6];
    computeRpmLedThresholds(rpm_thresholds);
    bool shift_active = current_rpm >= rpm_thresholds[4]; // ab Gruen-LED blinkt die gesamte Anzeige
    lv_color_t led_off = lv_palette_darken(LV_PALETTE_GREY, 3);

    for (int i = 0; i < 6; i++) {
        bool led_on = current_rpm >= rpm_thresholds[i];
        lv_color_t on_color = (i < 4) ? lv_palette_main(LV_PALETTE_YELLOW)
                             : (i == 4) ? lv_palette_main(LV_PALETTE_GREEN)
                                        : lv_palette_main(LV_PALETTE_RED);
        lv_color_t led_color = led_off;
        if (led_on) {
            led_color = (shift_active && !blink_state) ? led_off : on_color;
        }
        lv_obj_set_style_bg_color(rpm_leds[i], led_color, 0);
    }

    // 5. G-KRAFT (Punkt im Polar-Raster + Betrag/Geschwindigkeit als Text)
    float gforce_mag = sqrtf(current_gforce_x * current_gforce_x + current_gforce_y * current_gforce_y);
    const float gforce_scale = 100.0f;  // Pixel pro g
    const float gforce_max_offset = 150.0f; // Anzeige-Radius entspricht ca. 1.5g
    float blob_x = current_gforce_x * gforce_scale;
    float blob_y = -current_gforce_y * gforce_scale; // oben = Beschleunigung, unten = Bremsen
    float mag_px = sqrtf(blob_x * blob_x + blob_y * blob_y);
    if (mag_px > gforce_max_offset && mag_px > 0.0f) {
        blob_x = blob_x * gforce_max_offset / mag_px;
        blob_y = blob_y * gforce_max_offset / mag_px;
    }
    lv_obj_align(gforce_blob, LV_ALIGN_CENTER, (int)blob_x, (int)blob_y);
    lv_label_set_text_fmt(gforce_value_label, "%.2fG", gforce_mag);
    lv_label_set_text_fmt(gforce_speed_label, "%d KM/H", (int)current_speed_kmh);

    // 6. MULTI-DATEN-ANZEIGE (Geschwindigkeit zentral, RPM-Nadel im Hintergrund,
    // 4 Zusatzfelder in Weiß - nur das letzte Feld (Wasser) faerbt sich ab 110°C orange)
    lv_meter_set_indicator_value(multi_meter, multi_needle_trail, (int32_t)current_rpm);
    lv_meter_set_indicator_value(multi_meter, multi_needle_tip, (int32_t)current_rpm);
    lv_label_set_text_fmt(multi_speed_label, "%d", (int)current_speed_kmh);
    lv_label_set_text_fmt(multi_bat_label, "%.1fV", current_bat_voltage);
    lv_label_set_text_fmt(multi_throttle_label, "%d%%", (int)current_throttle_position);
    lv_label_set_text_fmt(multi_rpm_label, "%d", (int)current_rpm);
    lv_label_set_text_fmt(multi_water_label, "%d°C", (int)current_water_temp);
    lv_obj_set_style_text_color(multi_water_label,
        current_water_temp >= 110.0f ? lv_palette_main(LV_PALETTE_ORANGE) : lv_color_white(), 0);
}

// Färbt einen Screen/Container im dunklen Tacho-Look (schwarzer Hintergrund, weißer Text)
void setDarkBg(lv_obj_t *obj) {
    lv_obj_set_style_bg_color(obj, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(obj, lv_color_white(), 0);
    lv_obj_set_style_border_width(obj, 0, 0);
}

// --- EINSTELLUNGS-SCREEN: SCHWELLENWERT-REGLER (+/- BUTTONS) ---
// Ein ThresholdStep hält den Zeiger auf den einzustellenden Settings-Wert,
// die Schrittweite (positiv für "+", negativ für "-"), die erlaubte Spanne
// sowie das Label, das nach jedem Klick aktualisiert wird.
struct ThresholdStep {
    float *value;
    float delta;
    float min_limit;
    float max_limit;
    lv_obj_t *value_label;
    const char *fmt;
};

void onThresholdStepBtn(lv_event_t *e) {
    ThresholdStep *ts = (ThresholdStep *)lv_event_get_user_data(e);
    float nv = *(ts->value) + ts->delta;
    if (nv < ts->min_limit) nv = ts->min_limit;
    if (nv > ts->max_limit) nv = ts->max_limit;
    *(ts->value) = nv;
    lv_label_set_text_fmt(ts->value_label, ts->fmt, nv);
}

// Baut eine Zeile mit Beschriftung, aktuellem Wert sowie "-"/"+" Buttons zum
// Verstellen eines Schwellenwerts. y_label/y_value/y_btn sind Offsets relativ
// zur Tile-Mitte (LV_ALIGN_CENTER).
void createThresholdRow(lv_obj_t *tile, const char *label_text, float *value, float step,
                         float min_limit, float max_limit, const char *fmt,
                         int y_label, int y_value, int y_btn) {
    lv_obj_t *lbl = lv_label_create(tile);
    lv_label_set_text(lbl, label_text);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, y_label);

    lv_obj_t *value_label = lv_label_create(tile);
    lv_obj_set_style_text_font(value_label, &lv_font_montserrat_48, 0);
    lv_label_set_text_fmt(value_label, fmt, *value);
    lv_obj_align(value_label, LV_ALIGN_CENTER, 0, y_value);

    lv_obj_t *btn_minus = lv_btn_create(tile);
    lv_obj_set_size(btn_minus, 50, 40);
    lv_obj_align(btn_minus, LV_ALIGN_CENTER, -90, y_btn);
    lv_obj_t *lbl_minus = lv_label_create(btn_minus);
    lv_label_set_text(lbl_minus, "-");
    lv_obj_center(lbl_minus);

    lv_obj_t *btn_plus = lv_btn_create(tile);
    lv_obj_set_size(btn_plus, 50, 40);
    lv_obj_align(btn_plus, LV_ALIGN_CENTER, 90, y_btn);
    lv_obj_t *lbl_plus = lv_label_create(btn_plus);
    lv_label_set_text(lbl_plus, "+");
    lv_obj_center(lbl_plus);

    ThresholdStep *minus_ctrl = new ThresholdStep{value, -step, min_limit, max_limit, value_label, fmt};
    ThresholdStep *plus_ctrl  = new ThresholdStep{value, step, min_limit, max_limit, value_label, fmt};
    lv_obj_add_event_cb(btn_minus, onThresholdStepBtn, LV_EVENT_CLICKED, minus_ctrl);
    lv_obj_add_event_cb(btn_plus, onThresholdStepBtn, LV_EVENT_CLICKED, plus_ctrl);
}

// Baut eine Einstellungs-Kachel mit Titel sowie Warnschwelle (Schwelle 1) und
// Limit/Alarmschwelle (Schwelle 2), jeweils per +/- verstellbar.
void createSettingsTile(lv_obj_t *tile, const char *title,
                         float *warn_value, float warn_step, float warn_min, float warn_max,
                         float *limit_value, float limit_step, float limit_min, float limit_max,
                         const char *fmt) {
    setDarkBg(tile);

    lv_obj_t *title_label = lv_label_create(tile);
    lv_label_set_text(title_label, title);
    lv_obj_align(title_label, LV_ALIGN_CENTER, 0, -180);

    createThresholdRow(tile, "Warnschwelle", warn_value, warn_step, warn_min, warn_max, fmt,
                        -110, -75, -70);
    createThresholdRow(tile, "Limit (Alarm)", limit_value, limit_step, limit_min, limit_max, fmt,
                        0, 35, 40);
}

// --- FARBAUSWAHL (PRIMÄR-/SEKUNDÄRFARBE) ---

// Vorwärtsdeklaration, da rebuildGaugesUI() (unten) createGaugesUI() aufruft,
// dessen Definition erst weiter unten in der Datei folgt.
void createGaugesUI();

// Baut den Haupt-Gauge-Screen (alle 5 Kacheln) neu auf, z.B. nach einem
// Farbwechsel in den Einstellungen - der aktuell aktive Screen (i.d.R. der
// Einstellungs-Screen) bleibt dabei unverändert sichtbar.
void rebuildGaugesUI() {
    lv_obj_t *old_main_screen = main_screen;
    createGaugesUI();
    lv_obj_del(old_main_screen);
}

struct ColorOption {
    const char *name;
    lv_color_t color;
};

// Farbpalette für die Primär-/Sekundärfarbe-Auswahl in den Einstellungen
// (inkl. Neongelb für beide Auswahlspalten).
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

struct ColorBtnCtx {
    int index;
    bool is_primary;
};

void onColorBtnClicked(lv_event_t *e) {
    ColorBtnCtx *ctx = (ColorBtnCtx *)lv_event_get_user_data(e);
    lv_color_t chosen = COLOR_PALETTE[ctx->index].color;
    if (ctx->is_primary) {
        currentSettings.color_primary = chosen;
    } else {
        currentSettings.color_secondary = chosen;
    }
    rebuildGaugesUI();
}

// Baut eine vertikal scrollbare Spalte mit Farb-Buttons zur Auswahl einer
// Primär- oder Sekundärfarbe. Eine Auswahl wirkt sich sofort auf alle 5
// Anzeigen aus (siehe rebuildGaugesUI()).
void createColorPickerColumn(lv_obj_t *tile, const char *title, int x_offset, bool is_primary) {
    lv_obj_t *header = lv_label_create(tile);
    lv_label_set_text(header, title);
    lv_obj_align(header, LV_ALIGN_TOP_MID, x_offset, 60);

    lv_obj_t *list = lv_obj_create(tile);
    setDarkBg(list);
    lv_obj_set_size(list, 130, 320);
    lv_obj_align(list, LV_ALIGN_TOP_MID, x_offset, 90);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(list, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_style_pad_row(list, 10, 0);

    for (int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        lv_obj_t *btn = lv_btn_create(list);
        lv_obj_set_size(btn, 60, 45);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_style_bg_color(btn, COLOR_PALETTE[i].color, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);

        ColorBtnCtx *ctx = new ColorBtnCtx{i, is_primary};
        lv_obj_add_event_cb(btn, onColorBtnClicked, LV_EVENT_CLICKED, ctx);
    }
}

// Baut die Einstellungs-Kachel "FARBEN": zwei vertikal scrollbare Spalten
// mit Farb-Buttons für Primär- und Sekundärfarbe (inkl. Neongelb).
void createColorsSettingsTile(lv_obj_t *tile) {
    setDarkBg(tile);

    lv_obj_t *title_label = lv_label_create(tile);
    lv_label_set_text(title_label, "FARBEN");
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 20);

    createColorPickerColumn(tile, "Primaer", -110, true);
    createColorPickerColumn(tile, "Sekundaer", 110, false);
}

// --- EINSTELLUNGS-SCREEN ---
void createSettingsUI() {
    settings_screen = lv_obj_create(NULL);
    setDarkBg(settings_screen);

    lv_obj_t *settings_tileview = lv_tileview_create(settings_screen);
    setDarkBg(settings_tileview);
    lv_obj_t *tile_set_water    = lv_tileview_add_tile(settings_tileview, 0, 0, LV_DIR_RIGHT);
    lv_obj_t *tile_set_bat      = lv_tileview_add_tile(settings_tileview, 1, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    lv_obj_t *tile_set_throttle = lv_tileview_add_tile(settings_tileview, 2, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    lv_obj_t *tile_set_rpm      = lv_tileview_add_tile(settings_tileview, 3, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    lv_obj_t *tile_set_colors   = lv_tileview_add_tile(settings_tileview, 4, 0, LV_DIR_LEFT);

    createSettingsTile(tile_set_water, "WASSER",
                        &currentSettings.warn_temp, 1.0f, 40, 130,
                        &currentSettings.max_temp, 1.0f, 40, 150,
                        "%.0f°C");

    createSettingsTile(tile_set_bat, "BATTERIE",
                        &currentSettings.warn_bat, 0.1f, 9.0f, 16.0f,
                        &currentSettings.min_bat, 0.1f, 8.0f, 16.0f,
                        "%.1fV");

    createSettingsTile(tile_set_throttle, "GASPEDAL",
                        &currentSettings.throttle_warn_pct, 1.0f, 0, 100,
                        &currentSettings.throttle_max_pct, 1.0f, 0, 100,
                        "%.0f%%");

    createSettingsTile(tile_set_rpm, "DREHZAHL",
                        &currentSettings.rpm_warn, 100.0f, 3000, 7900,
                        &currentSettings.rpm_limit, 100.0f, 4000, 8500,
                        "%.0f");

    createColorsSettingsTile(tile_set_colors);

    // Dropdown zur Auswahl der Start-Anzeige liegt als Overlay über dem
    // Tileview (bleibt auf allen Kacheln sichtbar/klickbar) und speichert
    // die Auswahl sofort stromlos in den Preferences (NVS).
    lv_obj_t *dd_startup = lv_dropdown_create(settings_screen);
    lv_dropdown_set_options(dd_startup, "Start: Multi\nStart: Wasser\nStart: Batterie\nStart: Gaspedal\nStart: Drehzahl\nStart: G-Force");
    lv_dropdown_set_selected(dd_startup, startup_gauge);
    lv_obj_set_width(dd_startup, 220);
    lv_obj_align(dd_startup, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_add_event_cb(dd_startup, [](lv_event_t *e) {
        lv_obj_t *dd = lv_event_get_target(e);
        saveStartupGauge((uint8_t)lv_dropdown_get_selected(dd));
    }, LV_EVENT_VALUE_CHANGED, NULL);

    // Speichern-Button liegt als Overlay über dem Tileview und bleibt auf
    // allen Kacheln sichtbar/klickbar.
    lv_obj_t *btn_save = lv_btn_create(settings_screen);
    lv_obj_align(btn_save, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_t *lbl = lv_label_create(btn_save);
    lv_label_set_text(lbl, "Speichern & Beenden");

    lv_obj_add_event_cb(btn_save, [](lv_event_t * e) {
        lv_scr_load_anim(main_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
    }, LV_EVENT_CLICKED, NULL);
}

// Anzahl der Segmente für den weichen Farbverlauf der Zonen-Bögen (je mehr
// Segmente, desto fließender der Übergang Primärfarbe -> Warnfarbe -> Alarmfarbe).
static const int ZONE_ARC_SEGMENTS = 16;

// Baut auf einem Meter/Scale sowohl die Zonen-Bögen ("Tacho Farben") als
// weich ineinander übergehende Segmente, als auch die passend eingefärbten
// Skala-Striche/Beschriftung ("Skala Striche") mittels LVGL-Farbverlauf
// (lv_meter_add_scale_lines). Beide folgen der Primärfarbe und blenden ab
// ca. 10% vor der Warnschwelle zur Warnfarbe (Gelb/Orange) und zwischen
// Warn- und Limitschwelle weiter zur Alarmfarbe (Rot).
// invert=true: niedrige Werte sind kritisch (z.B. Batteriespannung), der
// Verlauf läuft dann spiegelverkehrt.
void addZoneColoring(lv_obj_t *meter, lv_meter_scale_t *scale, float min_val, float max_val,
                     float warn_val, float alert_val, bool invert) {
    lv_color_t primary_color = currentSettings.color_primary;
    lv_color_t warn_color = lv_palette_main(LV_PALETTE_ORANGE);
    lv_color_t alert_color = lv_palette_main(LV_PALETTE_RED);

    // Weicher Farbverlauf des Zonen-Bogens in vielen kleinen Segmenten
    float step = (max_val - min_val) / ZONE_ARC_SEGMENTS;
    for (int i = 0; i < ZONE_ARC_SEGMENTS; i++) {
        float seg_start = min_val + step * i;
        float seg_end = seg_start + step;
        float seg_mid = (seg_start + seg_end) / 2.0f;
        lv_color_t seg_color = computeZoneColor(seg_mid, warn_val, alert_val, invert);
        lv_meter_indicator_t *seg = lv_meter_add_arc(meter, scale, 10, seg_color, 0);
        lv_meter_set_indicator_start_value(meter, seg, (int32_t)seg_start);
        lv_meter_set_indicator_end_value(meter, seg, (int32_t)seg_end);
    }

    // Skala-Striche/Beschriftung: nativer LVGL-Farbverlauf entlang der
    // Skala-Linien
    lv_meter_indicator_t *lines;
    if (!invert) {
        float pre_warn = warn_val - warn_val * 0.10f;
        lines = lv_meter_add_scale_lines(meter, scale, primary_color, warn_color, false, 0);
        lv_meter_set_indicator_start_value(meter, lines, (int32_t)pre_warn);
        lv_meter_set_indicator_end_value(meter, lines, (int32_t)warn_val);

        lines = lv_meter_add_scale_lines(meter, scale, warn_color, alert_color, false, 0);
        lv_meter_set_indicator_start_value(meter, lines, (int32_t)warn_val);
        lv_meter_set_indicator_end_value(meter, lines, (int32_t)alert_val);

        lines = lv_meter_add_scale_lines(meter, scale, alert_color, alert_color, false, 0);
        lv_meter_set_indicator_start_value(meter, lines, (int32_t)alert_val);
        lv_meter_set_indicator_end_value(meter, lines, (int32_t)max_val);
    } else {
        float pre_warn = warn_val + warn_val * 0.10f;
        lines = lv_meter_add_scale_lines(meter, scale, alert_color, alert_color, false, 0);
        lv_meter_set_indicator_start_value(meter, lines, (int32_t)min_val);
        lv_meter_set_indicator_end_value(meter, lines, (int32_t)alert_val);

        lines = lv_meter_add_scale_lines(meter, scale, alert_color, warn_color, false, 0);
        lv_meter_set_indicator_start_value(meter, lines, (int32_t)alert_val);
        lv_meter_set_indicator_end_value(meter, lines, (int32_t)warn_val);

        lines = lv_meter_add_scale_lines(meter, scale, warn_color, primary_color, false, 0);
        lv_meter_set_indicator_start_value(meter, lines, (int32_t)warn_val);
        lv_meter_set_indicator_end_value(meter, lines, (int32_t)pre_warn);
    }
}

// Baut einen an einen BMW-M3-Drehzahlmesser angelehnten Zeiger statt eines
// einfachen duennen Strichs: eine breite, kurze Basis (dunklere Sekundaerfarbe)
// naeher am Zentrum, darueber eine duenne, bis an die Skala reichende Spitze
// (helle Sekundaerfarbe) - zusammen ergibt das eine spitz zulaufende
// Nadelform. Ein rundes Metall-Hub-Cap mit farbigem Mittelpunkt verdeckt den
// Drehpunkt. Beide Indikatoren muessen zusammen mit lv_meter_set_indicator_value()
// aktualisiert werden (siehe updateGaugeUI()).
void createM3StyleNeedle(lv_obj_t *meter, lv_meter_scale_t *scale,
                          lv_meter_indicator_t **out_base, lv_meter_indicator_t **out_tip) {
    lv_color_t base_color = mix_colors(currentSettings.color_secondary, lv_color_black(), 0.30f);
    lv_color_t tip_color = mix_colors(currentSettings.color_secondary, lv_color_white(), 0.25f);

    // Durchgehender, spitz zulaufender Zeiger: dicke Wurzel direkt am Hub, die
    // ohne Luecke in die duenne Spitze bis fast an die Skala uebergeht. Die
    // Basis reicht bis Radius-150 (kurze breite Wurzel), die Spitze darueber
    // liegend bis Radius-8 - beide vom selben Drehpunkt, also kollinear.
    *out_base = lv_meter_add_needle_line(meter, scale, 8, base_color, -150);
    *out_tip = lv_meter_add_needle_line(meter, scale, 3, tip_color, -8);

    lv_obj_t *hub_outer = lv_obj_create(meter);
    lv_obj_set_size(hub_outer, 34, 34);
    lv_obj_set_style_radius(hub_outer, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hub_outer, lv_palette_darken(LV_PALETTE_GREY, 4), 0);
    lv_obj_set_style_bg_opa(hub_outer, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(hub_outer, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
    lv_obj_set_style_border_width(hub_outer, 2, 0);
    lv_obj_center(hub_outer);
    lv_obj_clear_flag(hub_outer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(hub_outer, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *hub_center = lv_obj_create(meter);
    lv_obj_set_size(hub_center, 14, 14);
    lv_obj_set_style_radius(hub_center, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hub_center, currentSettings.color_primary, 0);
    lv_obj_set_style_bg_opa(hub_center, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hub_center, 0, 0);
    lv_obj_center(hub_center);
    lv_obj_clear_flag(hub_center, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(hub_center, LV_OBJ_FLAG_CLICKABLE);
}

// Baut ein rundes Tacho-Style-Meter mit Skalenstrichen, weich eingefärbten
// Farbzonen (Normal/Warnung/Alarm) sowie Nadel und großer digitaler Anzeige
// im Zentrum. invert_zones=true: niedrige Werte sind kritisch (z.B. Batteriespannung).
void createStyledMeter(lv_obj_t *tile, const char *title, float min_val, float max_val,
                        float warn_val, float alert_val, bool invert_zones,
                        lv_obj_t **out_meter, lv_meter_indicator_t **out_needle,
                        lv_meter_indicator_t **out_needle_tip, lv_obj_t **out_value_label,
                        lv_obj_t **out_secondary_label) {
    setDarkBg(tile);

    lv_obj_t *meter = lv_meter_create(tile);
    setDarkBg(meter);
    lv_obj_center(meter);
    lv_obj_set_size(meter, 450, 450);

    lv_meter_scale_t *scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale, (int32_t)min_val, (int32_t)max_val, 270, 135);
    lv_meter_set_scale_ticks(meter, scale, 41, 2, 8, currentSettings.color_primary);
    lv_meter_set_scale_major_ticks(meter, scale, 8, 3, 14, currentSettings.color_primary, 12);

    addZoneColoring(meter, scale, min_val, max_val, warn_val, alert_val, invert_zones);

    lv_meter_indicator_t *needle;
    lv_meter_indicator_t *needle_tip;
    createM3StyleNeedle(meter, scale, &needle, &needle_tip);

    lv_obj_t *value_label = lv_label_create(tile);
    lv_obj_set_style_text_font(value_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(value_label, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(value_label, LV_ALIGN_CENTER, 0, 85);

    lv_obj_t *title_label = lv_label_create(tile);
    lv_label_set_text(title_label, title);
    lv_obj_align(title_label, LV_ALIGN_CENTER, 0, 130);

    lv_obj_t *secondary_label = lv_label_create(tile);
    lv_obj_set_style_text_color(secondary_label, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(secondary_label, LV_ALIGN_CENTER, 0, 160);

    *out_meter = meter;
    *out_needle = needle;
    *out_needle_tip = needle_tip;
    *out_value_label = value_label;
    *out_secondary_label = secondary_label;
}

// Baut die Drehzahl-Kachel: rundes Tacho-Meter (0-8000 U/min) mit Nadel und
// Farbzonen, darunter statt einer digitalen Anzeige 6 LED-Punkte als
// Schaltpunktanzeige (ausgegraut, dann 4x Gelb, 1x Gruen, 1x Rot -
// siehe computeRpmLedThresholds()).
void createRpmTile(lv_obj_t *tile) {
    setDarkBg(tile);

    rpm_meter = lv_meter_create(tile);
    setDarkBg(rpm_meter);
    lv_obj_center(rpm_meter);
    lv_obj_set_size(rpm_meter, 450, 450);

    lv_meter_scale_t *scale = lv_meter_add_scale(rpm_meter);
    lv_meter_set_scale_range(rpm_meter, scale, 0, 8000, 270, 135);
    lv_meter_set_scale_ticks(rpm_meter, scale, 41, 2, 8, currentSettings.color_primary);
    lv_meter_set_scale_major_ticks(rpm_meter, scale, 8, 3, 14, currentSettings.color_primary, 12);

    addZoneColoring(rpm_meter, scale, 0, 8000, currentSettings.rpm_warn, currentSettings.rpm_limit, false);

    createM3StyleNeedle(rpm_meter, scale, &rpm_needle, &rpm_needle_tip);

    // 6 LED-Punkte anstelle der digitalen Anzeige
    const int led_x[6] = {-125, -75, -25, 25, 75, 125};
    for (int i = 0; i < 6; i++) {
        rpm_leds[i] = lv_obj_create(tile);
        lv_obj_set_size(rpm_leds[i], 40, 40);
        lv_obj_set_style_radius(rpm_leds[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(rpm_leds[i], 0, 0);
        lv_obj_set_style_bg_color(rpm_leds[i], lv_palette_darken(LV_PALETTE_GREY, 3), 0);
        lv_obj_set_style_bg_opa(rpm_leds[i], LV_OPA_COVER, 0);
        lv_obj_align(rpm_leds[i], LV_ALIGN_CENTER, led_x[i], 85);
    }

    lv_obj_t *title_label = lv_label_create(tile);
    lv_label_set_text(title_label, "DREHZAHL");
    lv_obj_align(title_label, LV_ALIGN_CENTER, 0, 130);
}

// Baut die G-Kraft-Kachel: ein kreisförmiges Polar-Raster (Ringe + Kreuz-
// Linien in der Primärfarbe) sowie ein Punkt in der Sekundärfarbe, dessen
// Position die aktuelle Quer-/Längsbeschleunigung anzeigt - Betrag (G) und
// Geschwindigkeit werden mittig im Raster als Text angezeigt (siehe
// updateGaugeUI()).
void createGForceTile(lv_obj_t *tile) {
    setDarkBg(tile);

    lv_obj_t *title_label = lv_label_create(tile);
    lv_label_set_text(title_label, "G-FORCE");
    lv_obj_set_style_text_color(title_label, currentSettings.color_primary, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 30);

    const int GF_SIZE = 360;
    lv_obj_t *field = lv_obj_create(tile);
    setDarkBg(field);
    lv_obj_set_size(field, GF_SIZE, GF_SIZE);
    lv_obj_align(field, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_radius(field, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(field, 2, 0);
    lv_obj_set_style_border_color(field, currentSettings.color_primary, 0);
    lv_obj_clear_flag(field, LV_OBJ_FLAG_SCROLLABLE);

    // Konzentrische Ringe
    const int ring_sizes[2] = {GF_SIZE * 2 / 3, GF_SIZE / 3};
    for (int i = 0; i < 2; i++) {
        lv_obj_t *ring = lv_obj_create(field);
        lv_obj_set_size(ring, ring_sizes[i], ring_sizes[i]);
        lv_obj_center(ring);
        lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(ring, 1, 0);
        lv_obj_set_style_border_color(ring, currentSettings.color_primary, 0);
        lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);
    }

    // Kreuz-Linien durch die Mitte
    static lv_point_t h_pts[2] = {{0, GF_SIZE / 2}, {GF_SIZE, GF_SIZE / 2}};
    static lv_point_t v_pts[2] = {{GF_SIZE / 2, 0}, {GF_SIZE / 2, GF_SIZE}};

    lv_obj_t *line_h = lv_line_create(field);
    lv_line_set_points(line_h, h_pts, 2);
    lv_obj_set_style_line_color(line_h, currentSettings.color_primary, 0);
    lv_obj_set_style_line_width(line_h, 1, 0);

    lv_obj_t *line_v = lv_line_create(field);
    lv_line_set_points(line_v, v_pts, 2);
    lv_obj_set_style_line_color(line_v, currentSettings.color_primary, 0);
    lv_obj_set_style_line_width(line_v, 1, 0);

    // G-Kraft-Punkt (Sekundärfarbe) - Position wird in updateGaugeUI() gesetzt
    gforce_blob = lv_obj_create(field);
    lv_obj_set_size(gforce_blob, 24, 24);
    lv_obj_set_style_radius(gforce_blob, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(gforce_blob, 0, 0);
    lv_obj_set_style_bg_color(gforce_blob, currentSettings.color_secondary, 0);
    lv_obj_set_style_bg_opa(gforce_blob, LV_OPA_COVER, 0);
    lv_obj_align(gforce_blob, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(gforce_blob, LV_OBJ_FLAG_SCROLLABLE);

    gforce_value_label = lv_label_create(field);
    lv_obj_set_style_text_font(gforce_value_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(gforce_value_label, currentSettings.color_secondary, 0);
    lv_obj_align(gforce_value_label, LV_ALIGN_CENTER, 0, -15);

    gforce_speed_label = lv_label_create(field);
    lv_obj_set_style_text_color(gforce_speed_label, lv_color_white(), 0);
    lv_obj_align(gforce_speed_label, LV_ALIGN_CENTER, 0, 25);
}

// Baut einen weißen, spitz zulaufenden Nadel-Zeiger mit dunkel eingefärbtem
// "Schweif" (kurze breite Basis nahe der Mitte, dünne helle Spitze bis zur
// Skala) - für die Multi-Daten-Anzeige, unabhängig von der Sekundärfarbe.
void createTrailStyleNeedle(lv_obj_t *meter, lv_meter_scale_t *scale,
                             lv_meter_indicator_t **out_trail, lv_meter_indicator_t **out_tip) {
    lv_color_t trail_color = mix_colors(lv_color_white(), lv_color_black(), 0.65f);
    *out_trail = lv_meter_add_needle_line(meter, scale, 6, trail_color, -60);
    *out_tip = lv_meter_add_needle_line(meter, scale, 2, lv_color_white(), -6);

    lv_obj_t *hub = lv_obj_create(meter);
    lv_obj_set_size(hub, 14, 14);
    lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hub, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(hub, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hub, 0, 0);
    lv_obj_center(hub);
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_CLICKABLE);
}

// Baut die Multi-Daten-Kachel (erste Kachel, kein Titel): statisches
// Hintergrundbild (multi_bg_img, RPM-Ring 0-8000 fest eingefaerbt, reagiert
// NICHT auf die Primär-/Sekundärfarbe), mittig groß die Geschwindigkeit in
// Primärfarbe, darunter 4 Zusatzfelder in Weiß (Batterie/Gaspedal/Drehzahl/
// Wasser) - nur das letzte Feld (Wasser) faerbt sich ab 110°C orange (siehe
// updateGaugeUI()).
void createMultiTile(lv_obj_t *tile) {
    setDarkBg(tile);

    // Statisches Hintergrundbild (RPM-Ring, Skala, Farbverlauf, LED-Boxen) -
    // ersetzt die zuvor per LVGL gezeichnete Ring-Skala/Farbzonen.
    lv_obj_t *bg_img = lv_img_create(tile);
    lv_img_set_src(bg_img, &multi_bg_img);
    lv_obj_center(bg_img);

    // Meter bleibt nur fuer die Nadel-Winkelberechnung bestehen (transparent,
    // ohne eigene Skala/Ticks/Farbzonen, da bereits im Hintergrundbild).
    multi_meter = lv_meter_create(tile);
    lv_obj_set_style_bg_opa(multi_meter, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(multi_meter, 0, 0);
    lv_obj_center(multi_meter);
    lv_obj_set_size(multi_meter, 450, 450);

    lv_meter_scale_t *scale = lv_meter_add_scale(multi_meter);
    lv_meter_set_scale_range(multi_meter, scale, 0, 8000, 270, 135);

    createTrailStyleNeedle(multi_meter, scale, &multi_needle_trail, &multi_needle_tip);

    multi_speed_label = lv_label_create(tile);
    lv_obj_set_style_text_font(multi_speed_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(multi_speed_label, currentSettings.color_primary, 0);
    lv_obj_align(multi_speed_label, LV_ALIGN_CENTER, 0, -60);

    lv_obj_t *unit_label = lv_label_create(tile);
    lv_label_set_text(unit_label, "KM/H");
    lv_obj_set_style_text_color(unit_label, lv_color_white(), 0);
    lv_obj_align(unit_label, LV_ALIGN_CENTER, 0, -15);

    const int field_x[4] = {-165, -55, 55, 165};
    multi_bat_label      = lv_label_create(tile);
    multi_throttle_label = lv_label_create(tile);
    multi_rpm_label      = lv_label_create(tile);
    multi_water_label    = lv_label_create(tile);
    lv_obj_t *fields[4] = {multi_bat_label, multi_throttle_label, multi_rpm_label, multi_water_label};
    for (int i = 0; i < 4; i++) {
        lv_obj_set_style_text_color(fields[i], lv_color_white(), 0);
        lv_obj_align(fields[i], LV_ALIGN_CENTER, field_x[i], 60);
    }
}

// --- HAUPT GAUGE UI (TILEVIEW: MULTI / WASSER / BATTERIE / GASPEDAL / DREHZAHL / G-FORCE) ---
void createGaugesUI() {
    main_screen = lv_obj_create(NULL);
    setDarkBg(main_screen);

    tileview = lv_tileview_create(main_screen);
    setDarkBg(tileview);
    tile_multi    = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_RIGHT);
    tile_water    = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    tile_bat      = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    tile_throttle = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    tile_rpm      = lv_tileview_add_tile(tileview, 4, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    tile_gforce   = lv_tileview_add_tile(tileview, 5, 0, LV_DIR_LEFT);

    // TILE 0: MULTI-DATEN-ANZEIGE (Geschwindigkeit zentral, RPM-Ring im Hintergrund)
    createMultiTile(tile_multi);

    // TILE 1: WASSERTEMP (40-130 °C, Warnung ab warn_temp, Alarm ab max_temp)
    createStyledMeter(tile_water, "WASSER", 40, 130, currentSettings.warn_temp, currentSettings.max_temp,
                       false, &water_meter, &water_needle, &water_needle_tip, &water_label, &water_secondary_label);

    // TILE 2: BATTERIE (10.0-16.0 V, intern *10 skaliert für Ganzzahl-Genauigkeit)
    createStyledMeter(tile_bat, "BATTERIE", 100, 160, currentSettings.warn_bat * 10, currentSettings.min_bat * 10,
                       true, &bat_meter, &bat_needle, &bat_needle_tip, &bat_label, &bat_secondary_label);

    // TILE 3: GASPEDALSTELLUNG (0-100 %)
    createStyledMeter(tile_throttle, "GASPEDAL", 0, 100, currentSettings.throttle_warn_pct, currentSettings.throttle_max_pct,
                       false, &throttle_meter, &throttle_needle, &throttle_needle_tip, &throttle_label, &throttle_secondary_label);

    // TILE 4: DREHZAHL (0-8000 U/min, Schaltpunktanzeige statt digitaler Anzeige)
    createRpmTile(tile_rpm);

    // TILE 5: G-FORCE (Polar-Raster mit Punkt für Quer-/Längsbeschleunigung)
    createGForceTile(tile_gforce);

    // Zuletzt in den Einstellungen gewählte Start-Kachel anzeigen (stromlos gespeichert)
    lv_obj_set_tile_id(tileview, startup_gauge, 0, LV_ANIM_OFF);
}

// --- FEHLERCODE-ÜBERSICHT (per Doppeltipp erreichbar) ---
void createDtcUI() {
    dtc_screen = lv_obj_create(NULL);
    setDarkBg(dtc_screen);

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
    delay(2500); // USB-CDC enumerieren lassen, damit die ersten Logs ankommen
    Serial.println("\n[BOOT] setup() start");

    // I2C fuer Touch UND IO-Expander frueh starten. Der TCA9554 muss VOR der
    // Display-Init IO6 (Power-Hold) auf HIGH ziehen, sonst schaltet sich das
    // Board nach dem Einschalt-Impuls wieder aus (Bootloop).
    Wire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN);
    Serial.println("[BOOT] Wire.begin ok");
    initIoExpander();
    Serial.println("[BOOT] initIoExpander ok");

    gfx->begin(40000000); // 40 MHz QSPI, wie im offiziellen Waveshare-BSP
    Serial.println("[BOOT] gfx->begin ok");
    // Redundant zur letzten Zeile in Arduino_SH8601W::tftInit() (dort bereits
    // auf 0xFF gesetzt), schadet aber nicht.
    gfx->setBrightness(255);
    gfx->fillScreen(RGB565_BLACK);
    Serial.println("[BOOT] display init ok");

    SPI.begin();

    // 1. Boot-Animation abspielen (nur wenn SD-Karte gesteckt ist)
    Serial.println("[BOOT] SD.begin ...");
    if (SD.begin(SD_CS_PIN)) {
        Serial.println("[BOOT] SD ok, playing animation");
        playBootAnimation(currentSettings.brand);
    } else {
        Serial.println("[BOOT] SD not present, skipping");
    }

    // 2. LVGL initialisieren
    Serial.println("[BOOT] lv_init ...");
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, DISPLAY_WIDTH * 30);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISPLAY_WIDTH;
    disp_drv.ver_res = DISPLAY_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpadReadCb;
    lv_indev_drv_register(&indev_drv);

    // 3. UI & CAN aufbauen
    Serial.println("[BOOT] building UI ...");
    loadStartupGauge();
    createSettingsUI();
    createGaugesUI();
    lv_scr_load(main_screen);
    createDtcUI();
    Serial.println("[BOOT] UI ok, initCAN ...");
    initCAN();
    Serial.println("[BOOT] setup() done");
}

void loop() {
    lv_timer_handler();
    processCAN();
    updateGaugeUI();
    checkCenterTouch();
    checkDoubleTap();
    delay(5);
}
