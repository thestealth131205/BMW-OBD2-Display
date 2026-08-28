/*
 * BMW E90 OBD2 Display
 * ──────────────────────────────────────────────────────────────
 * Hardware:
 *   ESP32 DevKit N4
 *   SSD1309 128x64 OLED (I2C)
 *   MCP2515 + TJA1050 CAN-Bus-Modul
 *
 * Messwerte:
 *   - Oeltemperatur         (OBD2 PID 0x5C)
 *   - Kuehlmitteltemperatur (OBD2 PID 0x05)
 *   - Abgastemperatur       (PT-CAN Broadcast, ID variiert – siehe Hinweise)
 *   - Lambda-Spannung B1S1  (OBD2 PID 0x24 Wideband)
 *   - Klemme-15-Spannung    (PT-CAN Broadcast, ID 0x0AC – muss verifiziert werden)
 *
 * Benoetiste Libraries (Arduino Library Manager):
 *   - U8g2    by olikraus   (Display)
 *   - mcp2515 by autowp     (CAN Bus)
 *
 * Verdrahtung:
 *   SSD1309  PIN1 GND  -> GND
 *   SSD1309  PIN2 VDD  -> 3.3V
 *   SSD1309  PIN3 SCL  -> GPIO22
 *   SSD1309  PIN4 SDA  -> GPIO21
 *
 *   MCP2515  GND       -> GND
 *   MCP2515  VCC       -> 3.3V (oder 5V je nach Modul)
 *   MCP2515  CS        -> GPIO5
 *   MCP2515  SCK       -> GPIO18
 *   MCP2515  MOSI      -> GPIO23
 *   MCP2515  MISO      -> GPIO19
 *   MCP2515  INT       -> GPIO4  (optional)
 *   MCP2515  CANH      -> OBD2 PIN6  (CAN High)
 *   MCP2515  CANL      -> OBD2 PIN14 (CAN Low)
 *
 * HINWEISE:
 *   - MCP2515-Modulcrystal: Standard 8MHz. Bei 16MHz -> MCP_16MHZ in setup() aendern.
 *   - Abgastemperatur: Nur bei Diesel serienmassig. Benziner -> externer Thermocouple.
 *   - KL15 / Abgas CAN-ID: Per CAN-Sniffer am eigenen Fahrzeug verifizieren.
 *   - Den Debug-Block am Ende von loop() einkommentieren um alle CAN-IDs zu loggen.
 */

#include <SPI.h>
#include <mcp2515.h>
#include <U8g2lib.h>
#include <Wire.h>

// ── Pin-Konfiguration ─────────────────────────────────────────
#define PIN_CAN_CS   5
#define PIN_CAN_INT  4
#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22

// ── CAN / OBD2 ────────────────────────────────────────────────
#define OBD2_REQ_ID       0x7DF   // Funktionaler Broadcast (alle ECUs)
#define OBD2_RESP_DME     0x7E8   // Antwort DME (Motorsteuergeraet)

#define PID_COOLANT_TEMP  0x05    // Byte3 - 40 = Grad C
#define PID_OIL_TEMP      0x5C    // Byte3 - 40 = Grad C
#define PID_O2_WIDEBAND   0x24    // Wideband-Lambda B1S1
                                   //   Spannung = (Byte4<<8 | Byte5) / 8192.0 V

// BMW PT-CAN Broadcasts (500 kbps):
//   KL15-Spannung: ID 0x0AC, Byte4+5 big-endian, Faktor 0.1V
//   Abgastemperatur (Diesel): ID 0x32C, Byte0+1 big-endian, Faktor 0.5, Offset -40
//   WICHTIG: Diese IDs koennen je nach Baujahr/SW-Stand abweichen!
//   Mit aktivem Debug-Logging verifizieren (siehe Ende von loop()).
#define BMW_KL15_CAN_ID     0x0AC
#define BMW_EXHAUST_CAN_ID  0x32C

// Auf 0 setzen wenn kein Abgassensor vorhanden (Benziner ohne Nachrueststung)
#define BMW_EXHAUST_ENABLED 1

// ── Display ───────────────────────────────────────────────────
// I2C-Adresse Standard: 0x3C. Falls nicht erkannt: u8g2.setI2CAddress(0x7A) fuer 0x3D.
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(
    U8G2_R0, U8X8_PIN_NONE, PIN_I2C_SCL, PIN_I2C_SDA);

// ── MCP2515 ───────────────────────────────────────────────────
MCP2515 mcp2515(PIN_CAN_CS);

// ── Messdaten ─────────────────────────────────────────────────
struct {
  float oilTemp     = 0.0f;
  float coolantTemp = 0.0f;
  float exhaustTemp = 0.0f;
  float lambdaVolt  = 0.0f;
  float kl15Volt    = 0.0f;
  bool  oilOk       = false;
  bool  coolantOk   = false;
  bool  exhaustOk   = false;
  bool  lambdaOk    = false;
  bool  kl15Ok      = false;
} sens;

// ── OBD2-Polling ─────────────────────────────────────────────
const uint8_t  PIDS[]    = {PID_COOLANT_TEMP, PID_OIL_TEMP, PID_O2_WIDEBAND};
const uint8_t  NUM_PIDS  = sizeof(PIDS) / sizeof(PIDS[0]);
uint8_t        pidIdx    = 0;
uint32_t       lastPollMs = 0;
const uint32_t POLL_MS   = 300;   // ms zwischen OBD2-Anfragen

// ── OBD2-Anfrage senden ───────────────────────────────────────
void sendOBD2Request(uint8_t pid) {
  struct can_frame f = {};
  f.can_id  = OBD2_REQ_ID;
  f.can_dlc = 8;
  f.data[0] = 0x02;   // Nutzlastlaenge
  f.data[1] = 0x01;   // Mode 01 = aktuelle Betriebsdaten
  f.data[2] = pid;
  f.data[3] = f.data[4] = f.data[5] = f.data[6] = f.data[7] = 0x55;
  mcp2515.sendMessage(&f);
}

// ── CAN-Frame auswerten ───────────────────────────────────────
void handleFrame(struct can_frame &f) {

  // OBD2-Antwort (Mode 01 -> positive Response 0x41)
  if (f.can_id == OBD2_RESP_DME && f.data[1] == 0x41) {
    switch (f.data[2]) {

      case PID_COOLANT_TEMP:
        sens.coolantTemp = (float)f.data[3] - 40.0f;
        sens.coolantOk   = true;
        break;

      case PID_OIL_TEMP:
        sens.oilTemp = (float)f.data[3] - 40.0f;
        sens.oilOk   = true;
        break;

      case PID_O2_WIDEBAND:
        if (f.can_dlc >= 6) {
          uint16_t raw    = ((uint16_t)f.data[4] << 8) | f.data[5];
          float    v      = raw / 8192.0f;
          if (v >= 0.0f && v <= 5.0f) {   // Plausibilitaet
            sens.lambdaVolt = v;
            sens.lambdaOk   = true;
          }
        }
        break;
    }
  }

  // BMW PT-CAN: Klemme-15-Spannung
  if (f.can_id == BMW_KL15_CAN_ID && f.can_dlc >= 6) {
    uint16_t raw = ((uint16_t)f.data[4] << 8) | f.data[5];
    float    v   = raw * 0.1f;
    if (v >= 6.0f && v <= 20.0f) {   // Plausibilitaet: 6-20V
      sens.kl15Volt = v;
      sens.kl15Ok   = true;
    }
  }

#if BMW_EXHAUST_ENABLED
  // BMW PT-CAN: Abgastemperatur (E90 Diesel, Wert per Sniffer pruefen!)
  if (f.can_id == BMW_EXHAUST_CAN_ID && f.can_dlc >= 2) {
    uint16_t raw = ((uint16_t)f.data[0] << 8) | f.data[1];
    float    t   = raw * 0.5f - 40.0f;
    if (t > -40.0f && t < 1200.0f) {
      sens.exhaustTemp = t;
      sens.exhaustOk   = true;
    }
  }
#endif
}

// ── Display-Darstellung ───────────────────────────────────────
//
//  5 Zeilen, jede Zeile: kleines Label (links) + grosser Wert (rechts)
//
//  Fonts:
//    Label : u8g2_font_5x8_tf      5px breit, 8px hoch
//    Wert  : u8g2_font_7x13B_tf    7px breit, 13px hoch, fett
//
//  Baselines: y = 12, 25, 38, 51, 64  (Abstand 13px)
//
void drawDisplay() {
  u8g2.clearBuffer();

  const char *labels[5] = {"OEL", "KMT", "ABG", "LSO", "KL15"};
  char        vals[5][12];

  // Werte als Strings formatieren (--- wenn kein Signal)
  if (sens.oilOk)
    snprintf(vals[0], 12, "%3.0f C", sens.oilTemp);
  else
    strcpy(vals[0], "--- C");

  if (sens.coolantOk)
    snprintf(vals[1], 12, "%3.0f C", sens.coolantTemp);
  else
    strcpy(vals[1], "--- C");

  if (sens.exhaustOk)
    snprintf(vals[2], 12, "%3.0f C", sens.exhaustTemp);
  else
    strcpy(vals[2], "--- C");

  if (sens.lambdaOk)
    snprintf(vals[3], 12, "%.3fV", sens.lambdaVolt);
  else
    strcpy(vals[3], "-.---V");

  if (sens.kl15Ok)
    snprintf(vals[4], 12, "%.1fV", sens.kl15Volt);
  else
    strcpy(vals[4], "--.-V");

  for (int i = 0; i < 5; i++) {
    int y = 12 + i * 13;

    // Label klein links
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(0, y, labels[i]);

    // Wert gross rechtsbuendig
    u8g2.setFont(u8g2_font_7x13B_tf);
    int w = u8g2.getStrWidth(vals[i]);
    u8g2.drawStr(128 - w, y, vals[i]);
  }

  u8g2.sendBuffer();
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("BMW E90 OBD2 Display gestartet");

  // Display
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!u8g2.begin()) {
    Serial.println("FEHLER: Display nicht gefunden! SDA/SCL pruefen.");
    while (true) delay(500);
  }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(16, 28, "BMW E90");
  u8g2.drawStr(4, 44, "OBD2 Display");
  u8g2.sendBuffer();
  delay(2000);

  // MCP2515 CAN
  SPI.begin(18, 19, 23, PIN_CAN_CS);
  mcp2515.reset();

  // 500 kbps -- MCP2515 mit 8 MHz Crystal (Standard bei guenstigen Modulen)
  // Bei 16 MHz Crystal die Zeile austauschen:
  MCP2515::ERROR err = mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  // MCP2515::ERROR err = mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);

  if (err != MCP2515::ERROR_OK) {
    Serial.println("FEHLER: CAN-Bitrate konnte nicht gesetzt werden!");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(4,  20, "CAN FEHLER!");
    u8g2.drawStr(4,  36, "MCP2515 pruefen");
    u8g2.drawStr(4,  52, "8 oder 16 MHz?");
    u8g2.sendBuffer();
    while (true) delay(500);
  }

  mcp2515.setNormalMode();
  Serial.println("CAN Bus bereit – 500 kbps");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // OBD2 PID-Polling
  if (now - lastPollMs >= POLL_MS) {
    lastPollMs = now;
    sendOBD2Request(PIDS[pidIdx]);
    pidIdx = (pidIdx + 1) % NUM_PIDS;
  }

  // Alle eingehenden CAN-Frames lesen
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    handleFrame(frame);

    // ── DEBUG: alle CAN-IDs auf Serial ausgeben ──────────────
    // Einkommentieren um BMW-spezifische Broadcast-IDs zu finden.
    // z.B. KL15 und Abgastemperatur-ID verifizieren.
    //
    // Serial.printf("ID:0x%03X DLC:%d", frame.can_id, frame.can_dlc);
    // for (int i = 0; i < frame.can_dlc; i++)
    //   Serial.printf(" %02X", frame.data[i]);
    // Serial.println();
    // ──────────────────────────────────────────────────────────
  }

  // Display alle 300 ms aktualisieren
  static uint32_t lastDisplayMs = 0;
  if (now - lastDisplayMs >= 300) {
    lastDisplayMs = now;
    drawDisplay();
  }
}
