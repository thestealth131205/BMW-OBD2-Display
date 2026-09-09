# BMW E90 OBD2 Display

ESP32-S3-basiertes CAN-Bus-Gauge-Display für einen BMW E90 320i (N43B20A, 2010).
Zeigt eine Multi-Daten-Kachel (Geschwindigkeit zentral, Kühlmitteltemperatur-Ring
im Hintergrund plus Batterie/Gaspedal/Drehzahl als Zusatzfelder) sowie
Kühlmitteltemperatur, Batteriespannung, Gaspedalstellung als analoge
Tacho-Style-Rundinstrumente (LVGL) mit Farbzonen, Drehzahl als
Schaltpunktanzeige (6 LEDs) sowie G-Kraft als Polar-Raster-Grafik auf einem
480×480 RGB-Touchdisplay an, inkl. Diagnose (DTCs lesen/löschen) und BMW
CBS-Service-Reset.

## Hardware

| Komponente | Modell |
|---|---|
| Mikrocontroller | ESP32-S3 (Waveshare ESP32-S3-Touch-LCD-2.1) |
| Display | ST7701S RGB-TFT, 480×480, über SPI-Init + paralleles RGB-Interface |
| Touch | CST820 (kapazitiv), I2C |
| IO-Expander | TCA9554 (Adresse 0x20) – steuert u. a. Touch-Reset/weitere Enable-Leitungen |
| CAN-Bus | externes **MCP2515-Modul** (SPI, TJA1050-Transceiver) |
| Batteriemessung | interner ADC (Spannungsteiler auf dem Board) |

Framework: **ESP-IDF** (nicht Arduino/PlatformIO), LVGL v8.2 via ESP Component
Manager.

## Pinbelegung

### ST7701S-Display (fest verlötet, keine externe Verdrahtung nötig)

| Signal | GPIO | Signal | GPIO |
|---|---|---|---|
| SPI SDA (Init) | 1 | SPI SCLK (Init) | 2 |
| HSYNC | 38 | VSYNC | 39 |
| DE | 40 | PCLK | 41 |
| Backlight (PWM) | 6 | | |
| B0–B4 (DATA0–4) | 5, 45, 48, 47, 21 | | |
| G0–G5 (DATA5–10) | 14, 13, 12, 11, 10, 9 | | |
| R0–R4 (DATA11–15) | 46, 3, 8, 18, 17 | | |

### I2C-Bus (Touch CST820 + IO-Expander TCA9554)

| Signal | GPIO |
|---|---|
| SCL | 7 |
| SDA | 15 |
| Touch-INT | 16 |
| Touch-RST | -1 (nicht verbunden, Reset über TCA9554) |

Ebenfalls fest verlötet, keine externe Verdrahtung nötig.

### MCP2515-CAN-Modul (extern anzuschließen)

**Wichtig:** Auf dem ESP32-S3-Touch-LCD-2.1 sind fast alle GPIOs durch das
RGB-Display und I2C belegt. Nur **GPIO 0, 19, 20, 43, 44** sind frei
herausgeführt.

| MCP2515-Modul | ESP32-S3 GPIO | Funktion |
|---|---|---|
| VCC | 3V3 | Versorgung |
| GND | GND | Masse |
| SCK | GPIO43 | SPI-Takt |
| SI (MOSI) | GPIO44 | SPI Data ESP→MCP2515 |
| SO (MISO) | GPIO19 | SPI Data MCP2515→ESP |
| CS | GPIO20 | SPI Chip-Select |
| INT | GPIO0 | Interrupt (Polling-Fallback möglich) |

**Achtung Pin-Konflikt:** GPIO43/44 sind gleichzeitig die UART-Konsole des
ESP32-S3 – mit dieser Belegung ist die serielle Debug-Ausgabe (`idf.py
monitor`) zur Laufzeit nicht nutzbar (Flashen funktioniert weiterhin über den
USB-Download-Modus). GPIO19/20 sind außerdem die nativen-USB-Pins – auch
diese Funktion steht dadurch nicht mehr zur Verfügung.

## Anschlussplan

```
                 +------------------+
                 |     ESP32-S3     |
                 | (Waveshare 2.1"  |
                 |  Touch-LCD)      |
                 +--------+---------+
                          |
     ------------------------------------------------
     |               |                              |
     v               v                              v
+---------+     +-----------+                 +-----------+
| Display |     |   Touch   |                 | MCP2515-  |
|ST7701S  |     | (CST820,  |                 | CAN-Modul |
| (RGB)   |     |  I2C)     |                 | (SPI)     |
+---------+     +-----------+                 +-----+-----+
                                                     |
                                          CANH/CANL |
                                                     v
                                          +----------------+
                                          |  OBD2-Stecker  |
                                          |  (BMW E90)     |
                                          |  Pin 6 = CANH  |
                                          |  Pin 14 = CANL |
                                          +----------------+
```

### MCP2515-Modul → CAN-Transceiver → OBD2-Stecker

Nach dem MCP2515 (SPI-Anschluss siehe oben) geht es weiter zum
CAN-Transceiver (meist TJA1050, schon auf dem Modul integriert) und von dort
zum BMW E90 OBD2-Stecker:

| OBD2-Pin | Signal |
|---|---|
| 6 | CANH |
| 14 | CANL |

- Baudrate: **500 kbit/s** (Quarz auf dem MCP2515-Modul muss passen, Standard
  8 MHz)
- Verbunden mit dem PT-CAN (Powertrain-CAN) des Fahrzeugs

## Software / Build

```bash
# Im Projektverzeichnis (ESP-IDF-Environment muss aktiviert sein, z. B. via
# `. $HOME/esp/esp-idf/export.sh`)
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

- **LVGL** wird per Component-Manager (`main/idf_component.yml`) geladen,
  Version 8.2.
- **Flash-Layout**: 16 MB, eigene `partitions.csv` (kein OTA).
- **CI**: `.github/workflows/build-s3.yml` baut das Projekt per
  `espressif/esp-idf-ci-action` (ESP-IDF v5.3.1, Target `esp32s3`) und lädt
  bei Push auf `test_waveshare` `firmware-s3.bin`/`.elf`/`-merged.bin` in das
  GitHub-Release **`s3-latest`**. Merge-Offsets: Bootloader `0x0`,
  Partitionstabelle `0x8000`, App `0x10000`.

## OBD2 / CAN-Auswertung

Läuft über `main/CAN_Driver/` (eigener MCP2515-SPI-Treiber + Decode-Task,
kein ESP-IDF-`twai`). CAN-IDs sind **Platzhalter**, am Fahrzeug per
CAN-Sniffer verifizieren:

| CAN-ID | Inhalt |
|---|---|
| `0x0AA` | Drehzahl (Byte 2–3 LE × 0,25 = U/min) |
| `0x0C4` | G-Kraft (Byte 0/1 int8 × 0,01 = g) + Geschwindigkeit (Byte 2–3 LE × 0,1 = km/h) |
| `0x1D0` | Kühlmitteltemperatur (Byte 0 − 40 = °C) |
| `0x1F0` | Gaspedalstellung (Byte 0 linear 0–255 → 0–100 %) |
| `0x7DF` | OBD2-Funktionsadresse (DTC lesen/löschen, Mode 03/04) |
| `0x611` | Kombiinstrument (CBS-Öl-Service-Reset, UDS Service `0x31`) |

Solange kein CAN-Signal anliegt, zeigt die UI Platzhalterwerte/Testanimation
an. Das Board-Akku-Feld kommt vom internen ADC, nicht von CAN. Die
Fahrzeug-Batteriespannung im Multi-View wird per Mode-01-PID-0x42-Anfrage
(1×/s) live über OBD2 ausgelesen. DTC-Antworten (`0x7E8`, Mode 03) werden zu
Klartext-Codes decodiert (nur Single-Frame-ISO-TP, kein Multi-Frame-
Reassembly).

## UI-Logik (Kurzfassung)

- **Multi-Kachel**: Geschwindigkeit zentral, Ring zeigt Kühlmitteltemperatur
  (Skala 40–119 °C, ab 95 °C Farbwechsel der Nadel), Drehzahl zusätzlich
  digital sichtbar (großer Font). Zusatzfelder: Board-Akku-Spannung (ADC),
  Gaspedal, Drehzahl, Wasser-Temperatur – dazwischen mittig, tiefer versetzt,
  die live per OBD2 (Mode 01 PID 0x42) ausgelesene Fahrzeug-Batteriespannung.
- **Schaltpunkt-Kästchen** (6 Stück, Drehzahl-Kachel): füllen sich einzeln je
  nach Drehzahl (Schwellen 1500/2500/3500/4500/5500/6500 U/min), ab
  6800 U/min blinken alle gemeinsam wie eine digitale Schaltanzeige.
- **Gaspedal-/Drehzahl-/Wasser-Temperatur-Felder** in großem Font mit
  schmaler schwarzer Umrandung für bessere Lesbarkeit.
- **Wisch nach links** auf der Multi-Ansicht öffnet den Fehlercode-Screen:
  oben eine scrollbare Liste der ausgelesenen DTCs (Klartext, z. B. „P0301“),
  darunter „Auslesen“/„Löschen“ nebeneinander sowie „Service Reset“ (CBS-
  Öl-Service). **Wisch nach rechts** führt zurück zur Multi-Ansicht.
- **3 Sekunden Touch in der Bildschirmmitte** zeigt den Waveshare-Demo-Screen,
  ein „Zurück“-Button führt zur BMW-Ansicht zurück.
- **Doppeltipp** in der Mitte öffnet den Farb-Einstellungsbildschirm
  (Primär-/Sekundärfarbe, wirkt sich auf alle Anzeigen und die Nadelfarbe
  aus).

Details zur BMW-Multi-Ansicht siehe `CLAUDE.md` im Repo-Root sowie die
projektspezifischen Notizen unter `2.1 LCD/demo projekt/esp-idf -
ESP32-S3-Touch-LCD-2.1-Test/README.md`.

## Lizenz

GPL-3.0, siehe [LICENSE](LICENSE).
