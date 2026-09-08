# BMW E90 OBD2 Display – ESP32-S3-Touch-LCD-2.1 (RGB, ESP-IDF)

Portierung der BMW-E90-Multi-Anzeige (siehe Haupt-`CLAUDE.md` im Repo-Root) auf
das Waveshare-Board **ESP32-S3-Touch-LCD-2.1** (480×480 ST7701S RGB-Display,
CST820-Touch). Framework: **ESP-IDF** (nicht Arduino/PlatformIO), LVGL v8.2
via ESP Component Manager. Basis ist das offizielle Waveshare-Demo-Projekt,
darauf aufgebaut liegt die BMW-UI in `main/BMW_UI/`.

Der eingebaute Waveshare-Demo-Screen bleibt erreichbar: **3 Sekunden Touch in
der Bildschirmmitte** zeigt ihn an, ein „Zurück“-Button führt zur BMW-Ansicht
zurück. **Doppeltipp** in der Mitte öffnet den Farb-Einstellungsbildschirm.

## Hardware

| Komponente | Modell |
|---|---|
| Mikrocontroller | ESP32-S3 (Waveshare ESP32-S3-Touch-LCD-2.1) |
| Display | ST7701S RGB-TFT, 480×480, über SPI-Init + paralleles RGB-Interface |
| Touch | CST820 (kapazitiv), I2C |
| IO-Expander | TCA9554 (Adresse 0x20) – steuert u. a. Touch-Reset/weitere Enable-Leitungen |
| CAN-Bus | externes **MCP2515-Modul** (SPI, TJA1050-Transceiver) |
| Batteriemessung | interner ADC (Spannungsteiler auf dem Board) |

Board-eigene Peripherie (RTC PCF85063, IMU QMI8658, microSD, WLAN) ist aus dem
Waveshare-Demo übernommen und unverändert nutzbar, wird von der BMW-UI aber
nicht benötigt.

## Verdrahtungsplan

### ST7701S-Display (fest verlötet, keine externe Verdrahtung nötig)

Das Display ist bereits fest mit dem ESP32-S3-Modul verlötet (Waveshare-
Compound-Board). Pin-Zuordnung nur zur Referenz (`main/LCD_Driver/ST7701S.h`):

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

### MCP2515-CAN-Modul (extern anzuschließen – einzige Verdrahtung, die der
Nutzer selbst vornehmen muss)

**Wichtig:** Auf dem ESP32-S3-Touch-LCD-2.1 sind fast alle GPIOs durch das
RGB-Display und I2C belegt. Nur **GPIO 0, 19, 20, 43, 44** sind frei
herausgeführt. Verdrahtung des MCP2515-Moduls (blaue Platine mit TJA1050,
`main/CAN_Driver/mcp2515.h`):

| MCP2515-Modul | ESP32-S3 GPIO | Funktion |
|---|---|---|
| VCC | 3V3 | Versorgung |
| GND | GND | Masse |
| SCK | GPIO43 | SPI-Takt |
| SI (MOSI) | GPIO44 | SPI Data ESP→MCP2515 |
| SO (MISO) | GPIO19 | SPI Data MCP2515→ESP |
| CS | GPIO20 | SPI Chip-Select |
| INT | GPIO0 | Interrupt (Polling-Fallback möglich) |

Danach MCP2515 → CAN-Transceiver (TJA1050 ist meist schon auf dem Modul
integriert) → BMW E90 OBD2-Stecker:

| OBD2-Pin | Signal |
|---|---|
| Pin 6 | CANH |
| Pin 14 | CANL |

Baudrate: **500 kbit/s**. Quarz auf dem MCP2515-Modul muss zu
`MCP2515_XTAL_MHZ` in `mcp2515.h` passen (Standard: 8 MHz – am Modul
nachprüfen, sonst stimmt die Baudrate nicht).

**Achtung Pin-Konflikt:** GPIO43/44 sind gleichzeitig die UART-Konsole des
ESP32-S3. Mit dieser Pinbelegung ist die serielle Debug-Ausgabe (`idf.py
monitor`) zur Laufzeit nicht nutzbar (Flashen funktioniert weiterhin über den
USB-Download-Modus). GPIO19/20 sind außerdem die nativen-USB-Pins – auch diese
Funktion steht dadurch nicht mehr zur Verfügung.

## Software / Build

```bash
# Im Projektverzeichnis (ESP-IDF-Environment muss aktiviert sein, z. B. via
# `. $HOME/esp/esp-idf/export.sh`)
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

- **LVGL** wird per Component-Manager (`main/idf_component.yml`) geladen,
  Version 8.2. Der vendorte Ordner `components/lvgl__lvgl/` wird von CI/Build
  automatisch nachgezogen und ist per `.gitignore` vom Repo ausgeschlossen.
- **Flash-Layout**: 16 MB, eigene `partitions.csv` (kein OTA).
- **CI**: `.github/workflows/build-s3.yml` baut das Projekt per
  `espressif/esp-idf-ci-action` (ESP-IDF v5.3.1, Target `esp32s3`) und lädt
  bei Push auf `test_waveshare` `firmware-s3.bin`/`.elf`/`-merged.bin` in das
  GitHub-Release **`s3-latest`**. Merge-Offsets: Bootloader `0x0`,
  Partitionstabelle `0x8000`, App `0x10000`.

## OBD2 / CAN-Auswertung

Läuft über `main/CAN_Driver/` (eigener MCP2515-SPI-Treiber +
Decode-Task, kein ESP-IDF-`twai`). CAN-IDs identisch zum C6-Hauptprojekt –
**Platzhalter**, am Fahrzeug per CAN-Sniffer verifizieren:

| CAN-ID | Inhalt |
|---|---|
| `0x0AA` | Drehzahl (Byte 2–3 LE × 0,25 = U/min) |
| `0x0C4` | G-Kraft (Byte 0/1 int8 × 0,01 = g) + Geschwindigkeit (Byte 2–3 LE × 0,1 = km/h) |
| `0x1D0` | Kühlmitteltemperatur (Byte 0 − 40 = °C) |
| `0x1F0` | Gaspedalstellung (Byte 0 linear 0–255 → 0–100 %) |
| `0x7DF` | OBD2-Funktionsadresse (DTC lesen/löschen, Mode 03/04) |
| `0x611` | Kombiinstrument (CBS-Öl-Service-Reset, UDS Service `0x31`) |

Solange kein CAN-Signal anliegt (`CAN_OBD2_online()` == false), zeigt die UI
Platzhalterwerte/Testanimation an. Batteriespannung kommt weiterhin vom
internen ADC (`BAT_Driver`), nicht von CAN.

## UI-Logik (Kurzfassung)

Details zur BMW-Multi-Ansicht (Anzeigen, Farbzonen, Schwellenwerte, Nadeln,
Einstellungs-Screen) siehe Haupt-`CLAUDE.md` im Repo-Root – die Logik ist
weitgehend 1:1 vom C6-Projekt übernommen (`main/BMW_UI/bmw_ui.c`). Abweichungen
zum C6-Projekt:

- Ringnadel der Multi-Kachel zeigt Kühlmitteltemperatur (Skala 40–119 °C,
  ab 95 °C Farbwechsel), Drehzahl bleibt zusätzlich digital sichtbar.
- Schaltpunkt-Kästchen (6 Stück) füllen sich einzeln je nach Drehzahl
  (aktuelle Schwellen: 1500/2500/3500/4500/5500/6500 U/min), ab 6800 U/min
  blinken alle gemeinsam.
- Farbverwaltung/Nadel-Auswahl per Einstellungs-Screen (Doppeltipp), analog
  zur Farbpalette im C6-Projekt.
