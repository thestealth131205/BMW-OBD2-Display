# BMW E90 OBD2 Display – CLAUDE.md

## Projektübersicht

ESP32-C6-basiertes CAN-Bus-Gauge-Display für einen BMW E90 320i (N43B20A, 2010).
Zeigt Kühlmitteltemperatur, Batteriespannung und Gaspedalstellung als analoge
Tacho-Style-Rundinstrumente (LVGL) mit Farbzonen (Normal/Warnung/Alarm) auf einem
runden 466×466 AMOLED-Touchdisplay an, inkl. Boot-Animation, Diagnose
(DTCs lesen/löschen) und BMW CBS-Service-Reset.

## Hardware

| Komponente | Modell | Schnittstelle |
|---|---|---|
| Mikrocontroller | ESP32-C6 (Waveshare 1.43" AMOLED Touch) | – |
| Display | CO5300 AMOLED, 466×466, rund | QSPI (CS=6, SCK=10, D0-D3=0-3, RST=7) |
| Touch | kapazitiv (LVGL Input-Device) | – |
| CAN-Bus | eingebauter ESP32-C6 TWAI-Controller + externer CAN-Transceiver | TX=GPIO19, RX=GPIO20 |
| microSD | Boot-Animation (JPEG-Frames) | SPI (CS=GPIO13) |

Es wird **kein** MCP2515 mehr verwendet – der ESP32-C6 hat einen eingebauten
TWAI-CAN-Controller (`driver/twai.h`), der nur noch einen externen CAN-Transceiver
(z. B. TJA1050/SN65HVD230) zwischen den GPIOs und dem OBD2-Stecker benötigt.

### CAN-Bus
BMW E90 PT-CAN (Powertrain-CAN) an OBD2-Stecker:
- Pin 6  → CANH (Transceiver)
- Pin 14 → CANL (Transceiver)
- Baudrate: 500 kbps

## Software / Build

**Framework:** Arduino via PlatformIO

```bash
# Build
pio run

# Flashen
pio run --target upload

# Serielle Ausgabe (Debugging)
pio device monitor
```

### Abhängigkeiten (`platformio.ini`)
- `moononournation/GFX Library for Arduino` – Display-Treiber (Arduino_CO5300 / QSPI)
- `lvgl/lvgl` (v8) – UI-Framework (Meter, Tileview, Labels, Buttons)
- `bodmer/TJpg_Decoder` – JPEG-Dekodierung für die Boot-Animation
- ESP32-Arduino-Core: `SD.h` (microSD) und `driver/twai.h` (CAN) sind bereits im Core enthalten

### LVGL-Konfiguration
`include/lv_conf.h` enthält eine minimale LVGL-v8-Konfiguration (Farbtiefe 16 bit,
`LV_TICK_CUSTOM` über `millis()`, aktivierte Widgets: Meter, Tileview, Label, Button).
Nicht gesetzte Optionen fallen auf die LVGL-Standardwerte zurück.

**Wichtig:** Der ESP32-C6 benötigt Arduino-ESP32-Core 3.x (ESP-IDF 5.x). Falls die
offizielle PlatformIO-`espressif32`-Plattform kein `esp32-c6-devkitc-1`-Board mit
ausreichend aktuellem Core bereitstellt, den `pioarduino`-Community-Fork der Plattform
verwenden (siehe Kommentar in `platformio.ini`).

## Projektstruktur

```
BMW E90 OBD2 Display/
├── src/
│   └── main.cpp          ← Hauptprogramm (alle Logik hier)
├── include/
│   └── lv_conf.h          ← LVGL-Konfiguration
├── platformio.ini        ← Build-Konfiguration
├── CLAUDE.md              ← Diese Datei
└── memory.md              ← Projektnotizen / Fortschritt
```

microSD-Karte für die Boot-Animation (Ordnerstruktur, JPEG-Frames):
```
/anim_<brand>/frame_000.jpg ... frame_149.jpg   (150 Frames, 15 FPS)
```
`<brand>` entspricht `currentSettings.brand` (Standard: `"bmw"`).

## UI / Anzeige-Logik

- **Tileview mit 3 Kacheln** (horizontales Wischen), erzeugt über den gemeinsamen
  Helper `createStyledMeter()` (Skalenstriche, 3 Farbzonen-Bögen, Nadel, große
  digitale Anzeige im Zentrum, kleines Sekundär-Label darunter – Tacho-Style,
  dunkler Hintergrund via `setDarkBg()`):
  1. **Wasser-Kachel** – Rundinstrument Kühlmitteltemperatur (40–130 °C),
     Sekundärwert darunter: aktuelle Batteriespannung
  2. **Batterie-Kachel** – Rundinstrument Batteriespannung (10.0–16.0 V),
     Sekundärwert darunter: aktuelle Kühlmitteltemperatur
  3. **Gaspedal-Kachel** – Rundinstrument Gaspedalstellung (0–100 %),
     Sekundärwert darunter: aktuelle Kühlmitteltemperatur
- **Fehlercode-Übersicht** – eigener Screen (per Doppeltipp erreichbar), DTCs
  auslesen/löschen, CBS-Öl-Service-Reset, „Zurück"-Button
- **Farbverlauf der Nadel/Anzeige:** Primärfarbe → Gelb (10 % vor Warnschwelle) →
  Rot (Warn- bis Alarmschwelle), oberhalb der Alarmschwelle blinkt der Wert rot/schwarz
  (250 ms) – Logik zentral in `computeGaugeColor()`. Bei der Batterie ist die Richtung
  umgekehrt (niedrige Spannung = kritisch, `invert_zones=true` in `createStyledMeter()`)
- **3 Sekunden Touch in der Displaymitte** öffnet den Einstellungs-Screen
- **Einstellungs-Screen** – eigenes Tileview mit 3 Kacheln (Wasser/Batterie/Gaspedal),
  je Kachel per +/- Buttons verstellbar: **Warnschwelle** (Schwelle 1) und
  **Limit/Alarmschwelle** (Schwelle 2), über `createSettingsTile()` /
  `createThresholdRow()` gebaut. „Speichern & Beenden"-Button liegt als Overlay
  über dem Tileview und schreibt direkt in `currentSettings`
- **Doppeltipp** auf dem Hauptbildschirm öffnet die Fehlercode-Übersicht
- **Boot-Animation** aus JPEG-Frames von der SD-Karte läuft vor der LVGL-Initialisierung

## OBD2 / UDS-Protokoll

- Anfrage an `0x7DF` (Broadcast / funktionale Adresse) für Standard-OBD2 (Mode 03/04)
- Fehlercodes lesen: Mode 03 (`0x02 0x03 ...`)
- Fehlercodes löschen: Mode 04 (`0x01 0x04 ...`)
- CBS-Service-Reset: Routine Control (UDS Service `0x31`) an Kombiinstrument `0x611`
- Kühlmitteltemperatur wird aktuell aus einem PT-CAN-Broadcast (ID `0x1D0`, Byte 0 − 40 = °C)
  gelesen, **nicht** per Mode-01-Polling – ID muss ggf. mit Diagnosetool verifiziert werden
- Gaspedalstellung wird aus einem PT-CAN-Broadcast (ID `0x1F0`, Byte 0 linear auf 0–100 %
  skaliert) gelesen – **Platzhalter-ID**, muss am Fahrzeug per CAN-Sniffer/Diagnosetool
  verifiziert werden

## Bekannte Einschränkungen / offene Punkte

- CAN-ID `0x1D0` für Kühlmitteltemperatur ist ein Startwert und muss am Fahrzeug
  per CAN-Sniffer/Diagnosetool verifiziert werden
- CAN-ID `0x1F0` für die Gaspedalstellung ist ebenfalls ein Startwert/Platzhalter
  und muss am Fahrzeug verifiziert werden
- Batteriespannung (`current_bat_voltage`) wird aktuell noch nicht per CAN aktualisiert
  (nur Platzhalter-Initialwert) – Auswertung in `processCAN()` ergänzen
- DTC-Antworten werden nur gesendet, aber die Antwort-Frames (`0x7E8`) werden noch
  nicht ausgewertet/decodiert (nur "Request gesendet" wird angezeigt)
- Einstellungs-Screen speichert aktuell keine Werte dauerhaft (kein NVS/Preferences)
- CO5300/QSPI-Pinbelegung und Touch-Treiber sind board-spezifisch für das
  Waveshare-1.43"-AMOLED-Modul – bei anderer Platine anpassen

## Fahrzeug

| Merkmal | Wert |
|---|---|
| Modell | BMW E90 320i |
| Motor | N43B20A |
| Baujahr | 2010 |
| Motorsteuergerät | Siemens/Continental MSV80 |
