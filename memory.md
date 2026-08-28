# BMW E90 OBD2 Display – Memory / Projektnotizen

## Status
- v2.0 – Kompletter Hardware-/Software-Wechsel (2026-08-28):
  ESP32-WROOM-32 + MCP2515 + SSD1309 128×64 → **ESP32-C6 + eingebautes TWAI +
  466×466 AMOLED (CO5300/QSPI) mit LVGL**. Analoge Rundinstrumente statt Textanzeige,
  plus Diagnose-Tile (DTC lesen/löschen, CBS-Reset) und SD-Boot-Animation.
- Alte v1.0-Doku (Mode 01/22 PID-Polling für Kühlmittel/Öl/Abgas/Lambda/Klemme15)
  ist mit v2.0 **nicht mehr aktuell** – neue Version pollt nur Kühlmitteltemperatur
  (per CAN-Broadcast) und Batteriespannung (Platzhalter, noch nicht verdrahtet).
- Noch nicht auf Hardware getestet
- Root-Datei `bmw_e90_display.ino` ist ein Überbleibsel der sehr frühen
  MCP2515-Version und stimmt weder mit v1.0 noch v2.0 überein – wird von
  PlatformIO nicht gebaut (nur `src/main.cpp` zählt). Ggf. entfernen.

## Offene Punkte

### Abgastemperatur-DID (hohe Priorität)
Der DID 0xA801 ist ein Startwert – muss mit einem BMW-Diagnosetool (z.B. INPA, BimmerCode, ISTA) verifiziert werden.
Alternativen für N43B20A:
- 0xA801 (aktuell eingestellt)
- 0x4009 (Pre-Kat Bank 1)
- 0x400A (Post-Kat Bank 1)
- 0x1327

Testprozedur: Seriellen Monitor öffnen (`pio device monitor`), auf Timeout-Meldungen für EXHAUST achten.

### IBS / Batteriesensor
Echter IBS-Daten (Strom, genaue Batteriespannung) liegen auf K-CAN (100 kbps).
Aktuell: PID 0x42 = Versorgungsspannung der ECU (~Klemme-30-Spannung) als Näherung.
Für echten IBS: separaten CAN-Bus-Kanal mit 100 kbps anschließen oder K-CAN-Gateway nutzen.

### Lambda-Sensor
PID 0x14 = Schmalband-O2-Sensor Bank 1, Sensor 1 (vor Kat).
Spannung 0–1 V: <0.3 V = mager, >0.7 V = fett, ~0.45 V = Regelbereich.
N43 hat Breitband-Sensor (LSU 4.9) – ggf. PID 0x24 versuchen für präzisere Werte.

## Hardware-Notizen

### MCP2515 Quarz prüfen
Falls CAN init fehlschlägt:
1. Quarz auf Modul ablesen (8 MHz oder 16 MHz)
2. `MCP_CRYSTAL` in main.cpp anpassen
3. CS-Pin prüfen (GPIO 5 korrekt belegt?)

### SSD1309 I2C-Adresse
Standard: 0x3C (SA0-Pin auf GND).
Falls Display nicht reagiert: 0x3D probieren (SA0 auf VCC).
In U8g2-Konstruktor anpassen: `U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(..., 0x3D)`

### Spannungsversorgung
- OBD2 Pin 16 = 12V (Klemme 30, Dauerstrom)
- Spannungsregler (LM2596 o.ä.) 12V → 5V für MCP2515 + ESP32
- ESP32 VIN-Pin verträgt 5V direkt

## Testergebnisse (leer bis Feldtest)

| Sensor | DID/PID | Status |
|---|---|---|
| Kühlmittel | 0x05 | Noch nicht getestet |
| Motoröl | 0x5C | Noch nicht getestet |
| Abgas | 0xA801 | Noch nicht getestet |
| Lambda | 0x14 | Noch nicht getestet |
| Klemme 15 | 0x42 | Noch nicht getestet |

## Nützliche Tools für Diagnose

- **INPA / ISTA** – BMW-eigene Diagnosesoftware (PC)
- **BimmerCode / BimmerLink** – Android-App für BMW OBD2
- **CAN-Bus-Sniffer**: `pio device monitor` zeigt Timeouts im Seriellen Monitor
