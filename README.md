# BMW E90 OBD2 Display

ESP32-C6-basiertes CAN-Bus-Gauge-Display für einen BMW E90 320i (N43B20A, 2010).
Zeigt Kühlmitteltemperatur und Batteriespannung als analoge Rundinstrumente (LVGL)
auf einem runden 466×466 AMOLED-Touchdisplay an, inkl. Boot-Animation, Diagnose
(DTCs lesen/löschen) und BMW CBS-Service-Reset.

## Hardware

| Komponente | Modell |
|---|---|
| Mikrocontroller | ESP32-C6 (Waveshare 1.43" AMOLED Touch) |
| Display | CO5300 AMOLED, 466×466, rund, QSPI |
| Touch | Kapazitiv (integriert im Display-Modul) |
| CAN-Transceiver | z. B. TJA1050 / SN65HVD230 |
| microSD-Karte | Boot-Animation (JPEG-Frames) |

Es wird **kein** MCP2515 verwendet – der ESP32-C6 besitzt einen eingebauten
TWAI-CAN-Controller (`driver/twai.h`). Es wird nur noch ein externer
CAN-Transceiver zwischen den GPIOs und dem OBD2-Stecker benötigt.

## Pinbelegung

### Display (CO5300, QSPI)

| Signal | ESP32-C6 GPIO |
|---|---|
| CS | GPIO6 |
| SCK | GPIO10 |
| D0 | GPIO0 |
| D1 | GPIO1 |
| D2 | GPIO2 |
| D3 | GPIO3 |
| RST | GPIO7 |

### microSD-Karte (SPI)

| Signal | ESP32-C6 GPIO |
|---|---|
| CS | GPIO13 |

Die restlichen SPI-Signale (MOSI/MISO/SCK) laufen über den Standard-`SPI`-Bus
des Boards (Waveshare-1.43"-AMOLED-Modul).

### CAN-Bus (TWAI, eingebauter Controller)

| Signal | ESP32-C6 GPIO |
|---|---|
| TX (zum Transceiver) | GPIO19 |
| RX (vom Transceiver) | GPIO20 |

## Anschlussplan

```
                 +------------------+
                 |     ESP32-C6     |
                 |  (Waveshare 1.43"|
                 |   AMOLED Touch)  |
                 +--------+---------+
                          |
     ------------------------------------------------
     |               |                |             |
     v               v                v             v
+---------+     +-----------+   +-----------+  +-----------+
| Display |     | microSD   |   |   CAN-    |  |   Touch   |
| CO5300  |     |   Karte   |   |Transceiver|  | (im       |
| (QSPI)  |     |  (SPI)    |   |(TJA1050/  |  | Display-  |
|         |     |           |   | SN65HVD230)| | Modul)   |
+---------+     +-----------+   +-----+-----+  +-----------+
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

### CAN-Transceiver-Verkabelung

| CAN-Transceiver-Pin | Verbindung |
|---|---|
| TXD | ESP32-C6 GPIO19 |
| RXD | ESP32-C6 GPIO20 |
| VCC | 3.3V oder 5V (je nach Transceiver-Modul) |
| GND | GND (gemeinsame Masse mit ESP32-C6 und OBD2-Stecker) |
| CANH | OBD2-Stecker Pin 6 |
| CANL | OBD2-Stecker Pin 14 |

### OBD2-Stecker (BMW E90, PT-CAN)

| OBD2-Pin | Signal |
|---|---|
| 6 | CANH |
| 14 | CANL |
| 16 | +12V (Dauerplus) |
| 4/5 | GND |

- Baudrate: **500 kbps**
- Verbunden mit dem PT-CAN (Powertrain-CAN) des Fahrzeugs

## Software / Build

Framework: Arduino via PlatformIO

```bash
# Build
pio run

# Flashen
pio run --target upload

# Serielle Ausgabe (Debugging)
pio device monitor
```

Details zu Abhängigkeiten, LVGL-Konfiguration und Projektstruktur siehe [CLAUDE.md](CLAUDE.md).

## Lizenz

GPL-3.0, siehe [LICENSE](LICENSE).
