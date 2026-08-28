#pragma once

#include <Arduino_GFX_Library.h>

// SH8601-AMOLED-Treiber (Waveshare ESP32-C6 1.43" AMOLED Touch, CO5300 ist
// hier NICHT der richtige Chip). Das QSPI-Kommandoprotokoll ist identisch zu
// Arduino_CO5300 (Opcode 0x02 + Kommando<<8 als "Adresse", siehe
// Arduino_ESP32QSPI::writeC8D8) - übernommen von dort. Nur die Init-Sequenz
// ist SH8601-spezifisch und 1:1 aus dem offiziellen Waveshare-SDK
// (Example/Arduino/11_FactoryProgram/src/port_bsp/display_bsp.cpp,
// esp_lcd_sh8601 vendor init) übernommen.
class Arduino_SH8601 : public Arduino_CO5300
{
public:
  Arduino_SH8601(Arduino_DataBus *bus, int8_t rst, uint8_t r, int16_t w, int16_t h)
      : Arduino_CO5300(bus, rst, r, w, h) {}

protected:
  void tftInit() override
  {
    if (_rst != GFX_NOT_DEFINED)
    {
      pinMode(_rst, OUTPUT);
      digitalWrite(_rst, HIGH);
      delay(10);
      digitalWrite(_rst, LOW);
      delay(150);
      digitalWrite(_rst, HIGH);
      delay(150);
    }
    else
    {
      _bus->sendCommand(0x01); // Software-Reset
      delay(80);
    }

    _bus->beginWrite();
    _bus->writeC8D8(0x36, 0x00); // MADCTL: RGB-Reihenfolge
    _bus->writeC8D8(0x3A, 0x55); // COLMOD: 16 bpp RGB565
    _bus->writeCommand(0x11);    // Sleep Out
    _bus->endWrite();
    delay(80);

    _bus->beginWrite();
    _bus->writeC8D8(0xC4, 0x80); // SPI-Modus-Steuerung
    _bus->writeC8D8(0x53, 0x20); // CTRL Display: Helligkeitssteuerung an
    delay(1);
    _bus->writeC8D8(0x63, 0xFF); // Helligkeit HBM-Modus
    delay(1);
    _bus->writeC8D8(0x51, 0x00); // Helligkeit Normal-Modus = 0 (dunkel starten)
    delay(1);
    _bus->writeCommand(0x29);    // Display On
    delay(10);
    _bus->writeC8D8(0x51, 0xFF); // Helligkeit Normal-Modus = max
    _bus->endWrite();
  }
};
