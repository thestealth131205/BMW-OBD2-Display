#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- Verdrahtung MCP2515 <-> ESP32-S3 (nur diese GPIOs sind am Waveshare
//     ESP32-S3-Touch-LCD-2.1 frei herausgefuehrt: 0/19/20/43/44) ---
//     MCP2515-Modul (blaue Platine, TJA1050) an diese Pins loeten/stecken.
#ifndef MCP2515_PIN_SCK
#define MCP2515_PIN_SCK   43   // MCP2515 SCK
#endif
#ifndef MCP2515_PIN_MOSI
#define MCP2515_PIN_MOSI  44   // MCP2515 SI  (MOSI)
#endif
#ifndef MCP2515_PIN_MISO
#define MCP2515_PIN_MISO  19   // MCP2515 SO  (MISO)
#endif
#ifndef MCP2515_PIN_CS
#define MCP2515_PIN_CS    20   // MCP2515 CS
#endif
// -1 = nicht verbunden (Polling): mcp2515_receive() fragt den Empfangsstatus
// ohnehin ueber das CANINTF-Register per SPI ab (reiner Software-Poll), der
// INT-Pin wurde nie per gpio_get_level()/ISR tatsaechlich ausgewertet - daher
// bleibt GPIO0 jetzt fest frei fuer LED-Streifen 1 (siehe LED_Control/
// led_ctrl.c), unabhaengig von der BLE/MCP-Datenquellenwahl. Hardware: ein
// evtl. am MCP2515-Modul angeschlossener INT-Draht muss fuer den
// Streifen-1-Betrieb abgezogen werden, die anderen vier Leitungen bleiben
// unveraendert.
#ifndef MCP2515_PIN_INT
#define MCP2515_PIN_INT   -1
#endif

// LED-Streifen 2 (led_ctrl.c, LED_Ctrl2_*) nutzt GPIO19 - exakt
// MCP2515_PIN_MISO. SPI braucht alle 4 Leitungen (SCK/MOSI/MISO/CS), ein
// fehlendes MISO macht das MCP2515-Modul komplett funktionsunfaehig - daher
// schliessen sich Streifen 2 und das (verkabelte) MCP2515-Modul gegenseitig
// aus. Wird Streifen 2 benutzt, muss die Datenquelle auf BLE-OBD stehen und
// das MCP2515-Modul unverbunden bleiben (siehe docs/led_wiring.png).

// Quarz auf dem MCP2515-Modul: 8 oder 16 (MHz). Muss zum Aufdruck passen,
// sonst stimmt die CAN-Baudrate nicht.
#ifndef MCP2515_XTAL_MHZ
#define MCP2515_XTAL_MHZ  8
#endif

// Ein empfangener CAN-Frame.
typedef struct {
    uint32_t id;       // 11-Bit Standard-ID (BMW PT-CAN)
    uint8_t  dlc;      // Datenlaenge 0..8
    uint8_t  data[8];
} mcp2515_frame_t;

// Initialisiert SPI-Bus + MCP2515 auf 500 kbit/s, Accept-All-Filter, Normal-Mode.
esp_err_t mcp2515_init(void);

// Liest den naechsten wartenden CAN-Frame. true = Frame in *out, false = keiner da.
bool mcp2515_receive(mcp2515_frame_t *out);

// Sendet einen Standard-CAN-Frame (fuer OBD2-Requests, Mode 03/04, CBS-Reset).
esp_err_t mcp2515_send(uint32_t id, const uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif
