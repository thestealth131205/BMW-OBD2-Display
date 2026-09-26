#pragma once

#include <stdint.h>

// Tabelle der Service-Funktionen fuer den "Service"-Screen (Wisch nach rechts
// auf der Multi-Ansicht). Gemeinsam genutzt von MCP2515- und BLE-OBD-Pfad.
//
// Jeder Eintrag ist eine UDS-Anfrage (ohne ISO-TP-Laengenbyte) an ein
// Steuergeraet. len == 0 bedeutet: Payload noch NICHT hinterlegt - der Button
// sendet dann nichts und zeigt "Nicht hinterlegt".
//
// ACHTUNG: Keine der Payloads ist am Fahrzeug verifiziert. Die echten
// Routine-IDs muessen aus einem ISTA-/ENET-Mitschnitt entnommen werden.
typedef struct {
    const char *label;     // Button-Beschriftung
    uint16_t    can_id;    // 11-Bit-Request-ID des Ziel-Steuergeraets
    uint8_t     len;       // Anzahl UDS-Bytes in data (0 = nicht hinterlegt)
    uint8_t     data[6];
} service_func_t;

static const service_func_t SERVICE_FUNCS[] = {
    // Kombiinstrument (0x611), CBS-Routine 0x31 01 FF <Typ>; Typ 0x01 = Motoroel
    // (wie bisheriger Service-Reset), 0x02 = Bremsbelaege - Typ geraten
    { "Oel Service Reset",       0x611, 4, {0x31, 0x01, 0xFF, 0x01} },
    { "Bremsen Verschleiss",     0x611, 4, {0x31, 0x01, 0xFF, 0x02} },
    // DSC-Steuergeraet bzw. DME: Routine-ID unbekannt
    { "Bremsen entlueften",      0x000, 0, {0} },
    { "NOx Regeneration",        0x000, 0, {0} },
};
#define SERVICE_FUNC_COUNT ((int)(sizeof(SERVICE_FUNCS) / sizeof(SERVICE_FUNCS[0])))
