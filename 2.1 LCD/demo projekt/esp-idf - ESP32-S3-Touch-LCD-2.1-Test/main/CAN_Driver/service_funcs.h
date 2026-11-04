#pragma once

#include <stdint.h>

// Tabelle der Service-Funktionen fuer den "Service"-Screen (Wisch nach rechts
// auf der Multi-Ansicht). Gemeinsam genutzt von MCP2515- und BLE-OBD-Pfad.
//
// Jeder Eintrag ist eine Diagnose-Anfrage (ohne ISO-TP-Laengenbyte) an ein
// Steuergeraet. len == 0 bedeutet: Payload noch NICHT hinterlegt - der Button
// sendet dann nichts und zeigt "Nicht hinterlegt".
//
// E-Serie (E90) nutzt am OBD-Stecker (D-CAN) BMW-"extended addressing":
// ALLE Anfragen gehen an CAN-ID 0x6F1 (Tester), Byte 0 = Diagnoseadresse
// des Ziel-Steuergeraets (z.B. 0x12 DME, 0x60 Kombi), danach ISO-TP-PCI +
// Daten. Die Antwort kommt auf 0x600 + Adresse (z.B. 0x660), Byte 0 = 0xF1.
// Die frueher hier eingetragene CAN-ID 0x611 war falsch (das waere eine
// Antwort-ID), deshalb kam der Service-Reset nie am Kombi an.
//
// ACHTUNG: Keine der Payloads ist am Fahrzeug verifiziert. Die echten
// Routine-IDs muessen aus einem ISTA-/ENET-Mitschnitt entnommen werden.
#define BMW_TESTER_ID   0x6F1   // Request-CAN-ID fuer alle Steuergeraete
#define BMW_RESP_BASE   0x600   // Antwort-CAN-ID = 0x600 + Steuergeraete-Adresse
#define BMW_ECU_DME     0x12    // Motorsteuergeraet (MSV80)
#define BMW_ECU_KOMBI   0x60    // Kombiinstrument (CBS)

// Kandidaten fuer die Oeltemperatur per Service 0x22 an die DME (0x12).
// 0x4402 stammt aus TomWis97/bmw-obd2-display - das ist aber ein F20 (UDS),
// am E90/MSV80 unverifiziert. 0x12F5/0x202F stammen aus einer Nutzerquelle,
// Formel unbekannt. Der erste Kandidat mit positiver Antwort (0x62) wird
// behalten; alle Antworten landen im SD-Log zum Auswerten.
#define BMW_OIL_DID_COUNT 3
static const uint16_t BMW_OIL_DIDS[BMW_OIL_DID_COUNT] = {0x4402, 0x12F5, 0x202F};

typedef struct {
    const char *label;     // Button-Beschriftung
    uint8_t     ecu;       // BMW-Diagnoseadresse des Ziel-Steuergeraets (0x60 = Kombi)
    uint8_t     len;       // Anzahl UDS-Bytes in data (0 = nicht hinterlegt)
    uint8_t     data[6];
} service_func_t;

static const service_func_t SERVICE_FUNCS[] = {
    // Kombiinstrument (Adresse 0x60), CBS-Routine 0x31 01 FF <Typ>; Typ 0x01 =
    // Motoroel, 0x02 = Bremsbelaege - Routine/Typ weiterhin geraten
    { "Oil reset",                BMW_ECU_KOMBI, 4, {0x31, 0x01, 0xFF, 0x01} },
    { "Brake reset",              BMW_ECU_KOMBI, 4, {0x31, 0x01, 0xFF, 0x02} },
    // DSC-Steuergeraet bzw. DME: Routine-ID unbekannt
    { "Brake Bleed",              0x00, 0, {0} },
    { "NOx Regen",                0x00, 0, {0} },
};
#define SERVICE_FUNC_COUNT ((int)(sizeof(SERVICE_FUNCS) / sizeof(SERVICE_FUNCS[0])))
