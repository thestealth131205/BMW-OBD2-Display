#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialisiert einen BLE-GATT-Client fuer einen ELM327-kompatiblen
// Bluetooth-Low-Energy-OBD2-Adapter (Ziel: Veepeak OBDCheck BLE). Scannt
// dauerhaft im Hintergrund nach einem Geraet, dessen Werbename "OBD",
// "VEEPEAK" oder "VLINK" enthaelt, verbindet sich automatisch (inkl.
// BLE-Pairing-Fallback ueber die Passwoerter 1234/5678/0000, falls das
// Geraet eine PIN verlangt) und pollt danach zyklisch Standard-OBD2-PIDs.
//
// Muss NACH Wireless_Init() aufgerufen werden, da derselbe Bluedroid-Stack
// (BT-Controller + GAP) mitgenutzt wird. Wartet intern, bis der einmalige
// WiFi/BLE-Demo-Scan aus Wireless.c abgeschlossen ist, bevor der GAP-
// Callback uebernommen wird (Bluedroid erlaubt nur einen globalen
// GAP-Callback).
void BLE_OBD_Init(void);

// true, sobald mindestens eine gueltige PID-Antwort ausgewertet wurde.
bool BLE_OBD_online(void);
// Kurzer Verbindungsstatus-Text fuer die Anzeige (Suche/Verbinde/ELM Init/...).
const char *BLE_OBD_status(void);
// Zeitpunkt (ms seit Boot) des letzten gesendeten Kommandos bzw. der letzten
// empfangenen Notification, 0 = noch nie. Fuer die TX/RX-Aktivitaetspunkte.
uint32_t BLE_OBD_last_tx_ms(void);
uint32_t BLE_OBD_last_rx_ms(void);

// --- Live-Werte (Standard-OBD2-PIDs, Mode 01) ---
float BLE_OBD_speed_kmh(void);
float BLE_OBD_rpm(void);
float BLE_OBD_water_temp(void);
float BLE_OBD_throttle_pct(void);

// Steuergeraete-Batteriespannung (ELM327-Kommando "ATRV").
float BLE_OBD_bat_voltage(void);

// Kein eigener Beschleunigungssensor ueber Standard-OBD2-PIDs verfuegbar
// (im Gegensatz zum MCP2515-Pfad, der BMW-spezifische PT-CAN-Broadcasts
// nutzt) - liefert immer 0.0.
float BLE_OBD_gforce_x(void);
float BLE_OBD_gforce_y(void);

// OBD2-Diagnose (funktionale Adresse, ELM327-Kommandos "03"/"04").
void BLE_OBD_read_dtc(void);
void BLE_OBD_clear_dtc(void);
// BMW-CBS-Oel-Service-Reset ueber UDS Routine Control (0x31) an das
// Kombiinstrument (0x611), per ELM327-Header-Umschaltung ("ATSH611").
void BLE_OBD_reset_service_oil(void);

// Anzahl der zuletzt ausgelesenen Fehlercodes. -1 = noch nicht ausgelesen.
int BLE_OBD_dtc_count(void);
// Fehlercode als Text (z.B. "P0301"), idx 0..BLE_OBD_dtc_count()-1.
const char *BLE_OBD_dtc_code(int idx);

#ifdef __cplusplus
}
#endif
