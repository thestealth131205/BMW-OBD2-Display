#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_gap_ble_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// GAP-Events von Wireless.c (dem einzigen registrierten GAP-Callback) hier einspeisen.
void BLE_OBD_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// Initialisiert einen BLE-GATT-Client fuer einen ELM327-kompatiblen
// Bluetooth-Low-Energy-OBD2-Adapter (Ziel: Veepeak OBDCheck BLE). Scannt
// dauerhaft im Hintergrund nach einem Geraet, dessen Werbename "OBD",
// "VEEPEAK" oder "VLINK" enthaelt, verbindet sich automatisch (inkl.
// BLE-Pairing-Fallback ueber die Passwoerter 1234/5678/0000, falls das
// Geraet eine PIN verlangt) und pollt danach zyklisch Standard-OBD2-PIDs.
//
// Muss NACH Wireless_Init() aufgerufen werden, da derselbe Bluedroid-Stack
// (BT-Controller + GAP) mitgenutzt wird. Wartet intern nur, bis der
// Bluedroid-Stack aus Wireless.c bereit ist (BLE_Stack_Ready), sodass die
// Suche parallel zur Start-Animation laeuft. GAP-Events kommen ueber
// BLE_OBD_gap_event() aus Wireless.c (Bluedroid erlaubt nur einen globalen
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

// --- Erweiterte Sensorwerte fuer den Sensoren-Screen (Standard-Mode-01-PIDs
// 0x24/0x25/0x0B, im Rundlauf mit den anderen Live-Werten abgefragt, siehe
// ble_obd_task()). Lambda = Kraftstoff-Luft-Aequivalenzverhaeltnis (1.0 =
// stoechiometrisch), kein Rohspannungswert. "Lambda1"/"Lambda2" ist eine
// Annahme: Standard-OBD2 nummeriert O2-Sensoren 1-8 (PID 0x24-0x2B)
// durchlaufend, die Zuordnung zu Bank1/Bank2 ist fahrzeugabhaengig - beim N43
// (Reihenmotor, nur eine Bank) sind das vermutlich Sensor1 (vor Kat) und
// Sensor2 (nach Kat) derselben Bank, nicht zwei getrennte Baenke. Am
// Fahrzeug noch zu verifizieren.
//
// Die drei PIDs werden nur abgefragt, solange der Sensoren-Screen sichtbar
// ist (true/false hier vom UI beim Rein-/Rausswipen gesetzt) - sonst bleibt
// mehr Zeit im Poll-Rundlauf fuer RPM/Speed/Wasser/Gaspedal, und ohne
// aktiven Trigger kaemen hier sonst nie Werte an. ---
void BLE_OBD_set_sensors_active(bool active);
float BLE_OBD_lambda1_ratio(void);
float BLE_OBD_lambda1_voltage(void);
float BLE_OBD_lambda2_ratio(void);
float BLE_OBD_lambda2_voltage(void);
// Ansaugkruemmerdruck (MAP), PID 0x0B, kPa.
float BLE_OBD_intake_pressure(void);
// Ladedruck (Turbo): Der N43B20A ist laut CLAUDE.md ein Saugmotor (kein
// Turbo), zudem gibt es dafuer keine standardisierte Mode-01-PID - liefert
// immer -1.0 (= "n/v" in der UI), es wird nichts geraten/abgefragt.
float BLE_OBD_boost_pressure(void);
// Nockenwellen-Position Einlass/Auslass (VANOS-Winkel): kein Standard-OBD2-
// PID, nur per BMW-spezifischem UDS-Identifier auslesbar (nicht verifiziert,
// nicht implementiert) - liefert immer -1.0 (= "n/v" in der UI).
float BLE_OBD_cam_intake_pos(void);
float BLE_OBD_cam_exhaust_pos(void);

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
// Fuehrt Eintrag idx aus SERVICE_FUNCS[] (service_funcs.h) aus.
// Liefert false bei ungueltigem Index oder nicht hinterlegter Payload.
bool BLE_OBD_service_func(int idx);

// Anzahl der zuletzt ausgelesenen Fehlercodes. -1 = noch nicht ausgelesen.
int BLE_OBD_dtc_count(void);
// Fehlercode als Text (z.B. "P0301"), idx 0..BLE_OBD_dtc_count()-1.
const char *BLE_OBD_dtc_code(int idx);

// Stoppt Scan/Verbindung voruebergehend (kein automatisches Wiederverbinden,
// bis BLE_OBD_Resume() aufgerufen wird). Gedacht fuer den WLAN-Update-Modus:
// WiFi-AP und eine aktiv pollende BLE-Verbindung (alle 20ms ein Kommando)
// teilen sich auf dem ESP32-S3 dieselbe 2,4-GHz-Antenne (Software-Koexistenz)
// - bei hoher BLE-Last kann esp_wifi_start() dadurch extrem lange haengen
// bzw. nie fertig werden. Mehrfachaufruf ist unschaedlich (no-op).
void BLE_OBD_Suspend(void);
// Nimmt die Suche nach dem Adapter wieder auf (nach BLE_OBD_Suspend()).
void BLE_OBD_Resume(void);

#ifdef __cplusplus
}
#endif
