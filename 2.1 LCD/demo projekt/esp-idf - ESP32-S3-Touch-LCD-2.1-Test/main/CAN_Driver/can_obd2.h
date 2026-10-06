#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Startet den MCP2515 und einen Hintergrund-Task, der PT-CAN-Broadcasts
// mitliest und die Live-Werte (Geschwindigkeit, Drehzahl, Wasser, Gaspedal,
// G-Kraft) dekodiert. Gibt false zurueck, wenn der MCP2515 nicht antwortet
// (dann bleiben die Werte auf Platzhaltern / CAN_OBD2_online() = false).
bool CAN_OBD2_Init(void);

// true sobald mindestens ein gueltiger CAN-Frame empfangen wurde.
bool CAN_OBD2_online(void);

// --- Live-Werte (thread-safe genug fuer einzelne floats/ints auf 32-bit) ---
float CAN_OBD2_speed_kmh(void);
float CAN_OBD2_rpm(void);
float CAN_OBD2_water_temp(void);
float CAN_OBD2_throttle_pct(void);
float CAN_OBD2_gforce_x(void);
float CAN_OBD2_gforce_y(void);

// OBD2-Diagnose (funktionale Adresse 0x7DF): Fehlercodes lesen/loeschen,
// BMW-CBS-Oel-Service-Reset.
void CAN_OBD2_read_dtc(void);
void CAN_OBD2_clear_dtc(void);
void CAN_OBD2_reset_service_oil(void);
// Fuehrt Eintrag idx aus SERVICE_FUNCS[] (service_funcs.h) aus.
// Liefert false bei ungueltigem Index oder nicht hinterlegter Payload.
bool CAN_OBD2_service_func(int idx);

// Anzahl der zuletzt ausgelesenen Fehlercodes. -1 = noch nicht ausgelesen
// (seit Boot bzw. seit dem letzten CAN_OBD2_read_dtc()-Aufruf noch keine
// Mode-03-Antwort (0x7E8) erhalten), 0 = ausgelesen, keine Fehler.
int CAN_OBD2_dtc_count(void);

// Fehlercode als Text (z.B. "P0301"), idx 0..CAN_OBD2_dtc_count()-1.
// Liefert NULL bei ungueltigem Index.
const char *CAN_OBD2_dtc_code(int idx);

// Batteriespannung per OBD2 (Mode 01, PID 0x42 - Steuergeraete-Spannung),
// wird zyklisch abgefragt (1x/Sekunde). 0.0 solange keine Antwort da war.
float CAN_OBD2_bat_voltage(void);

// --- Erweiterte Sensorwerte fuer den Sensoren-Screen (Standard-Mode-01-PIDs
// 0x24/0x25/0x0B, im Rundlauf mit PID 0x42 abgefragt, siehe can_task()).
// Lambda = Kraftstoff-Luft-Aequivalenzverhaeltnis (1.0 = stoechiometrisch),
// kein Rohspannungswert. "Lambda1"/"Lambda2" ist eine Annahme: Standard-OBD2
// nummeriert O2-Sensoren 1-8 (PID 0x24-0x2B) durchlaufend, die Zuordnung zu
// Bank1/Bank2 ist fahrzeugabhaengig - beim N43 (Reihenmotor, nur eine Bank)
// sind das vermutlich Sensor1 (vor Kat) und Sensor2 (nach Kat) derselben
// Bank, nicht zwei getrennte Baenke. Am Fahrzeug noch zu verifizieren. ---
float CAN_OBD2_lambda1_ratio(void);
float CAN_OBD2_lambda1_voltage(void);
float CAN_OBD2_lambda2_ratio(void);
float CAN_OBD2_lambda2_voltage(void);
// Ansaugkruemmerdruck (MAP), PID 0x0B, kPa.
float CAN_OBD2_intake_pressure(void);
// Ladedruck (Turbo): Der N43B20A ist laut CLAUDE.md ein Saugmotor (kein
// Turbo), zudem gibt es dafuer keine standardisierte Mode-01-PID - liefert
// immer -1.0 (= "n/v" in der UI), es wird nichts geraten/abgefragt.
float CAN_OBD2_boost_pressure(void);
// Nockenwellen-Position Einlass/Auslass (VANOS-Winkel): kein Standard-OBD2-
// PID, nur per BMW-spezifischem UDS-Identifier auslesbar (nicht verifiziert,
// nicht implementiert) - liefert immer -1.0 (= "n/v" in der UI).
float CAN_OBD2_cam_intake_pos(void);
float CAN_OBD2_cam_exhaust_pos(void);

#ifdef __cplusplus
}
#endif
