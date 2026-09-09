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

#ifdef __cplusplus
}
#endif
