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
// BMW-CBS-Oel-Service-Reset. Senden nur; Antwort-Auswertung folgt.
void CAN_OBD2_read_dtc(void);
void CAN_OBD2_clear_dtc(void);
void CAN_OBD2_reset_service_oil(void);

#ifdef __cplusplus
}
#endif
