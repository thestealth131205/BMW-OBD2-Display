#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Schaltet das WiFi-Firmware-Update ein/aus: baut bei "ein" einen eigenen
// WLAN-Access-Point (SoftAP) plus einfachen HTTP-Server auf, der unter
// http://192.168.4.1 eine Upload-Seite fuer eine .bin-Firmware-Datei
// anzeigt, und wechselt den aktiven LVGL-Screen auf eine Status-Anzeige
// (SSID/Passwort/Fortschritt). Bei "aus" wird wieder der vorherige Screen
// geladen und WLAN zurueck in den normalen Scan-Modus (STA) versetzt.
void OTA_Web_SetEnabled(bool enable);

// Aktueller Zustand, damit z.B. ein Schalter im Onboard-Panel (LVGL_Example.c)
// unabhaengig davon synchron bleibt, ob das Update ueber den Schalter selbst
// oder ueber den "Beenden"-Button auf der Update-Seite beendet wurde.
bool OTA_Web_IsEnabled(void);

#ifdef __cplusplus
}
#endif
