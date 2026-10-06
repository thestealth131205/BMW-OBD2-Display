#pragma once

// Einmaliger Zeitabgleich per SNTP beim Boot, Ergebnis wird in die PCF85063-
// RTC geschrieben. Das ist ein ANDERES WLAN als der SoftAP des WLAN-Updates
// (ota_web.c, SSID "BMW-E90-OTA") - hier muss das Display sich stattdessen
// als Client bei einem echten Netz mit Internetzugang anmelden (z.B. Handy-
// Hotspot oder Heim-WLAN).
//
// WICHTIG: Die Zugangsdaten stehen bewusst NICHT im Code (dieses Repo ist
// oeffentlich auf GitHub) - sie stehen auf der SD-Karte in der Textdatei
// /sdcard/wifi-einstellungen.txt. Ist eine Karte eingelegt, aber die Datei
// fehlt noch, legt SNTP_Sync_Init() beim naechsten Boot automatisch eine
// Vorlage mit Erklaerung/Beispiel an (siehe sntp_sync.c). Dann: Karte am PC
// herausnehmen, in der Datei SSID=... und PASSWORT=... ausfuellen, Karte
// zurueck ins Display stecken. Ohne ausgefuellte SSID (oder ohne Karte)
// bleibt das Feature inaktiv - kein schaedlicher Nebeneffekt.

// Startet den einmaligen Verbindungs-/Zeitabgleichsversuch als Hintergrund-
// Task. Nicht blockierend. Ohne erreichbares WLAN aus der Einstellungsdatei
// (z.B. im Auto, ausserhalb der Reichweite) bricht der Versuch nach ein paar
// Sekunden Timeout ab und die RTC bleibt unveraendert.
void SNTP_Sync_Init(void);
