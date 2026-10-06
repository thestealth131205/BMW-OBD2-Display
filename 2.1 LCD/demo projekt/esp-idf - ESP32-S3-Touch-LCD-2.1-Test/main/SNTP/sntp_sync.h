#pragma once

#include <stdbool.h>

// Manuell ausgeloester, einmaliger Zeitabgleich per SNTP, Ergebnis wird in
// die PCF85063-RTC geschrieben. Das ist ein ANDERES WLAN als der SoftAP des
// WLAN-Updates (ota_web.c, SSID "BMW-E90-OTA") - hier muss das Display sich
// stattdessen als Client bei einem echten Netz mit Internetzugang anmelden
// (z.B. Handy-Hotspot oder Heim-WLAN).
//
// Laeuft NICHT automatisch beim Boot: Der Nutzer loest den Versuch gezielt
// per "Hotspot verbinden"-Schalter im Funktionen-Screen aus (bmw_ui.c). Der
// Schalterzustand selbst wird bewusst NICHT in NVS gespeichert - nach jedem
// Neustart ist er wieder aus, es wird also nie unbeabsichtigt nach einem
// Hotspot gesucht (z.B. waehrend der Fahrt, wo das die BLE-OBD-Verbindung
// kurz pausieren wuerde, siehe sntp_sync.c).
//
// WICHTIG: Die Zugangsdaten stehen bewusst NICHT im Code (dieses Repo ist
// oeffentlich auf GitHub) - sie stehen auf der SD-Karte in der Textdatei
// /sdcard/wifi-einstellungen.txt. Ist eine Karte eingelegt, aber die Datei
// fehlt noch, legt der erste Verbindungsversuch automatisch eine Vorlage mit
// Erklaerung/Beispiel an (siehe sntp_sync.c). Dann: Karte am PC herausnehmen,
// in der Datei SSID=... und PASSWORT=... ausfuellen, Karte zurueck ins
// Display stecken. Ohne ausgefuellte SSID (oder ohne Karte) bricht der
// Versuch sofort mit einer entsprechenden Statusmeldung ab.

// Startet den einmaligen Verbindungs-/Zeitabgleichsversuch als Hintergrund-
// Task, sofern nicht schon einer laeuft. Nicht blockierend.
void SNTP_Sync_Start(void);

// Bricht einen laufenden Versuch ab (z.B. wenn der Nutzer den Schalter
// waehrend des Verbindens wieder ausschaltet). Ohne laufenden Versuch ein
// No-Op.
void SNTP_Sync_Cancel(void);

// true, solange ein Versuch noch laeuft (Verbinden/Zeitabgleich noch nicht
// abgeschlossen).
bool SNTP_Sync_IsBusy(void);

// Kurzer Statustext fuer die Anzeige ("Inaktiv", "SD-Karte nicht gefunden",
// "Bitte Hotspot-Daten eintragen", "Verbinde...", "Zeit abgeglichen", ...).
const char *SNTP_Sync_Status(void);
