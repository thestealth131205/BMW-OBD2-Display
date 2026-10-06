#pragma once

// Fortlaufendes Textlog auf der SD-Karte (/sdcard/obd_log.txt, Append-Modus).
// Ist keine Karte gemountet, sind alle Funktionen No-Ops.
void SD_Log_Init(void);
void SD_Log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Schliesst die Log-Datei und setzt den Init-Zustand zurueck, damit
// SD_Log_Init() danach wieder eine neue Datei anlegt (z.B. nach SD_Format()
// -  der alte Datei-Handle ist nach einer Neuformatierung ungueltig).
void SD_Log_Deinit(void);
