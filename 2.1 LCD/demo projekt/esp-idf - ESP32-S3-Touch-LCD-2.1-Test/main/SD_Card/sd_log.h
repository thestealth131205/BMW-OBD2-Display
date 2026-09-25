#pragma once

// Fortlaufendes Textlog auf der SD-Karte (/sdcard/obd_log.txt, Append-Modus).
// Ist keine Karte gemountet, sind alle Funktionen No-Ops.
void SD_Log_Init(void);
void SD_Log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
