#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// Baut den BMW-Multi-Screen (Geschwindigkeit zentral, RPM-Ring im Hintergrund,
// 4 Zusatzfelder) auf einem eigenen Screen auf und laedt ihn als Standard-
// Ansicht. `demo_screen` ist der Screen des Waveshare-Demo-Projekts, der beim
// 3-Sekunden-Halten in der Bildschirmmitte angezeigt wird.
void BMW_UI_Init(lv_obj_t *demo_screen);

// Aktualisiert die Live-Werte der Multi-Ansicht (wird zyklisch aufgerufen).
void BMW_UI_Update(void);

#ifdef __cplusplus
}
#endif
