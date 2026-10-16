#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Adressierbarer LED-Streifen (WS2812/SK6812-kompatibel) an GPIO0 - dem
// einzigen der 5 herausgefuehrten freien GPIOs (0/19/20/43/44), der nicht
// fest fuer das MCP2515-CAN-Modul gebraucht wird (siehe mcp2515.h). Zwei
// Muster: Solid (alle LEDs identisch eingefaerbt) und Chase (Hauptfarbe als
// Hintergrund, Zweitfarbe laeuft mit kurzem Komet-Schweif von LED 0 bis zum
// Ende, im Loop, Geschwindigkeit einstellbar).

#define LED_CTRL_MAX_LEDS 100

typedef enum {
    LED_PATTERN_SOLID = 0,
    LED_PATTERN_CHASE,
} led_pattern_t;

// Einmalig beim Boot aufrufen: legt nur den (blockierten) Hintergrund-Task
// an, konfiguriert GPIO0 aber noch nicht als Ausgang - das passiert erst in
// LED_Ctrl_SetEnabled(true), damit der Pin ausserhalb bewusster Nutzung
// nicht dauerhaft als RMT-Ausgang belegt ist.
void LED_Ctrl_Init(void);

// Haupt-Ein-/Ausschalter.
void LED_Ctrl_SetEnabled(bool on);
bool LED_Ctrl_IsEnabled(void);

void LED_Ctrl_SetCount(int count);   // wird auf 1..LED_CTRL_MAX_LEDS begrenzt
int  LED_Ctrl_GetCount(void);

void LED_Ctrl_SetPattern(led_pattern_t pattern);
led_pattern_t LED_Ctrl_GetPattern(void);

void LED_Ctrl_SetMainColor(uint8_t r, uint8_t g, uint8_t b);
void LED_Ctrl_SetSecondColor(uint8_t r, uint8_t g, uint8_t b);
void LED_Ctrl_GetMainColor(uint8_t *r, uint8_t *g, uint8_t *b);
void LED_Ctrl_GetSecondColor(uint8_t *r, uint8_t *g, uint8_t *b);

// 0 (langsam) .. 100 (schnell), nur fuer LED_PATTERN_CHASE relevant.
void LED_Ctrl_SetSpeed(uint8_t speed_pct);
uint8_t LED_Ctrl_GetSpeed(void);

#ifdef __cplusplus
}
#endif
