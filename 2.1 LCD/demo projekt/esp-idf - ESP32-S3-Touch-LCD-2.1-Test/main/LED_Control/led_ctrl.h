#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Adressierbare LED-Streifen (WS2812/SK6812-kompatibel). Streifen 1 haengt an
// GPIO0 - dem einzigen der 5 herausgefuehrten freien GPIOs (0/19/20/43/44),
// der nicht fest fuer das MCP2515-CAN-Modul gebraucht wird (siehe
// mcp2515.h). Streifen 2 haengt an GPIO19 - das ist zugleich die MISO-
// Leitung des MCP2515: Streifen 2 und MCP2515 schliessen sich gegenseitig
// aus (SPI braucht alle 4 Leitungen, nicht nur 3), wird Streifen 2 benutzt,
// muss die Datenquelle auf BLE-OBD stehen und das MCP2515-Modul unverbunden
// bleiben (siehe docs/led_wiring.png). Beide Streifen unabhaengig
// voneinander schaltbar, je zwei Muster: Solid (alle LEDs identisch
// eingefaerbt) und Chase (Hauptfarbe als Hintergrund, Zweitfarbe laeuft mit
// kurzem Komet-Schweif von LED 0 bis zum Ende, im Loop, Geschwindigkeit
// einstellbar).

#define LED_CTRL_MAX_LEDS 100

typedef enum {
    LED_PATTERN_SOLID = 0,
    LED_PATTERN_CHASE,
} led_pattern_t;

// Einmalig beim Boot aufrufen: legt nur die (blockierten) Hintergrund-Tasks
// fuer beide Streifen an, konfiguriert die GPIOs aber noch nicht als Ausgang
// - das passiert erst in LED_Ctrl_SetEnabled(true)/LED_Ctrl2_SetEnabled(true),
// damit die Pins ausserhalb bewusster Nutzung nicht dauerhaft als RMT-
// Ausgang belegt sind.
void LED_Ctrl_Init(void);

// --- Streifen 1 (GPIO0) ---
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
// 0 (aus) .. 100 (volle Helligkeit), skaliert Haupt-/Zweitfarbe vor dem Senden.
void LED_Ctrl_SetBrightness(uint8_t brightness_pct);
uint8_t LED_Ctrl_GetBrightness(void);

// --- Streifen 2 (GPIO19 - teilt sich den Pin mit MCP2515-MISO) ---
void LED_Ctrl2_SetEnabled(bool on);
bool LED_Ctrl2_IsEnabled(void);
void LED_Ctrl2_SetCount(int count);
int  LED_Ctrl2_GetCount(void);
void LED_Ctrl2_SetPattern(led_pattern_t pattern);
led_pattern_t LED_Ctrl2_GetPattern(void);
void LED_Ctrl2_SetMainColor(uint8_t r, uint8_t g, uint8_t b);
void LED_Ctrl2_SetSecondColor(uint8_t r, uint8_t g, uint8_t b);
void LED_Ctrl2_GetMainColor(uint8_t *r, uint8_t *g, uint8_t *b);
void LED_Ctrl2_GetSecondColor(uint8_t *r, uint8_t *g, uint8_t *b);
void LED_Ctrl2_SetSpeed(uint8_t speed_pct);
uint8_t LED_Ctrl2_GetSpeed(void);
void LED_Ctrl2_SetBrightness(uint8_t brightness_pct);
uint8_t LED_Ctrl2_GetBrightness(void);

#ifdef __cplusplus
}
#endif
