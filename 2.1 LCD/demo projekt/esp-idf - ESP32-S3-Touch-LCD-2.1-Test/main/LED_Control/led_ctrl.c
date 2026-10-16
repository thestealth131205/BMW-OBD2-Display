#include "led_ctrl.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"

// Siehe mcp2515.h: GPIO0 ist jetzt dauerhaft frei (der MCP2515-INT-Pin wird
// im Treiber nie ausgewertet, reiner Software-Poll per SPI), unabhaengig von
// der BLE/MCP-Datenquellenwahl. Haengt am MCP2515-Modul trotzdem ein INT-
// Draht an GPIO0, muss er fuer den LED-Betrieb abgezogen werden, sonst
// treibt er dieselbe Leitung wie der RMT-Ausgang.
#define LED_CTRL_GPIO 0

// Anzahl nachziehender, ausblendender LEDs hinter der Chase-Spitze
// ("leichtes Nachziehen" statt hartem Abschnitt).
#define CHASE_TAIL 5

// Geschwindigkeitsregler (0..100%) auf eine Bildverzoegerung abgebildet.
#define CHASE_DELAY_SLOW_MS 160
#define CHASE_DELAY_FAST_MS  12

static const char *TAG = "LED_CTRL";

static led_strip_handle_t s_strip = NULL;
static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_start_sem;

static volatile bool s_enabled = false;
static int s_count = 30;
static led_pattern_t s_pattern = LED_PATTERN_SOLID;
static uint8_t s_main_r = 0,   s_main_g = 162, s_main_b = 255; // Standard: Blau
static uint8_t s_sec_r  = 255, s_sec_g  = 0,   s_sec_b  = 0;   // Standard: Rot
static uint8_t s_speed  = 50;

static uint8_t lerp_u8(uint8_t a, uint8_t b, uint8_t frac /* 0..255 */)
{
    int16_t diff = (int16_t)b - (int16_t)a;
    return (uint8_t)(a + (diff * (int16_t)frac) / 255);
}

static void strip_create_locked(void)
{
    if (s_strip) return;

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_CTRL_GPIO,
        .max_leds = LED_CTRL_MAX_LEDS,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device fehlgeschlagen: %s", esp_err_to_name(err));
        s_strip = NULL;
        return;
    }
    led_strip_clear(s_strip);
}

static void strip_destroy_locked(void)
{
    if (!s_strip) return;
    led_strip_clear(s_strip);
    led_strip_del(s_strip);
    s_strip = NULL;
}

static void render_solid_locked(void)
{
    for (int i = 0; i < s_count; i++) {
        led_strip_set_pixel(s_strip, i, s_main_r, s_main_g, s_main_b);
    }
    led_strip_refresh(s_strip);
}

static void render_chase_frame_locked(int head)
{
    for (int i = 0; i < s_count; i++) {
        led_strip_set_pixel(s_strip, i, s_main_r, s_main_g, s_main_b);
    }
    for (int t = 0; t < CHASE_TAIL; t++) {
        int idx = head - t;
        if (idx < 0) idx += s_count;
        if (idx < 0 || idx >= s_count) continue;
        uint8_t fade = (uint8_t)(255 - (255 * t) / CHASE_TAIL);
        uint8_t r = lerp_u8(s_main_r, s_sec_r, fade);
        uint8_t g = lerp_u8(s_main_g, s_sec_g, fade);
        uint8_t b = lerp_u8(s_main_b, s_sec_b, fade);
        led_strip_set_pixel(s_strip, idx, r, g, b);
    }
    led_strip_refresh(s_strip);
}

// Langlebiger Hintergrund-Task, nach dem Vorbild des SNTP-Hotspot-Tasks
// (sntp_sync.c): blockiert auf ein Semaphor, bis LED_Ctrl_SetEnabled(true)
// es freigibt, legt dann das RMT-Device an und rendert, bis
// LED_Ctrl_SetEnabled(false) es wieder abraeumt - der Pin wird also nur
// waehrend bewusster Nutzung als Ausgang konfiguriert.
static void led_task(void *arg)
{
    (void)arg;
    int head = 0;

    while (1) {
        xSemaphoreTake(s_start_sem, portMAX_DELAY);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        strip_create_locked();
        xSemaphoreGive(s_lock);

        while (s_enabled && s_strip) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (s_pattern == LED_PATTERN_SOLID) {
                render_solid_locked();
                xSemaphoreGive(s_lock);
                vTaskDelay(pdMS_TO_TICKS(200)); // statisches Bild, kein haeufiges Neuzeichnen noetig
                continue;
            }
            if (head >= s_count) head = 0;
            render_chase_frame_locked(head);
            head = (head + 1) % s_count;
            xSemaphoreGive(s_lock);

            uint32_t delay_ms = CHASE_DELAY_SLOW_MS -
                ((uint32_t)s_speed * (CHASE_DELAY_SLOW_MS - CHASE_DELAY_FAST_MS)) / 100;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        xSemaphoreTake(s_lock, portMAX_DELAY);
        strip_destroy_locked();
        xSemaphoreGive(s_lock);
        head = 0;
    }
}

void LED_Ctrl_Init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_start_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(led_task, "led_ctrl", 3072, NULL, 2, NULL, 1);
}

void LED_Ctrl_SetEnabled(bool on)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool changed = (on != s_enabled);
    s_enabled = on;
    xSemaphoreGive(s_lock);
    if (changed && on) {
        xSemaphoreGive(s_start_sem);
    }
}

bool LED_Ctrl_IsEnabled(void)
{
    return s_enabled;
}

void LED_Ctrl_SetCount(int count)
{
    if (count < 1) count = 1;
    if (count > LED_CTRL_MAX_LEDS) count = LED_CTRL_MAX_LEDS;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_count = count;
    xSemaphoreGive(s_lock);
}

int LED_Ctrl_GetCount(void)
{
    return s_count;
}

void LED_Ctrl_SetPattern(led_pattern_t pattern)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_pattern = pattern;
    xSemaphoreGive(s_lock);
}

led_pattern_t LED_Ctrl_GetPattern(void)
{
    return s_pattern;
}

void LED_Ctrl_SetMainColor(uint8_t r, uint8_t g, uint8_t b)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_main_r = r; s_main_g = g; s_main_b = b;
    xSemaphoreGive(s_lock);
}

void LED_Ctrl_SetSecondColor(uint8_t r, uint8_t g, uint8_t b)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_sec_r = r; s_sec_g = g; s_sec_b = b;
    xSemaphoreGive(s_lock);
}

void LED_Ctrl_GetMainColor(uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = s_main_r; *g = s_main_g; *b = s_main_b;
}

void LED_Ctrl_GetSecondColor(uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = s_sec_r; *g = s_sec_g; *b = s_sec_b;
}

void LED_Ctrl_SetSpeed(uint8_t speed_pct)
{
    if (speed_pct > 100) speed_pct = 100;
    s_speed = speed_pct;
}

uint8_t LED_Ctrl_GetSpeed(void)
{
    return s_speed;
}
