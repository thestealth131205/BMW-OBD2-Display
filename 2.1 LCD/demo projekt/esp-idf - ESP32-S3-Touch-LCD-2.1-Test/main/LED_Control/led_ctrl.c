#include "led_ctrl.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"

// Siehe mcp2515.h: GPIO0 ist dauerhaft frei (der MCP2515-INT-Pin wird im
// Treiber nie ausgewertet, reiner Software-Poll per SPI), unabhaengig von
// der BLE/MCP-Datenquellenwahl - Streifen 1 nutzt diesen Pin. Streifen 2
// nutzt GPIO19, das zugleich die MISO-Leitung des MCP2515 ist: SPI braucht
// alle 4 Leitungen (SCK/MOSI/MISO/CS), daher schliessen sich Streifen 2 und
// MCP2515 gegenseitig aus (siehe docs/led_wiring.png). Haengt am MCP2515-
// Modul trotzdem ein INT-Draht an GPIO0, muss er fuer den Streifen-1-Betrieb
// abgezogen werden, sonst treibt er dieselbe Leitung wie der RMT-Ausgang.
#define LED_CTRL1_GPIO 0
#define LED_CTRL2_GPIO 19

// Anzahl nachziehender, ausblendender LEDs hinter der Chase-Spitze
// ("leichtes Nachziehen" statt hartem Abschnitt).
#define CHASE_TAIL 5

// Geschwindigkeitsregler (0..100%) auf eine Bildverzoegerung abgebildet.
#define CHASE_DELAY_SLOW_MS 160
#define CHASE_DELAY_FAST_MS  12

static const char *TAG = "LED_CTRL";

// Ein Satz Zustand je Streifen - beide laufen unabhaengig in ihrem eigenen
// Hintergrund-Task, mit eigenem Mutex/Start-Semaphor, analog zum frueheren
// Einzelstreifen-Aufbau (nur jetzt zweimal instanziiert statt global).
typedef struct {
    int gpio;
    const char *name;

    led_strip_handle_t strip;
    SemaphoreHandle_t lock;
    SemaphoreHandle_t start_sem;

    volatile bool enabled;
    int count;
    led_pattern_t pattern;
    uint8_t main_r, main_g, main_b;
    uint8_t sec_r, sec_g, sec_b;
    uint8_t speed;
} led_strip_state_t;

static led_strip_state_t s_led[2] = {
    { .gpio = LED_CTRL1_GPIO, .name = "led_ctrl1", .count = 30, .pattern = LED_PATTERN_SOLID,
      .main_r = 0,   .main_g = 162, .main_b = 255, .sec_r = 255, .sec_g = 0, .sec_b = 0, .speed = 50 },
    { .gpio = LED_CTRL2_GPIO, .name = "led_ctrl2", .count = 30, .pattern = LED_PATTERN_SOLID,
      .main_r = 0,   .main_g = 162, .main_b = 255, .sec_r = 255, .sec_g = 0, .sec_b = 0, .speed = 50 },
};

static uint8_t lerp_u8(uint8_t a, uint8_t b, uint8_t frac /* 0..255 */)
{
    int16_t diff = (int16_t)b - (int16_t)a;
    return (uint8_t)(a + (diff * (int16_t)frac) / 255);
}

static void strip_create_locked(led_strip_state_t *s)
{
    if (s->strip) return;

    led_strip_config_t strip_config = {
        .strip_gpio_num = s->gpio,
        .max_leds = LED_CTRL_MAX_LEDS,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s->strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: led_strip_new_rmt_device (GPIO%d) fehlgeschlagen: %s",
                 s->name, s->gpio, esp_err_to_name(err));
        s->strip = NULL;
        return;
    }
    led_strip_clear(s->strip);
}

static void strip_destroy_locked(led_strip_state_t *s)
{
    if (!s->strip) return;
    led_strip_clear(s->strip);
    led_strip_del(s->strip);
    s->strip = NULL;
}

static void render_solid_locked(led_strip_state_t *s)
{
    for (int i = 0; i < s->count; i++) {
        led_strip_set_pixel(s->strip, i, s->main_r, s->main_g, s->main_b);
    }
    led_strip_refresh(s->strip);
}

static void render_chase_frame_locked(led_strip_state_t *s, int head)
{
    for (int i = 0; i < s->count; i++) {
        led_strip_set_pixel(s->strip, i, s->main_r, s->main_g, s->main_b);
    }
    for (int t = 0; t < CHASE_TAIL; t++) {
        int idx = head - t;
        if (idx < 0) idx += s->count;
        if (idx < 0 || idx >= s->count) continue;
        uint8_t fade = (uint8_t)(255 - (255 * t) / CHASE_TAIL);
        uint8_t r = lerp_u8(s->main_r, s->sec_r, fade);
        uint8_t g = lerp_u8(s->main_g, s->sec_g, fade);
        uint8_t b = lerp_u8(s->main_b, s->sec_b, fade);
        led_strip_set_pixel(s->strip, idx, r, g, b);
    }
    led_strip_refresh(s->strip);
}

// Langlebiger Hintergrund-Task je Streifen, nach dem Vorbild des SNTP-
// Hotspot-Tasks (sntp_sync.c): blockiert auf ein Semaphor, bis
// LED_Ctrl[2]_SetEnabled(true) es freigibt, legt dann das RMT-Device an und
// rendert, bis LED_Ctrl[2]_SetEnabled(false) es wieder abraeumt - der Pin
// wird also nur waehrend bewusster Nutzung als Ausgang konfiguriert.
static void led_task(void *arg)
{
    led_strip_state_t *s = (led_strip_state_t *)arg;
    int head = 0;

    while (1) {
        xSemaphoreTake(s->start_sem, portMAX_DELAY);

        xSemaphoreTake(s->lock, portMAX_DELAY);
        strip_create_locked(s);
        xSemaphoreGive(s->lock);

        while (s->enabled && s->strip) {
            xSemaphoreTake(s->lock, portMAX_DELAY);
            if (s->pattern == LED_PATTERN_SOLID) {
                render_solid_locked(s);
                xSemaphoreGive(s->lock);
                vTaskDelay(pdMS_TO_TICKS(200)); // statisches Bild, kein haeufiges Neuzeichnen noetig
                continue;
            }
            if (head >= s->count) head = 0;
            render_chase_frame_locked(s, head);
            head = (head + 1) % s->count;
            xSemaphoreGive(s->lock);

            uint32_t delay_ms = CHASE_DELAY_SLOW_MS -
                ((uint32_t)s->speed * (CHASE_DELAY_SLOW_MS - CHASE_DELAY_FAST_MS)) / 100;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        xSemaphoreTake(s->lock, portMAX_DELAY);
        strip_destroy_locked(s);
        xSemaphoreGive(s->lock);
        head = 0;
    }
}

void LED_Ctrl_Init(void)
{
    for (int i = 0; i < 2; i++) {
        s_led[i].lock = xSemaphoreCreateMutex();
        s_led[i].start_sem = xSemaphoreCreateBinary();
        xTaskCreatePinnedToCore(led_task, s_led[i].name, 3072, &s_led[i], 2, NULL, 1);
    }
}

// --- generische Implementierung, von den duennen Strip-1/Strip-2-Wrappern
// weiter unten aufgerufen ---

static void ctrl_set_enabled(led_strip_state_t *s, bool on)
{
    xSemaphoreTake(s->lock, portMAX_DELAY);
    bool changed = (on != s->enabled);
    s->enabled = on;
    xSemaphoreGive(s->lock);
    if (changed && on) {
        xSemaphoreGive(s->start_sem);
    }
}

static void ctrl_set_count(led_strip_state_t *s, int count)
{
    if (count < 1) count = 1;
    if (count > LED_CTRL_MAX_LEDS) count = LED_CTRL_MAX_LEDS;
    xSemaphoreTake(s->lock, portMAX_DELAY);
    s->count = count;
    xSemaphoreGive(s->lock);
}

static void ctrl_set_pattern(led_strip_state_t *s, led_pattern_t pattern)
{
    xSemaphoreTake(s->lock, portMAX_DELAY);
    s->pattern = pattern;
    xSemaphoreGive(s->lock);
}

static void ctrl_set_main_color(led_strip_state_t *s, uint8_t r, uint8_t g, uint8_t b)
{
    xSemaphoreTake(s->lock, portMAX_DELAY);
    s->main_r = r; s->main_g = g; s->main_b = b;
    xSemaphoreGive(s->lock);
}

static void ctrl_set_second_color(led_strip_state_t *s, uint8_t r, uint8_t g, uint8_t b)
{
    xSemaphoreTake(s->lock, portMAX_DELAY);
    s->sec_r = r; s->sec_g = g; s->sec_b = b;
    xSemaphoreGive(s->lock);
}

static void ctrl_set_speed(led_strip_state_t *s, uint8_t speed_pct)
{
    if (speed_pct > 100) speed_pct = 100;
    s->speed = speed_pct;
}

// --- Streifen 1 (GPIO0) ---
void LED_Ctrl_SetEnabled(bool on) { ctrl_set_enabled(&s_led[0], on); }
bool LED_Ctrl_IsEnabled(void) { return s_led[0].enabled; }
void LED_Ctrl_SetCount(int count) { ctrl_set_count(&s_led[0], count); }
int  LED_Ctrl_GetCount(void) { return s_led[0].count; }
void LED_Ctrl_SetPattern(led_pattern_t pattern) { ctrl_set_pattern(&s_led[0], pattern); }
led_pattern_t LED_Ctrl_GetPattern(void) { return s_led[0].pattern; }
void LED_Ctrl_SetMainColor(uint8_t r, uint8_t g, uint8_t b) { ctrl_set_main_color(&s_led[0], r, g, b); }
void LED_Ctrl_SetSecondColor(uint8_t r, uint8_t g, uint8_t b) { ctrl_set_second_color(&s_led[0], r, g, b); }
void LED_Ctrl_GetMainColor(uint8_t *r, uint8_t *g, uint8_t *b) { *r = s_led[0].main_r; *g = s_led[0].main_g; *b = s_led[0].main_b; }
void LED_Ctrl_GetSecondColor(uint8_t *r, uint8_t *g, uint8_t *b) { *r = s_led[0].sec_r; *g = s_led[0].sec_g; *b = s_led[0].sec_b; }
void LED_Ctrl_SetSpeed(uint8_t speed_pct) { ctrl_set_speed(&s_led[0], speed_pct); }
uint8_t LED_Ctrl_GetSpeed(void) { return s_led[0].speed; }

// --- Streifen 2 (GPIO19) ---
void LED_Ctrl2_SetEnabled(bool on) { ctrl_set_enabled(&s_led[1], on); }
bool LED_Ctrl2_IsEnabled(void) { return s_led[1].enabled; }
void LED_Ctrl2_SetCount(int count) { ctrl_set_count(&s_led[1], count); }
int  LED_Ctrl2_GetCount(void) { return s_led[1].count; }
void LED_Ctrl2_SetPattern(led_pattern_t pattern) { ctrl_set_pattern(&s_led[1], pattern); }
led_pattern_t LED_Ctrl2_GetPattern(void) { return s_led[1].pattern; }
void LED_Ctrl2_SetMainColor(uint8_t r, uint8_t g, uint8_t b) { ctrl_set_main_color(&s_led[1], r, g, b); }
void LED_Ctrl2_SetSecondColor(uint8_t r, uint8_t g, uint8_t b) { ctrl_set_second_color(&s_led[1], r, g, b); }
void LED_Ctrl2_GetMainColor(uint8_t *r, uint8_t *g, uint8_t *b) { *r = s_led[1].main_r; *g = s_led[1].main_g; *b = s_led[1].main_b; }
void LED_Ctrl2_GetSecondColor(uint8_t *r, uint8_t *g, uint8_t *b) { *r = s_led[1].sec_r; *g = s_led[1].sec_g; *b = s_led[1].sec_b; }
void LED_Ctrl2_SetSpeed(uint8_t speed_pct) { ctrl_set_speed(&s_led[1], speed_pct); }
uint8_t LED_Ctrl2_GetSpeed(void) { return s_led[1].speed; }
