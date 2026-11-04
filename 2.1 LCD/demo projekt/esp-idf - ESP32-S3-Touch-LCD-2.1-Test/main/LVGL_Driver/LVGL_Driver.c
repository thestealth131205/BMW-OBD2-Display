#include "LVGL_Driver.h"
#include <math.h>
#include <string.h>
#include "esp_heap_caps.h"

static const char *LVGL_TAG = "LVGL";   
lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
lv_disp_drv_t disp_drv;      // contains callback functions

lv_indev_drv_t indev_drv;
esp_timer_handle_t lvgl_tick_timer = NULL;


static void *buf1 = NULL;
static void *buf2 = NULL;             


#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

// --- Display-Drehung um beliebige Winkel (Ausgleich einer schief montierten
// Halterung, siehe LVGL_SetRotation()) ---
// LVGL 8 kann nur 0/90/180/270 Grad drehen. Fuer freie Winkel rendert LVGL
// bei aktiver Drehung in einen eigenen Vollbild-Puffer (s_rot_src, PSRAM),
// den example_lvgl_flush_cb() per Rueckwaerts-Abbildung (naechster Nachbar,
// 16.16-Festkomma) gedreht in den gerade nicht angezeigten Panel-Framebuffer
// kopiert und diesen dann anzeigt. Nur Pixel innerhalb des runden Panels
// (Radius 240) werden berechnet. Bei 0 Grad rendert LVGL wie bisher direkt
// in die Panel-Framebuffer - dann kostet die Funktion nichts.
static int16_t s_rot_deg = 0;
static float s_rot_cos = 1.0f, s_rot_sin = 0.0f;
static lv_color_t *s_rot_src = NULL;
static void *s_fb[2] = {NULL, NULL};
static void *s_fb_shown = NULL;
static uint16_t s_row_half[EXAMPLE_LCD_V_RES]; // halbe sichtbare Zeilenbreite des runden Panels

// Bildschirmpunkt (physisch) -> LVGL-Koordinate: Drehung um -s_rot_deg um
// die Bildschirmmitte (positive Winkel = Inhalt im Uhrzeigersinn gedreht).
static inline void rot_inverse(float dx, float dy, float *sx, float *sy)
{
    *sx = s_rot_cos * dx + s_rot_sin * dy;
    *sy = -s_rot_sin * dx + s_rot_cos * dy;
}

static void rotate_frame(uint16_t *dst, const uint16_t *src)
{
    const int W = EXAMPLE_LCD_H_RES, H = EXAMPLE_LCD_V_RES;
    const float c = (W - 1) * 0.5f;
    const int32_t step_x = (int32_t)lroundf(s_rot_cos * 65536.0f);
    const int32_t step_y = (int32_t)lroundf(-s_rot_sin * 65536.0f);
    for (int y = 0; y < H; y++) {
        int half = s_row_half[y];
        int x0 = (W / 2) - half, x1 = (W / 2) + half;
        if (x0 < 0) x0 = 0;
        if (x1 > W) x1 = W;
        if (x0 >= x1) continue;
        float sxf, syf;
        rot_inverse((float)x0 - c, (float)y - c, &sxf, &syf);
        // +0.5 (0x8000) -> Rundung auf den naechsten Quellpixel
        int32_t sx = (int32_t)lroundf((sxf + c) * 65536.0f) + 0x8000;
        int32_t sy = (int32_t)lroundf((syf + c) * 65536.0f) + 0x8000;
        uint16_t *d = dst + y * W + x0;
        for (int x = x0; x < x1; x++) {
            int32_t ix = sx >> 16, iy = sy >> 16;
            *d++ = ((uint32_t)ix < (uint32_t)W && (uint32_t)iy < (uint32_t)H) ? src[iy * W + ix] : 0;
            sx += step_x;
            sy += step_y;
        }
    }
}

int16_t LVGL_GetRotation(void)
{
    return s_rot_deg;
}

bool LVGL_SetRotation(int16_t deg)
{
#if CONFIG_EXAMPLE_DOUBLE_FB
    while (deg > 180)   deg -= 360;
    while (deg <= -180) deg += 360;
    if (deg == s_rot_deg) return true;

    if (deg != 0 && s_rot_src == NULL) {
        s_rot_src = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
        if (s_rot_src == NULL) {
            ESP_LOGE(LVGL_TAG, "Kein PSRAM fuer Drehpuffer - Drehung nicht moeglich");
            return false;
        }
        memset(s_rot_src, 0, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t));
        for (int y = 0; y < EXAMPLE_LCD_V_RES; y++) {
            float dy = (float)y - (EXAMPLE_LCD_V_RES - 1) * 0.5f;
            float r = EXAMPLE_LCD_H_RES * 0.5f;
            s_row_half[y] = (dy * dy < r * r) ? (uint16_t)ceilf(sqrtf(r * r - dy * dy)) : 0;
        }
    }

    float rad = (float)deg * (float)M_PI / 180.0f;
    s_rot_cos = cosf(rad);
    s_rot_sin = sinf(rad);
    bool was_rotated = (s_rot_deg != 0);
    s_rot_deg = deg;

    if (deg != 0 && !was_rotated) {
        // LVGL ab jetzt in den eigenen Vollbild-Puffer rendern lassen
        lv_disp_draw_buf_init(&disp_buf, s_rot_src, NULL, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
    } else if (deg == 0 && was_rotated) {
        // Zurueck auf direktes Rendern in die Panel-Framebuffer - zuerst in
        // den gerade NICHT angezeigten, sonst sieht man den Aufbau kurz.
        void *back = (s_fb_shown == s_fb[0]) ? s_fb[1] : s_fb[0];
        void *front = (back == s_fb[0]) ? s_fb[1] : s_fb[0];
        lv_disp_draw_buf_init(&disp_buf, back, front, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
    }
    lv_obj_invalidate(lv_scr_act());
    return true;
#else
    LV_UNUSED(deg);
    return false;
#endif
}

void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
#if CONFIG_EXAMPLE_DOUBLE_FB
    if (s_rot_src != NULL && color_map == s_rot_src) {
        // full_refresh: color_map ist immer das komplette Bild
        uint16_t *dst = (uint16_t *)((s_fb_shown == s_fb[0]) ? s_fb[1] : s_fb[0]);
        rotate_frame(dst, (const uint16_t *)color_map);
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, dst);
        s_fb_shown = dst;
        lv_disp_flush_ready(drv);
        return;
    }
    s_fb_shown = color_map;
#endif
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

/*Read the touchpad*/
void example_touchpad_read( lv_indev_drv_t * drv, lv_indev_data_t * data )
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        if (s_rot_deg != 0) {
            // Beruehrpunkt mit derselben Rueckwaerts-Drehung wie die Pixel auf
            // die LVGL-Koordinate abbilden, damit Buttons dort reagieren, wo
            // sie gedreht zu sehen sind.
            const float c = (EXAMPLE_LCD_H_RES - 1) * 0.5f;
            float sx, sy;
            rot_inverse((float)touchpad_x[0] - c, (float)touchpad_y[0] - c, &sx, &sy);
            int32_t lx = (int32_t)lroundf(sx + c), ly = (int32_t)lroundf(sy + c);
            if (lx < 0) lx = 0;
            if (lx > EXAMPLE_LCD_H_RES - 1) lx = EXAMPLE_LCD_H_RES - 1;
            if (ly < 0) ly = 0;
            if (ly > EXAMPLE_LCD_V_RES - 1) ly = EXAMPLE_LCD_V_RES - 1;
            data->point.x = (lv_coord_t)lx;
            data->point.y = (lv_coord_t)ly;
        }
        data->state = LV_INDEV_STATE_PR;
        ESP_LOGI(LVGL_TAG, "X=%u Y=%u", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}
void LVGL_Init(void)
{
    ESP_LOGI(LVGL_TAG, "Initialize LVGL library");
    lv_init();
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(LVGL_TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    s_fb[0] = buf1;
    s_fb[1] = buf2;
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(LVGL_TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(LVGL_TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(LVGL_TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };

    /********************* LVGL *********************/
    ESP_LOGI(LVGL_TAG,"Register display indev to LVGL");
    lv_indev_drv_init ( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_touchpad_read;
    indev_drv.user_data = tp;
    lv_indev_drv_register( &indev_drv );

    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

}