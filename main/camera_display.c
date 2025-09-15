/* camera_display.c
 * Capture frames from esp32-camera and display them using the ST7796S driver.
 * This file provides camera_display_start() which creates a task that runs
 * the capture->blit->swap loop. It expects the driver functions from
 * components/st7796s (init_double_buffers, get_draw_buffer, swap_and_display).
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "st7796s.h"

static const char *TAG = "CAMERA_DISPLAY";

// Pin mapping for XIAO ESP32-S3 Sense (provided by user)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#define LED_GPIO_NUM      21

// Capture settings we will use
static esp_err_t init_camera_for_display(void) {
    // Enable PSRAM-backed framebuffers in the camera driver (we have PSRAM)
    esp_camera_set_psram_mode(true);

    camera_config_t camera_config = {
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .sccb_i2c_port = 1, // use SCCB port 1 as in the examples

        .pin_d7 = Y9_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d0 = Y2_GPIO_NUM,

        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_CIF, // use QVGA for reliability
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
    } else {
        // Optional: log sensor PID/VER if driver supports it
        sensor_t *s = esp_camera_sensor_get();
        if (s) {
            ESP_LOGI(TAG, "Camera sensor detected: PID=%d VER=%d", s->id.PID, s->id.VER);
        }
    }
    return err;
}

static void camera_display_task(void *arg) {
    if (init_camera_for_display() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed, exiting camera task");
        vTaskDelete(NULL);
        return;
    }

    // Prepare driver double buffers
    init_double_buffers();

    const int cam_w = 320;
    const int cam_h = 240;
    const size_t cam_row_bytes = cam_w * 2; // RGB565

    const int dest_x = (TFT_WIDTH - cam_w) / 2;
    const int dest_y = (TFT_HEIGHT - cam_h) / 2;

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "camera fb get failed");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // If not RGB565 or unexpected size, try to convert or just skip
        if (fb->format != PIXFORMAT_RGB565 || fb->width != cam_w || fb->height != cam_h) {
            ESP_LOGW(TAG, "Unexpected fb format %d %dx%d", fb->format, fb->width, fb->height);
            // For simplicity, we skip conversion here. Return frame and continue.
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Get draw buffer and memcpy row-by-row into the target position
        uint16_t *draw = get_draw_buffer();
        if (!draw) {
            ESP_LOGE(TAG, "No draw buffer available");
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

            for (int row = 0; row < cam_h; ++row) {
                uint16_t *dst = draw + (dest_y + row) * TFT_WIDTH + dest_x;
                const uint8_t *src = fb->buf + (size_t)row * cam_row_bytes;
                memcpy(dst, src, cam_row_bytes);
            }

            // Use camera-specific flush to avoid byte-order swapping
            flush_camera_frame((const uint16_t*)draw);

        esp_camera_fb_return(fb);

        // small delay to yield and control frame rate
        vTaskDelay(pdMS_TO_TICKS(33)); // ~30 FPS
    }
}

void camera_display_start(void) {
    xTaskCreatePinnedToCore(camera_display_task, "camera_display", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}

// Initialize camera and driver buffers for use from the caller's context.
esp_err_t camera_display_init(void) {
    esp_err_t err = init_camera_for_display();
    if (err != ESP_OK) return err;
    init_double_buffers();
    return ESP_OK;
}

// Perform one capture->blit->display iteration. Caller can invoke repeatedly from main loop.
esp_err_t camera_display_step(void) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGW(TAG, "camera fb get failed");
        return ESP_FAIL;
    }

    int cam_w = fb->width;
    int cam_h = fb->height;
    size_t cam_row_bytes = (size_t)cam_w * 2; // RGB565

    // If not RGB565, bail out
    if (fb->format != PIXFORMAT_RGB565) {
        ESP_LOGW(TAG, "Unexpected fb format %d %dx%d", fb->format, cam_w, cam_h);
        esp_camera_fb_return(fb);
        return ESP_ERR_INVALID_SIZE;
    }

    const int dest_x = (TFT_WIDTH - cam_w) / 2;
    const int dest_y = (TFT_HEIGHT - cam_h) / 2;

    uint16_t *draw = get_draw_buffer();
    if (!draw) {
        ESP_LOGE(TAG, "No draw buffer available");
        esp_camera_fb_return(fb);
        return ESP_ERR_NO_MEM;
    }

    for (int row = 0; row < cam_h; ++row) {
        uint16_t *dst = draw + (dest_y + row) * TFT_WIDTH + dest_x;
        const uint8_t *src = fb->buf + (size_t)row * cam_row_bytes;
        memcpy(dst, src, cam_row_bytes);
    }

        flush_camera_frame((const uint16_t*)draw);
    esp_camera_fb_return(fb);
    return ESP_OK;
}
