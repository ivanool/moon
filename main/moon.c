#include "st7796s.h"
#include "camera_display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    ESP_LOGI("MOON", "Iniciando sistema minimal...");
    mount_spiffs();
    tft_init();
    init_double_buffers();

    // Preload frames from SPIFFS (if any)
    int preloaded = preload_spiffs_frames("/spiffs", 64);
    ESP_LOGI("MOON", "Preloaded %d frames from SPIFFS", preloaded);

    // Init camera
    if (camera_display_init() != ESP_OK) {
        ESP_LOGE("MOON", "No se pudo inicializar la cámara");
    }

    const TickType_t camera_period = pdMS_TO_TICKS(1); // ~30 FPS target
    const TickType_t slideshow_period = pdMS_TO_TICKS(15); // ~30 FPS for slideshow

    while (1) {
        // 10 seconds of camera live
        TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(100000);
        while (xTaskGetTickCount() < end) {
            if (camera_display_step() != ESP_OK) {
                // On error, clear screen
                uint16_t *buf = get_draw_buffer();
                if (buf) {
                    fillScreen(buf, 0x0000);
                    swap_and_display();
                }
            }
            vTaskDelay(camera_period);
        }

        // 10 seconds of slideshow using preloaded frames (if any)
        if (preloaded > 0) {
            TickType_t slide_end = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
            int idx = 0;
            while (xTaskGetTickCount() < slide_end) {
                const uint8_t* frame = st7796s_get_preloaded_frame(idx % preloaded);
                if (frame) {
                    uint16_t* draw = get_draw_buffer();
                    if (draw) {
                        // memcpy from preloaded (uint8_t*) into draw buffer (uint16_t*)
                        memcpy(draw, frame, (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * 2);
                        swap_and_display();
                    }
                }
                idx++;
                vTaskDelay(slideshow_period);
            }
        } else {
            // No frames preloaded: small pause
            vTaskDelay(30);
        }
    }
}