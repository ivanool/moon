// main_ultra_fast.c - Ejemplo de uso optimizado
#include "st7796s.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
// main/moon.c - NO-TEARING demo (uses double buffering)
#include "st7796s.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char* TAG = "MAIN_NO_TEARING";

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting ultra-smooth ST7796S test (NO TEARING)...");
    
    // PASO 1: Inicializar SPIFFS + TFT (incluye double buffering automáticamente)
    mount_spiffs();
    tft_init();
    init_double_buffers();

    const uint16_t colors[] = {
        rgb888_to_rgb565(255, 0, 0),    // Rojo
        rgb888_to_rgb565(0, 255, 0),    // Verde
        rgb888_to_rgb565(0, 0, 255),    // Azul
        rgb888_to_rgb565(255, 255, 0),  // Amarillo
        rgb888_to_rgb565(255, 0, 255),  // Magenta
        rgb888_to_rgb565(0, 255, 255),  // Cyan
        rgb888_to_rgb565(255, 255, 255) // Blanco
    };
    const int ncolors = sizeof(colors) / sizeof(colors[0]);
    int color_idx = 0;

    // Variables para medir FPS real
    uint64_t last_time = esp_timer_get_time();
    uint32_t frame_count = 0;

    ESP_LOGI(TAG, "🎨 Iniciando loop de renderizado suave (preloading frames into PSRAM)...");

    // PRELOAD: try to load up to MAX_PRELOAD frames into PSRAM to avoid per-frame SPIFFS reads
    const int MAX_PRELOAD = 16; // adjust as you like (memory permitting)
    const size_t FB_BYTES = (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * sizeof(uint16_t);
    uint8_t** preloads = heap_caps_malloc(sizeof(uint8_t*) * MAX_PRELOAD, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int preload_count = 0;
    for (int i = 0; i < MAX_PRELOAD; ++i) {
        char path[64];
        snprintf(path, sizeof(path), "C:/Users/giliv/OneDrive/Escritorio/moon/spiffs_image/%d.bin", i+1);
        // Use fopen on the project filesystem to verify existence (we're running on host during build; on device path is /spiffs)
        FILE* f = fopen(path, "rb");
        if (!f) {
            // try spiffs path (device runtime)
            snprintf(path, sizeof(path), "/spiffs/%d.bin", i+1);
            f = fopen(path, "rb");
        }
        if (!f) {
            break; // stop at first missing file
        }
        uint8_t* buf = heap_caps_malloc(FB_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!buf) {
            fclose(f);
            break;
        }
        size_t read = fread(buf, 1, FB_BYTES, f);
        fclose(f);
        if (read != FB_BYTES) {
            heap_caps_free(buf);
            break;
        }
        preloads[preload_count++] = buf;
        ESP_LOGI(TAG, "Preloaded frame %d into PSRAM", i+1);
    }

    ESP_LOGI(TAG, "Preloaded %d frames into PSRAM", preload_count);

    // Playback loop: memcpy preloaded frame into draw buffer and swap
    while (1) {
        uint64_t frame_start = esp_timer_get_time();
        uint16_t* draw_buffer = get_draw_buffer();
        if (!draw_buffer) {
            ESP_LOGE(TAG, "❌ No se pudo obtener draw buffer");
            vTaskDelay(pdMS_TO_TICKS(16));
            continue;
        }

        if (preload_count > 0) {
            int idx = frame_count % preload_count;
            memcpy(draw_buffer, preloads[idx], FB_BYTES);
        } else {
            // fall back to drawing/color fills if nothing preloaded
            fillScreen(draw_buffer, colors[color_idx]);
        }

        swap_and_display();

        uint64_t frame_end = esp_timer_get_time();
        uint32_t frame_time_us = (uint32_t)(frame_end - frame_start);

        frame_count++;
        color_idx = (color_idx + 1) % ncolors;

        if (frame_end - last_time >= 1000000) {
            float fps = (float)frame_count * 1000000.0f / (float)(frame_end - last_time);
            float frame_time_ms = frame_time_us / 1000.0f;
            ESP_LOGI(TAG, "📊 FPS: %.1f | Frame time: %.2f ms | Status: %s",
                     fps, frame_time_ms,
                     (fps > 30) ? "🚀 SMOOTH" : (fps > 15) ? "✅ Good" : "⚠️ Slow");
            frame_count = 0;
            last_time = frame_end;
        }

        // small delay to avoid starving CPU; adjust for desired FPS
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}

// FUNCIÓN ALTERNATIVA: Test de velocidad pura
void speed_test(void) {
    ESP_LOGI(TAG, "🏁 Iniciando test de velocidad pura...");

    uint64_t start_time = esp_timer_get_time();
    const int test_frames = 100;

    for (int i = 0; i < test_frames; i++) {
        uint16_t* draw_buffer = get_draw_buffer();
        if (!draw_buffer) continue;

        // Dibujar patrón de prueba
        uint16_t color = rgb888_to_rgb565(i % 255, (i * 2) % 255, (i * 3) % 255);
        fillScreen(draw_buffer, color);
        fillRect(draw_buffer, i % 200, (i * 2) % 200, 100, 100, rgb888_to_rgb565(255, 255, 255));

        swap_and_display();
    }

    uint64_t end_time = esp_timer_get_time();
    uint64_t total_time = end_time - start_time;

    float avg_fps = (float)test_frames * 1000000.0f / (float)total_time;
    float avg_frame_time = (float)total_time / (float)test_frames / 1000.0f;

    ESP_LOGI(TAG, "🏆 RESULTADOS DEL TEST:");
    ESP_LOGI(TAG, "   📊 %d frames en %lld μs", test_frames, total_time);
    ESP_LOGI(TAG, "   ⚡ FPS promedio: %.1f", avg_fps);
    ESP_LOGI(TAG, "   ⏱️ Tiempo por frame: %.2f ms", avg_frame_time);
}

// FUNCIÓN: Test de transiciones suaves
void smooth_transition_test(void) {
    ESP_LOGI(TAG, "🌈 Test de transiciones suaves...");

    while (1) {
        // Transición suave entre colores
        for (int r = 0; r <= 255; r += 5) {
            uint16_t* draw_buffer = get_draw_buffer();
            if (!draw_buffer) continue;

            uint16_t color = rgb888_to_rgb565(r, 255 - r, 128);
            fillScreen(draw_buffer, color);

            // Agregar gradiente
            for (int y = 0; y < TFT_HEIGHT; y += 4) {
                uint8_t intensity = (y * 255) / TFT_HEIGHT;
                uint16_t line_color = rgb888_to_rgb565(intensity, intensity, intensity);
                fillRect(draw_buffer, 0, y, TFT_WIDTH, 2, line_color);
            }

            swap_and_display();
            vTaskDelay(pdMS_TO_TICKS(16));  // Transición visible
        }
    }
}

// FUNCIÓN: Test de animación compleja
void complex_animation_test(void) {
    ESP_LOGI(TAG, "🎭 Test de animación compleja...");

    uint32_t frame = 0;

    while (1) {
        uint16_t* draw_buffer = get_draw_buffer();
        if (!draw_buffer) continue;

        // Fondo dinámico
        uint16_t bg_color = rgb888_to_rgb565(
            (frame * 2) % 255,
            (frame * 3) % 255,
            (frame * 5) % 255
        );
        fillScreen(draw_buffer, bg_color);

        // Rectángulos animados
        for (int i = 0; i < 5; i++) {
            int x = (frame * (i + 1) + i * 50) % (TFT_WIDTH - 100);
            int y = (frame * (i + 2) + i * 30) % (TFT_HEIGHT - 60);
            uint16_t rect_color = rgb888_to_rgb565(
                255 - (i * 50),
                i * 50,
                (i * 100) % 255
            );
            fillRect(draw_buffer, x, y, 80, 40, rect_color);
        }

        swap_and_display();
        frame++;

        // vTaskDelay(pdMS_TO_TICKS(16));  // ~60 FPS
    }
}