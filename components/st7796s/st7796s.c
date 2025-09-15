#include "st7796s.h"

// Incluye todas las cabeceras de ESP-IDF necesarias para la implementación.
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"

// ... (El resto de tus defines y prototipos se mantienen igual) ...
static const char* TAG = "ST7796S_LIB";
#define CMD_MODE  0
#define DATA_MODE 1
#define NOP         0x00
#define SWRESET     0x01
#define SLPOUT      0x11
#define NORON       0x13
#define INVOFF      0x20
#define INVON       0x21
#define DISPON      0x29
#define CASET       0x2A
#define RASET       0x2B
#define RAMWR       0x2C
#define COLMOD      0x3A
#define MADCTL      0x36
#define PORCTRL     0xB2
#define GCTRL       0xB7
#define VCOMS       0xBB

// OPTIMIZACIONES: Nuevas definiciones para máxima velocidad
#define SPI_MAX_TRANSFER_SIZE (32 * 1024)  // 64KB - máximo para ESP32-S3
#define SPI_CLOCK_SPEED_HZ (80 * 1000 * 1000)  // 80MHz - máximo estable
#define SPI_QUEUE_SIZE 8  // Cola más grande para DMA

static spi_device_handle_t spi_handle;

// Variables estáticas para optimización
static uint8_t* dma_buffer = NULL;
static size_t dma_buffer_size = 0;
static bool window_set = false;

// Preloaded frames store
static uint8_t** preloaded_frames = NULL;
static int preloaded_count = 0;
static size_t preloaded_frame_bytes = 0;

static void setup_spi(void);
static void send_cmd(uint8_t cmd);
static void send_data(const uint8_t* data, size_t size);
static void send_word(uint16_t data);
static void enviar_datos_grandes_dma(const uint8_t* data, size_t size);
static void gpio_init(void);
static void tft_reset(void);
static void porch_control(void);
void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
static void draw_char_scaled(uint16_t* frame_buffer, int32_t x, int32_t y, char c, uint16_t color, uint8_t scale, const uint8_t* font);

// --- Implementación de Funciones Públicas ---
void mount_spiffs(void) {
    // Try mounting the partition with the explicit label first. If that
    // fails (e.g., spiffs image wasn't flashed), fall back to trying the
    // default unnamed SPIFFS partition so the device can still operate.
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs", .partition_label = SPIFFS_PARTITION_LABEL,
        .max_files = 5, .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Mount with label '%s' failed: %s. Trying default SPIFFS partition...",
                 SPIFFS_PARTITION_LABEL, esp_err_to_name(ret));
        // Try without partition label (default partition)
        conf.partition_label = NULL;
        esp_err_t ret2 = esp_vfs_spiffs_register(&conf);
        if (ret2 != ESP_OK) {
            if (ret2 == ESP_FAIL) ESP_LOGE(TAG, "Failed to mount or format filesystem");
            else if (ret2 == ESP_ERR_NOT_FOUND) ESP_LOGE(TAG, "Failed to find SPIFFS partition");
            else ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret2));
            return;
        } else {
            ESP_LOGI(TAG, "Mounted default SPIFFS partition at %s", conf.base_path);
        }
    } else {
        ESP_LOGI(TAG, "Mounted SPIFFS partition '%s' at %s", SPIFFS_PARTITION_LABEL, conf.base_path);
    }
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to get SPIFFS info (%s)", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "SPIFFS Partition size: total: %d, used: %d", total, used);
}

int preload_spiffs_frames(const char* base_dir, int max_preload) {
    if (!base_dir || max_preload <= 0) return 0;
    // free previous if any
    if (preloaded_frames) {
        for (int i = 0; i < preloaded_count; ++i) if (preloaded_frames[i]) heap_caps_free(preloaded_frames[i]);
        heap_caps_free(preloaded_frames);
        preloaded_frames = NULL;
        preloaded_count = 0;
    }

    size_t fb_bytes = (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * sizeof(uint16_t);
    preloaded_frame_bytes = fb_bytes;

    preloaded_frames = heap_caps_malloc(sizeof(uint8_t*) * max_preload, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!preloaded_frames) return 0;

    for (int i = 0; i < max_preload; ++i) {
        char path[128];
        snprintf(path, sizeof(path), "%s/%d.bin", base_dir, i + 1);
        FILE* f = fopen(path, "rb");
        if (!f) {
            break; // stop on first missing
        }
        uint8_t* buf = heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!buf) { fclose(f); break; }
        size_t r = fread(buf, 1, fb_bytes, f);
        fclose(f);
        if (r != fb_bytes) { heap_caps_free(buf); break; }
        preloaded_frames[preloaded_count++] = buf;
        ESP_LOGI(TAG, "Preloaded frame %d (%s)", i+1, path);
    }
    if (preloaded_count == 0) {
        heap_caps_free(preloaded_frames);
        preloaded_frames = NULL;
    }
    return preloaded_count;
}

const uint8_t* st7796s_get_preloaded_frame(int index) {
    if (index < 0 || index >= preloaded_count) return NULL;
    return preloaded_frames[index];
}

int st7796s_get_preloaded_count(void) {
    return preloaded_count;
}

void st7796s_free_preloaded_frames(void) {
    if (!preloaded_frames) return;
    for (int i = 0; i < preloaded_count; ++i) if (preloaded_frames[i]) heap_caps_free(preloaded_frames[i]);
    heap_caps_free(preloaded_frames);
    preloaded_frames = NULL;
    preloaded_count = 0;
    preloaded_frame_bytes = 0;
}

void load_font(uint8_t *font_data) {
    FILE* file = fopen(FONT_FILE, "rb");
    size_t font_size = (size_t)(FONT_END - FONT_START + 1) * (size_t)FONT_HEIGHT;
    if (file) {
        size_t read = fread(font_data, sizeof(uint8_t), font_size, file);
        fclose(file);
        if (read == font_size) {
            ESP_LOGI(TAG, "Fuente '%s' cargada correctamente (%u bytes).", FONT_FILE, (unsigned)font_size);
            return;
        } else {
            ESP_LOGW(TAG, "Fuente '%s' leída parcialmente (%u de %u). Usando fuente por defecto.", FONT_FILE, (unsigned)read, (unsigned)font_size);
        }
    } else {
        ESP_LOGW(TAG, "No se pudo abrir '%s'. Usando fuente por defecto integrada.", FONT_FILE);
    }
    for (int ch = 0; ch < (FONT_END - FONT_START + 1); ch++) {
        for (int row = 0; row < FONT_HEIGHT; row++) {
            font_data[ch * FONT_HEIGHT + row] = 0xF8;
        }
    }
    ESP_LOGI(TAG, "Fuente por defecto integrada cargada (%u bytes).", (unsigned)font_size);
}

// OPTIMIZACIÓN: flush mejorado con ventana fija
void flush(const uint16_t* frame_buffer) {
    // Enviar comandos de ventana una sola vez al inicio
    if (!window_set) {
        set_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
        window_set = true;
    } else {
        // Solo enviar RAMWR para frames subsecuentes
        send_cmd(RAMWR);
    }
    enviar_datos_grandes_dma((const uint8_t*)frame_buffer, TFT_WIDTH * TFT_HEIGHT * 2);
}

void set_orientation(orientation_t orientation) {
    uint8_t madctl_data = 0;
    switch (orientation) {
        case PORTRAIT:           madctl_data = 0x08; break;
        case LANDSCAPE:          madctl_data = 0x68; break;
        case PORTRAIT_INVERTED:  madctl_data = 0xC8; break;
        case LANDSCAPE_INVERTED: madctl_data = 0xA8; break;
        default:                 madctl_data = 0x08; break;
    }
    send_cmd(MADCTL);
    send_data(&madctl_data, 1);
    // Reset window flag cuando cambiamos orientación
    window_set = false;
}

void backlight(uint8_t duty) {
    ESP_LOGD(TAG, "Ajustando backlight a %d", duty);
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES, .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL,
        .gpio_num = TFT_BL, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// OPTIMIZACIÓN: fillScreen con loop unrolling
void fillScreen(uint16_t* frame_buffer, uint16_t color) {
    uint32_t total_pixels = TFT_WIDTH * TFT_HEIGHT;
    uint32_t color32 = (color << 16) | color;
    uint32_t* fb_ptr32 = (uint32_t*)frame_buffer;
    
    // Optimización: loop unrolling para escribir 8 palabras (16 píxeles) por iteración
    uint32_t words_to_write = total_pixels / 2;
    uint32_t remaining_words = words_to_write;
    
    while (remaining_words >= 8) {
        *fb_ptr32++ = color32;
        *fb_ptr32++ = color32;
        *fb_ptr32++ = color32;
        *fb_ptr32++ = color32;
        *fb_ptr32++ = color32;
        *fb_ptr32++ = color32;
        *fb_ptr32++ = color32;
        *fb_ptr32++ = color32;
        remaining_words -= 8;
    }
    
    // Escribir palabras restantes
    while (remaining_words > 0) {
        *fb_ptr32++ = color32;
        remaining_words--;
    }
    
    // Si hay un píxel impar
    if (total_pixels % 2) {
        frame_buffer[total_pixels - 1] = color;
    }
}

void draw_pixel(uint16_t* frame_buffer, int32_t x, int32_t y, uint16_t color) {
    if (x < 0 || x >= TFT_WIDTH || y < 0 || y >= TFT_HEIGHT) return;
    frame_buffer[y * TFT_WIDTH + x] = color;
}

// OPTIMIZACIÓN: fillRect mejorado
void fillRect(uint16_t* frame_buffer, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color) {
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT || (x + w) <= 0 || (y + h) <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if ((x + w) > TFT_WIDTH) w = TFT_WIDTH - x;
    if ((y + h) > TFT_HEIGHT) h = TFT_HEIGHT - y;
    
    // Si el rectángulo abarca todo el ancho, optimizar con palabras de 32 bits
    if (x == 0 && w == TFT_WIDTH) {
        uint16_t* start_ptr = &frame_buffer[y * TFT_WIDTH];
        size_t total_pixels = w * h;
        
        if (total_pixels >= 4) {
            uint32_t color32 = (color << 16) | color;
            uint32_t* ptr32 = (uint32_t*)start_ptr;
            uint32_t words = total_pixels / 2;
            
            while (words >= 4) {
                *ptr32++ = color32;
                *ptr32++ = color32;
                *ptr32++ = color32;
                *ptr32++ = color32;
                words -= 4;
            }
            while (words > 0) {
                *ptr32++ = color32;
                words--;
            }
            
            if (total_pixels % 2) {
                start_ptr[total_pixels - 1] = color;
            }
        }
    } else {
        // Método tradicional para rectángulos parciales
        for (int32_t j = 0; j < h; j++) {
            uint16_t* row_ptr = &frame_buffer[(y + j) * TFT_WIDTH + x];
            for (int32_t i = 0; i < w; i++) {
                *row_ptr++ = color;
            }
        }
    }
}

bool drawImage(uint16_t* frame_buffer, const char* path) {
    FILE* file = fopen(path, "rb");
    if (!file) {
        ESP_LOGE(TAG, "No se pudo abrir el archivo de imagen: %s", path);
        return false;
    }

    // Determine file size first so we can detect non-raw/compressed images.
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    if (file_size < 0) file_size = 0;

    const size_t expected_bytes = (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * sizeof(uint16_t);
    if ((size_t)file_size != expected_bytes) {
        ESP_LOGW(TAG, "Imagen '%s' no tiene el tamaño esperado: %ld bytes (se esperan %u bytes para raw RGB565).",
                 path, file_size, (unsigned)expected_bytes);
        ESP_LOGW(TAG, "Es probable que las imágenes estén comprimidas o en otro formato. "
                     "Reempaqueta los frames como raw RGB565 o añade un decodificador (PNG/JPEG) al firmware.");
        fclose(file);
        return false;
    }

    size_t total_pixels = (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT;
    size_t pixels_read = fread(frame_buffer, sizeof(uint16_t), total_pixels, file);
    fclose(file);
    if (pixels_read != total_pixels) {
        ESP_LOGW(TAG, "Lectura de imagen incompleta. Leídos %u de %u píxeles.", (unsigned)pixels_read, (unsigned)total_pixels);
        return false;
    }
    return true;
}

void drawText(uint16_t* frame_buffer, const char *text, int32_t x, int32_t y, uint16_t color, uint8_t scale, const uint8_t* font_data) {
    int32_t cursor_x = x;
    int32_t cursor_y = y;
    while (*text) {
        if (*text == '\n') {
            cursor_y += (FONT_HEIGHT + 2) * scale;
            cursor_x = x;
        } else {
            draw_char_scaled(frame_buffer, cursor_x, cursor_y, *text, color, scale, font_data);
            cursor_x += FONT_WIDTH * scale;
        }
        text++;
    }
}

uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

// OPTIMIZACIÓN: tft_init con delays reducidos
void tft_init(void) {
    ESP_LOGI(TAG, "Inicializando driver ST7796S en modo optimizado...");
    gpio_init();
    setup_spi(); 
    tft_reset();
    
    ESP_LOGI(TAG, "Enviando secuencia de inicialización optimizada...");
    
    ESP_LOGD(TAG, "Enviando SLPOUT (0x11)...");
    send_cmd(SLPOUT); vTaskDelay(pdMS_TO_TICKS(5));  // Reducido de 120 a 5

    ESP_LOGD(TAG, "Enviando COLMOD (0x3A) con datos 0x55...");
    send_cmd(COLMOD); send_data((uint8_t[]){0x55}, 1);

    ESP_LOGD(TAG, "Estableciendo orientación...");
    set_orientation(LANDSCAPE_INVERTED);
    
    ESP_LOGD(TAG, "Enviando Porch Control...");
    porch_control();
    
    ESP_LOGD(TAG, "Enviando GCTRL (0xB7)...");
    send_cmd(GCTRL); send_data((uint8_t[]){0x35}, 1);
    
    ESP_LOGD(TAG, "Enviando VCOMS (0xBB)...");
    send_cmd(VCOMS); send_data((uint8_t[]){0x1A}, 1);
    
    ESP_LOGD(TAG, "Enviando INVOFF (0x20)...");
    send_cmd(INVOFF);
    
    ESP_LOGD(TAG, "Enviando NORON (0x13)...");
    send_cmd(NORON);
    
    ESP_LOGD(TAG, "Enviando DISPON (0x29)...");
    send_cmd(DISPON); vTaskDelay(pdMS_TO_TICKS(5));  // Reducido de 120 a 5
    
    backlight(255);  // Máximo brillo para mejor rendimiento visual
    ESP_LOGI(TAG, "Inicialización optimizada completada a %d MHz.", SPI_CLOCK_SPEED_HZ / 1000000);
}

//==================================================================================
//==                      IMPLEMENTACIÓN DE FUNCIONES PRIVADAS OPTIMIZADAS       ==
//==================================================================================

// OPTIMIZACIÓN: setup_spi a máxima velocidad
static void setup_spi(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = TFT_MOSI, 
        .miso_io_num = TFT_MISO, 
        .sclk_io_num = TFT_SCLK,
        .quadwp_io_num = -1, 
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,  // 64KB en lugar de 320KB
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED_HZ,  // 80MHz en lugar de 40MHz
        .mode = 0,
        .spics_io_num = TFT_CS, 
        .queue_size = SPI_QUEUE_SIZE,  // Cola más grande
        .flags = SPI_DEVICE_NO_DUMMY,  // Sin bits dummy para máxima velocidad
    };
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al inicializar bus SPI: %s", esp_err_to_name(ret));
    }
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al añadir dispositivo SPI: %s", esp_err_to_name(ret));
    }
    
    // Pre-asignar buffer DMA estático para evitar malloc/free repetidos
    dma_buffer_size = SPI_MAX_TRANSFER_SIZE;
    dma_buffer = (uint8_t*)heap_caps_malloc(dma_buffer_size, MALLOC_CAP_DMA);
    if (!dma_buffer) {
        ESP_LOGE(TAG, "Failed to allocate static DMA buffer");
    } else {
        ESP_LOGI(TAG, "Allocated static DMA buffer: %u bytes", (unsigned)dma_buffer_size);
    }
}

static void send_cmd(uint8_t cmd) {
    gpio_set_level(TFT_DC, CMD_MODE);
    static spi_transaction_t t; 
    memset(&t, 0, sizeof(t));  // Limpiar estructura
    t.length = 8; 
    t.tx_buffer = &cmd;
    
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al enviar comando 0x%02X: %s", cmd, esp_err_to_name(ret));
    }
}

// OPTIMIZACIÓN: send_data con chunks más grandes
#define MAX_SPI_TRANSFER_SIZE SPI_MAX_TRANSFER_SIZE  // Usar el máximo disponible

static void send_data(const uint8_t* data, size_t size) {
    if (size == 0) return;
    gpio_set_level(TFT_DC, DATA_MODE);
    
    size_t remaining = size;
    const uint8_t* ptr = data;
    
    while (remaining > 0) {
        size_t chunk_size = (remaining > MAX_SPI_TRANSFER_SIZE) ? MAX_SPI_TRANSFER_SIZE : remaining;
        
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = chunk_size * 8;  // En bits
        t.tx_buffer = ptr;
        
        esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al enviar chunk de %u bytes: %s", (unsigned)chunk_size, esp_err_to_name(ret));
            return;
        }
        
        ptr += chunk_size;
        remaining -= chunk_size;
    }
}

static void send_word(uint16_t data){
    uint8_t buffer[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    send_data(buffer, 2);
}

// OPTIMIZACIÓN: enviar_datos_grandes_dma ultra-rápido con buffer estático
static void enviar_datos_grandes_dma(const uint8_t* data, size_t size) {
    if (size == 0 || !dma_buffer) return;
    
    gpio_set_level(TFT_DC, DATA_MODE);
    
    size_t remaining = size;
    const uint8_t* ptr = data;
    
    // Simpler: use polling transmit per chunk. This avoids queue/get result logic
    // and prevents validation errors (rxlength/rxbuffer mismatches) observed
    // when using pipelined transactions on some configurations.
    while (remaining > 0) {
        size_t chunk_size = (remaining > dma_buffer_size) ? dma_buffer_size : remaining;

        // Preparar datos en buffer DMA estático
        #ifdef TFT_SWAP_BYTES_FOR_DMA
        for (size_t i = 0; i + 1 < chunk_size; i += 2) {
            dma_buffer[i] = ptr[i + 1];
            dma_buffer[i + 1] = ptr[i];
        }
        if (chunk_size % 2) dma_buffer[chunk_size - 1] = ptr[chunk_size - 1];
        #else
        memcpy(dma_buffer, ptr, chunk_size);
        #endif

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = chunk_size * 8; // bits
        t.tx_buffer = dma_buffer;
        t.rx_buffer = NULL;
        t.rxlength = 0;

        esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al transmitir chunk DMA de %u bytes (polling): %s", (unsigned)chunk_size, esp_err_to_name(ret));
            return;
        }

        ptr += chunk_size;
        remaining -= chunk_size;
    }
}

// Helper: always send without swapping bytes (copy as-is into DMA buffer)
static void enviar_datos_grandes_dma_no_swap(const uint8_t* data, size_t size) {
    if (size == 0 || !dma_buffer) return;
    gpio_set_level(TFT_DC, DATA_MODE);
    size_t remaining = size;
    const uint8_t* ptr = data;
    while (remaining > 0) {
        size_t chunk_size = (remaining > dma_buffer_size) ? dma_buffer_size : remaining;
        memcpy(dma_buffer, ptr, chunk_size);

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = chunk_size * 8; // bits
        t.tx_buffer = dma_buffer;
        t.rx_buffer = NULL;
        t.rxlength = 0;

        esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al transmitir chunk DMA (no-swap) de %u bytes (polling): %s", (unsigned)chunk_size, esp_err_to_name(ret));
            return;
        }

        ptr += chunk_size;
        remaining -= chunk_size;
    }
}

// Helper: always send with explicit byte-swap (swap each pair before sending)
static void enviar_datos_grandes_dma_swap(const uint8_t* data, size_t size) {
    if (size == 0 || !dma_buffer) return;
    gpio_set_level(TFT_DC, DATA_MODE);
    size_t remaining = size;
    const uint8_t* ptr = data;
    while (remaining > 0) {
        size_t chunk_size = (remaining > dma_buffer_size) ? dma_buffer_size : remaining;

        for (size_t i = 0; i + 1 < chunk_size; i += 2) {
            dma_buffer[i] = ptr[i + 1];
            dma_buffer[i + 1] = ptr[i];
        }
        if (chunk_size % 2) dma_buffer[chunk_size - 1] = ptr[chunk_size - 1];

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = chunk_size * 8; // bits
        t.tx_buffer = dma_buffer;
        t.rx_buffer = NULL;
        t.rxlength = 0;

        esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al transmitir chunk DMA (swap) de %u bytes (polling): %s", (unsigned)chunk_size, esp_err_to_name(ret));
            return;
        }

        ptr += chunk_size;
        remaining -= chunk_size;
    }
}

static void gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TFT_DC) | (1ULL << TFT_RST),
        .mode = GPIO_MODE_OUTPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static void tft_reset(void) {
    gpio_set_level(TFT_RST, 0); vTaskDelay(pdMS_TO_TICKS(10));  // Reducido de 20 a 10
    gpio_set_level(TFT_RST, 1); vTaskDelay(pdMS_TO_TICKS(50));  // Reducido de 150 a 50
    send_cmd(SWRESET);
    vTaskDelay(pdMS_TO_TICKS(50));  // Reducido de 150 a 50
}

static void porch_control(void) {
    send_cmd(PORCTRL);
    send_data((uint8_t[]){0x0C, 0x0C, 0x00, 0x33, 0x33}, 5);
}

void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    if (x0 > x1) { uint16_t t=x0; x0=x1; x1=t; }
    if (y0 > y1) { uint16_t t=y0; y0=y1; y1=t; }
    if (x0 >= TFT_WIDTH) x0 = TFT_WIDTH - 1;
    if (x1 >= TFT_WIDTH) x1 = TFT_WIDTH - 1;
    if (y0 >= TFT_HEIGHT) y0 = TFT_HEIGHT - 1;
    if (y1 >= TFT_HEIGHT) y1 = TFT_HEIGHT - 1;
    ESP_LOGV(TAG, "Set Window: X=%d-%d, Y=%d-%d", x0, x1, y0, y1);  // Cambiado a LOGV para reducir spam
    send_cmd(CASET);
    send_word(x0 + X_OFFSET); send_word(x1 + X_OFFSET);
    send_cmd(RASET);
    send_word(y0 + Y_OFFSET); send_word(y1 + Y_OFFSET);
    send_cmd(RAMWR);
}

static void draw_char_scaled(uint16_t* frame_buffer, int32_t x, int32_t y, char c, uint16_t color, uint8_t scale, const uint8_t* font) {
    if (c < FONT_START || c > FONT_END || !font) return; 
    const uint8_t *glyph = &font[(c - FONT_START) * FONT_HEIGHT];
    for (int32_t row = 0; row < FONT_HEIGHT; row++) {
        uint8_t line = glyph[row];
        for (int32_t col = 0; col < FONT_WIDTH; col++) {
            if (line & (1 << (7 - col))) {
                if (scale == 1) {
                    draw_pixel(frame_buffer, x + col, y + row, color);
                } else {
                    fillRect(frame_buffer, x + col * scale, y + row * scale, scale, scale, color);
                }
            }
        }
    }
}

// Función de limpieza para liberar recursos
void cleanup_st7796s(void) {
    if (dma_buffer) {
        heap_caps_free(dma_buffer);
        dma_buffer = NULL;
        dma_buffer_size = 0;
    }
    window_set = false;
}

// --- Ultra-fast double-buffer helpers ---
static uint16_t* fb_front = NULL;
static uint16_t* fb_back = NULL;

void tft_init_fast(void) {
    // Initialize the normal driver first
    tft_init();

    // Allocate two framebuffers in PSRAM for double-buffering
    size_t fb_bytes = (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * sizeof(uint16_t);
    fb_front = (uint16_t*)heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    fb_back  = (uint16_t*)heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!fb_front || !fb_back) {
        ESP_LOGW(TAG, "tft_init_fast: failed to allocate PSRAM framebuffers");
        if (fb_front) heap_caps_free(fb_front);
        if (fb_back) heap_caps_free(fb_back);
        fb_front = fb_back = NULL;
    } else {
        // Start with front cleared
        memset(fb_front, 0, fb_bytes);
        memset(fb_back, 0, fb_bytes);
    }
}

uint16_t* get_back_buffer(void) {
    return fb_back ? fb_back : NULL;
}

void fillScreen_fast(uint16_t* frame_buffer, uint16_t color) {
    if (!frame_buffer) return;
    size_t total = (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT;
    // Fill 32-bits at a time for speed
    uint32_t color32 = ((uint32_t)color << 16) | color;
    uint32_t* ptr32 = (uint32_t*)frame_buffer;
    size_t count32 = total / 2;
    for (size_t i = 0; i < count32; ++i) ptr32[i] = color32;
    if (total % 2) frame_buffer[total - 1] = color;
}

void fillRect_fast(uint16_t* frame_buffer, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color) {
    if (!frame_buffer) return;
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT || (x + w) <= 0 || (y + h) <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if ((x + w) > TFT_WIDTH) w = TFT_WIDTH - x;
    if ((y + h) > TFT_HEIGHT) h = TFT_HEIGHT - y;
    for (int32_t row = 0; row < h; ++row) {
        uint16_t* dst = &frame_buffer[(y + row) * TFT_WIDTH + x];
        for (int32_t col = 0; col < w; ++col) dst[col] = color;
    }
}

void swap_buffers_and_flush(void) {
    if (!fb_front || !fb_back) return;
    // Swap pointers
    uint16_t* tmp = fb_front; fb_front = fb_back; fb_back = tmp;
    // Flush the new front buffer to display
    flush(fb_front);
}

void flush_ultra_fast(const uint16_t* frame_buffer) {
    if (!frame_buffer) return;
    // Use the optimized DMA path
    enviar_datos_grandes_dma((const uint8_t*)frame_buffer, (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * 2);
}

// ==== Backwards-compatible double-buffering API requested by user ====
void init_double_buffers(void) {
    if (fb_front && fb_back) return; // already initialized
    size_t fb_bytes = (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * sizeof(uint16_t);
    fb_front = (uint16_t*)heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    fb_back  = (uint16_t*)heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!fb_front || !fb_back) {
        ESP_LOGE(TAG, "❌ init_double_buffers: allocation failed");
        if (fb_front) { heap_caps_free(fb_front); fb_front = NULL; }
        if (fb_back)  { heap_caps_free(fb_back);  fb_back  = NULL; }
        return;
    }
    memset(fb_front, 0, fb_bytes);
    memset(fb_back, 0, fb_bytes);
    ESP_LOGI(TAG, "✅ Double buffers initialized: %u bytes each", (unsigned)fb_bytes);
}

uint16_t* get_draw_buffer(void) {
    if (!fb_back) return NULL;
    return fb_back;
}

void swap_and_display(void) {
    if (!fb_front || !fb_back) return;
    uint16_t* tmp = fb_front; fb_front = fb_back; fb_back = tmp;
    // Display front buffer
    flush(fb_front);
}

void cleanup_double_buffers(void) {
    if (fb_front) { heap_caps_free(fb_front); fb_front = NULL; }
    if (fb_back)  { heap_caps_free(fb_back);  fb_back  = NULL; }
    ESP_LOGI(TAG, "Double buffers freed");
}

void flush_immediate(const uint16_t* frame_buffer) {
    // Direct flush without checking window_set state
    send_cmd(RAMWR);
    enviar_datos_grandes_dma((const uint8_t*)frame_buffer, (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * 2);
}

// Special-case flush for camera frames: do not perform any byte-swap and
// send the raw buffer as-is. This avoids color-channel flipping for camera
// sources that already produce the desired byte order.
void flush_camera_frame(const uint16_t* frame_buffer) {
    if (!frame_buffer) return;
    // Ensure window is set
    if (!window_set) {
        set_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
        window_set = true;
    } else {
        send_cmd(RAMWR);
    }

    // Call the opposite path of the general-purpose DMA sender so camera
    // frames get the inverse endianness treatment. If the generic path
    // swaps bytes (TFT_SWAP_BYTES_FOR_DMA defined), we must send no-swap
    // for camera frames. Otherwise, send the explicit swap path.
#ifdef TFT_SWAP_BYTES_FOR_DMA
    // Generic path swaps -> camera needs no-swap
    enviar_datos_grandes_dma_no_swap((const uint8_t*)frame_buffer, (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * 2);
#else
    // Generic path does not swap -> camera needs swapped bytes
    enviar_datos_grandes_dma_swap((const uint8_t*)frame_buffer, (size_t)TFT_WIDTH * (size_t)TFT_HEIGHT * 2);
#endif
}
