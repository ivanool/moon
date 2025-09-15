#ifndef ST7796S_H
#define ST7796S_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//==================================================================================
//==                              CONFIGURACIÓN GENERAL                           ==
//==   Modifica estos valores para que coincidan con tu hardware y recursos       ==
//==================================================================================

// Pines usados para ST7796S en ESP32-S3
#define TFT_CS    1    // CS - Chip Select
#define TFT_DC    2    // DC - Data/Command
#define TFT_RST   3    // RST - Reset
#define TFT_SCLK  7    // SCL - SPI Clock (ya configurado en ESP32-S3)
#define TFT_MOSI  9    // SDA - SPI MOSI (ya configurado en ESP32-S3)
#define TFT_MISO  8    // MISO (ya configurado en ESP32-S3)
#define TFT_BL    4    // Backlight
#define TFT_WIDTH 480
#define TFT_HEIGHT 320

// --- Configuración del Backlight (LEDC PWM) ---
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Resolución de 8 bits (0-255)
#define LEDC_FREQUENCY          (5000)           // Frecuencia de 5 kHz

// --- Offsets del Controlador (si la imagen aparece desplazada) ---
#define X_OFFSET    0
#define Y_OFFSET    0

// Si tu pantalla muestra colores intercambiados (ej. rojo aparece azul),
// activa el swap de bytes por chunk en la transmisión DMA. Esto fuerza
// a enviar cada par de bytes en orden inverso (LSB<->MSB).
#define TFT_SWAP_BYTES_FOR_DMA 1

// Nombre de la partición SPIFFS que contiene los archivos empaquetados.
// Debe coincidir con la entrada en `partitions.csv` y con la llamada
// a spiffs_create_partition_image(...) en CMake.
#define SPIFFS_PARTITION_LABEL "spiffs_image"

#define FONT_FILE       "/spiffs/font.bin" // Ruta al archivo de la fuente (montado en /spiffs)
#define FONT_HEIGHT     8  // Alto de cada caracter en píxeles
#define FONT_WIDTH      5  // Ancho de cada caracter en píxeles
#define FONT_START      32 // Caracter ASCII inicial en la fuente (espacio)
#define FONT_END        127// Caracter ASCII final en la fuente


//==================================================================================
//==                                 TIPOS DE DATOS PÚBLICOS                      ==
//==================================================================================

/**
 * @brief Define las 4 orientaciones posibles de la pantalla.
 */
typedef enum {
    PORTRAIT = 0,           // Rotación 0 grados (Vertical)
    LANDSCAPE = 1,          // Rotación 90 grados (Horizontal)
    PORTRAIT_INVERTED = 2,  // Rotación 180 grados (Vertical Invertido)
    LANDSCAPE_INVERTED = 3  // Rotación 270 grados (Horizontal Invertido)
} orientation_t;


//==================================================================================
//==                             API PÚBLICA DE LA LIBRERÍA                       ==
//==================================================================================

// --- Funciones de Inicialización y Sistema de Archivos ---

/**
 * @brief Monta el sistema de archivos SPIFFS.
 * Debe llamarse al inicio de la aplicación, antes de acceder a cualquier archivo.
 */
void mount_spiffs(void);

/**
 * @brief Carga los datos de una fuente desde un archivo a un buffer en memoria.
 * @param font_data Puntero al buffer donde se cargarán los datos de la fuente.
 */
void load_font(uint8_t *font_data);

/**
 * @brief Inicializa por completo el controlador de la pantalla ST7796S.
 * ESTA FUNCIÓN DEBE SER LLAMADA DESPUÉS de mount_spiffs si se van a usar recursos.
 */
void tft_init(void);

/* Ultra-fast (optional) helpers used by benchmarks/examples. These are thin
 * wrappers around the normal API that provide a simple double-buffering
 * helper and faster buffer fill routines. They are optional; if you don't
 * use them the regular API (tft_init, flush, fillScreen, fillRect) works.
 */
void tft_init_fast(void);
uint16_t* get_back_buffer(void);
void fillScreen_fast(uint16_t* frame_buffer, uint16_t color);
void fillRect_fast(uint16_t* frame_buffer, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);
void swap_buffers_and_flush(void);
void flush_ultra_fast(const uint16_t* frame_buffer);

// Double-buffering API (simple, PSRAM-backed)
void init_double_buffers(void);
uint16_t* get_draw_buffer(void);
void swap_and_display(void);
void cleanup_double_buffers(void);
void flush_immediate(const uint16_t* frame_buffer);

/**
 * @brief Special-case flush for camera frames: sends the framebuffer without
 * performing the driver-wide byte-swap-for-DMA. Use this for camera-sourced
 * buffers to preserve correct byte order.
 */
void flush_camera_frame(const uint16_t* frame_buffer);

// --- SPIFFS preload API ----------------------------------------------------
/**
 * @brief Preload raw RGB565 frames from a directory in SPIFFS into PSRAM.
 * @param base_dir Directory where frames are stored (e.g. "/spiffs").
 *                 Frames are expected to be named "1.bin", "2.bin", ...
 * @param max_preload Maximum number of frames to attempt to preload.
 * @return Number of frames successfully preloaded (0 on failure).
 */
int preload_spiffs_frames(const char* base_dir, int max_preload);

/**
 * @brief Get pointer to a preloaded frame buffer (read-only pointer).
 * @param index Frame index [0..count-1]
 * @return Pointer to the raw RGB565 buffer or NULL if index invalid.
 */
const uint8_t* st7796s_get_preloaded_frame(int index);

/**
 * @brief Number of frames currently preloaded.
 */
int st7796s_get_preloaded_count(void);

/**
 * @brief Free any preloaded frames and release PSRAM.
 */
void st7796s_free_preloaded_frames(void);

// Backwards-compatible double-buffering API requested by some examples
void init_double_buffers(void);
uint16_t* get_draw_buffer(void);
void swap_and_display(void);
void cleanup_double_buffers(void);
void flush_immediate(const uint16_t* frame_buffer);


// --- Funciones de Control de la Pantalla ---

/**
 * @brief Envía el contenido completo de un framebuffer a la pantalla usando DMA.
 * @param frame_buffer Puntero al framebuffer (debe tener un tamaño de TFT_WIDTH * TFT_HEIGHT).
 */
void flush(const uint16_t* frame_buffer);

/**
 * @brief Establece la orientación de la pantalla (rotación).
 * @param orientation La orientación deseada (PORTRAIT, LANDSCAPE, etc.).
 */
void set_orientation(orientation_t orientation);

/**
 * @brief Ajusta el brillo de la retroiluminación.
 * @param duty Valor de brillo de 0 (apagado) a 255 (máximo).
 */
void backlight(uint8_t duty);


// --- Funciones de Dibujo (Operan sobre el framebuffer en memoria) ---

/**
 * @brief Rellena todo el framebuffer con un color específico.
 * @param frame_buffer Puntero al framebuffer a modificar.
 * @param color Color en formato RGB565.
 */
void fillScreen(uint16_t* frame_buffer, uint16_t color);

/**
 * @brief Dibuja un píxel en una coordenada específica del framebuffer.
 * @param frame_buffer Puntero al framebuffer a modificar.
 * @param x Coordenada X del píxel.
 * @param y Coordenada Y del píxel.
 * @param color Color del píxel en formato RGB565.
 */
void draw_pixel(uint16_t* frame_buffer, int32_t x, int32_t y, uint16_t color);

/**
 * @brief Dibuja un rectángulo relleno en el framebuffer.
 * @param frame_buffer Puntero al framebuffer a modificar.
 * @param x Coordenada X de la esquina superior izquierda.
 * @param y Coordenada Y de la esquina superior izquierda.
 * @param w Ancho del rectángulo.
 * @param h Alto del rectángulo.
 * @param color Color del relleno en formato RGB565.
 */
void fillRect(uint16_t* frame_buffer, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);

/**
 * @brief Carga una imagen desde un archivo directamente al framebuffer.
 * @param frame_buffer Puntero al framebuffer donde se cargará la imagen.
 * @param path Ruta del archivo de imagen (ej. "/spiffs/mi_imagen.img").
 * @return true si la carga fue exitosa, false en caso de error.
 */
bool drawImage(uint16_t* frame_buffer, const char* path);

/**
 * @brief Dibuja texto escalado en el framebuffer.
 * @param frame_buffer Puntero al framebuffer a modificar.
 * @param text La cadena de texto a dibujar.
 * @param x Coordenada X donde comenzará el texto.
 * @param y Coordenada Y donde comenzará el texto.
 * @param color Color del texto.
 * @param scale Factor de escala (1 = tamaño normal, 2 = doble tamaño, etc.).
 * @param font_data Puntero al buffer que contiene los datos de la fuente.
 */
void drawText(uint16_t* frame_buffer, const char *text, int32_t x, int32_t y, uint16_t color, uint8_t scale, const uint8_t* font_data);

void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

// --- Funciones de Utilidad ---

/**
 * 
 * @brief Convierte un color de formato RGB888 (24 bits) a RGB565 (16 bits).
 * @param r Componente rojo (0-255).
 * @param g Componente verde (0-255).
 * @param b Componente azul (0-255).
 * @return El color convertido a formato uint16_t.
 */
uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b);

/**
 * Backwards-compatible alias: some code calls bgr888_to_rgb565
 * (B,G,R order). Provide a small inline wrapper so older callers
 * link without changing the implementation.
 */
static inline uint16_t bgr888_to_rgb565(uint8_t b, uint8_t g, uint8_t r) {
    return rgb888_to_rgb565(r, g, b);
}

#endif // ST7796S_H