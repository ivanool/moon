// camera_display.h - public API for camera display helpers
#ifndef CAMERA_DISPLAY_H
#define CAMERA_DISPLAY_H

#include "esp_err.h"

// Start camera display in a separate task (optional)
void camera_display_start(void);

// Initialize camera for caller-driven operation (returns ESP_OK on success)
esp_err_t camera_display_init(void);

// Perform one capture->blit->display iteration. Returns ESP_OK on success.
esp_err_t camera_display_step(void);

#endif // CAMERA_DISPLAY_H
