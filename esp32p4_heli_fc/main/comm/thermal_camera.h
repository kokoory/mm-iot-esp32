/*
 * Thermal Camera (FLIR Lepton 3.5 via PureThermal USB UVC)
 *
 * Receives 160x120 @ 9fps thermal frames via USB Host UVC.
 * PureThermal presents Lepton as a standard UVC webcam.
 *
 * Frame format: Y16 (16-bit per pixel, raw 14-bit radiometric data)
 * Frame size: 160 * 120 * 2 = 38,400 bytes
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#define THERMAL_WIDTH   160
#define THERMAL_HEIGHT  120
#define THERMAL_FPS     9
#define THERMAL_FRAME_SIZE  (THERMAL_WIDTH * THERMAL_HEIGHT * 2)  /* Y16 = 2 bytes/pixel */

/* Callback invoked on each new thermal frame */
typedef void (*thermal_frame_cb_t)(const uint16_t *frame_data, size_t len, void *user_ctx);

/**
 * Initialize USB Host and UVC driver, open PureThermal stream.
 * frame_cb: called from USB task context with each new frame
 * Returns ESP_OK on success.
 */
esp_err_t thermal_camera_init(thermal_frame_cb_t frame_cb, void *user_ctx);

/**
 * Start streaming thermal frames.
 */
esp_err_t thermal_camera_start(void);

/**
 * Stop streaming.
 */
esp_err_t thermal_camera_stop(void);

/**
 * Get latest frame (thread-safe copy).
 * buf must be at least THERMAL_FRAME_SIZE bytes.
 * Returns true if a valid frame was copied.
 */
bool thermal_camera_get_frame(uint16_t *buf);

/**
 * Check if thermal camera is connected and streaming.
 */
bool thermal_camera_is_active(void);
