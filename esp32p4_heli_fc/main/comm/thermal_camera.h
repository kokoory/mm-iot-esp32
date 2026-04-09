/*
 * USB Camera (Logitech C920 or compatible UVC webcam)
 *
 * Receives MJPEG frames via USB Host UVC driver.
 * Frames are double-buffered in PSRAM for thread-safe access.
 *
 * Previously: FLIR Lepton via PureThermal (Y16 160x120 @9fps)
 * Now: Logitech C920 (MJPEG 640x480 @15fps)
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* Callback invoked on each new frame */
typedef void (*thermal_frame_cb_t)(const uint16_t *frame_data, size_t len, void *user_ctx);

/**
 * Initialize USB Host and UVC driver, open webcam stream.
 * Resolution and format are auto-negotiated with the device.
 */
esp_err_t thermal_camera_init(thermal_frame_cb_t frame_cb, void *user_ctx);

/** Start streaming frames. */
esp_err_t thermal_camera_start(void);

/** Stop streaming. */
esp_err_t thermal_camera_stop(void);

/**
 * Get latest frame (thread-safe copy) — legacy Y16 API.
 * buf must be at least thermal_camera_frame_size() bytes.
 * Returns true if a valid frame was copied.
 */
bool thermal_camera_get_frame(uint16_t *buf);

/**
 * Get latest MJPEG frame (thread-safe copy).
 * Returns actual JPEG data length in *out_len.
 */
bool thermal_camera_get_jpeg(uint8_t *buf, size_t buf_size, size_t *out_len);

/** Check if USB camera is connected and streaming. */
bool thermal_camera_is_active(void);

/** Get negotiated resolution. Returns 0 before init. */
unsigned thermal_camera_width(void);
unsigned thermal_camera_height(void);
size_t thermal_camera_frame_size(void);
