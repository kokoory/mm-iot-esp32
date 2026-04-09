/*
 * Thermal Camera — FLIR Lepton 3.5 via SPI (VoSPI) + I2C (CCI)
 *
 * SparkFun Lepton Breakout Board:
 *   VoSPI (video): SPI3_HOST shared with IMU/MAG, 10MHz Mode 3
 *   CCI (control): I2C0, addr 0x2A
 *   Resolution: 160x120, Grey14 (2 bytes/pixel), ~9fps
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* Callback invoked on each new thermal frame */
typedef void (*thermal_frame_cb_t)(const uint16_t *frame_data, size_t len, void *user_ctx);

/**
 * Initialize SPI + I2C, reset Lepton, start VoSPI reader task.
 */
esp_err_t thermal_camera_init(thermal_frame_cb_t frame_cb, void *user_ctx);

/** Start streaming thermal frames. */
esp_err_t thermal_camera_start(void);

/** Stop streaming. */
esp_err_t thermal_camera_stop(void);

/**
 * Get latest Y16 frame (thread-safe copy).
 * buf must be at least thermal_camera_frame_size() bytes.
 * Returns true if a valid frame was copied.
 */
bool thermal_camera_get_frame(uint16_t *buf);

/**
 * Get latest MJPEG frame — not applicable for Lepton (returns false).
 * Provided for API compatibility with USB camera path.
 */
bool thermal_camera_get_jpeg(uint8_t *buf, size_t buf_size, size_t *out_len);

/** Check if Lepton is connected and streaming. */
bool thermal_camera_is_active(void);

/** Get resolution (160x120). */
unsigned thermal_camera_width(void);
unsigned thermal_camera_height(void);
size_t thermal_camera_frame_size(void);
