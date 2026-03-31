/*
 * Thermal Camera (FLIR Lepton via PureThermal USB UVC)
 *
 * Receives thermal frames via USB Host UVC.
 * PureThermal presents Lepton as a standard UVC webcam.
 * Frame format: Y16 (16-bit per pixel, raw radiometric data)
 *
 * Resolution depends on Lepton model:
 *   Lepton 2/3:   80x60
 *   Lepton 3.5: 160x120 (or 80x60 default)
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* Callback invoked on each new thermal frame */
typedef void (*thermal_frame_cb_t)(const uint16_t *frame_data, size_t len, void *user_ctx);

/**
 * Initialize USB Host and UVC driver, open PureThermal stream.
 * Resolution and format are auto-negotiated with the device.
 */
esp_err_t thermal_camera_init(thermal_frame_cb_t frame_cb, void *user_ctx);

/** Start streaming thermal frames. */
esp_err_t thermal_camera_start(void);

/** Stop streaming. */
esp_err_t thermal_camera_stop(void);

/**
 * Get latest frame (thread-safe copy).
 * buf must be at least thermal_camera_frame_size() bytes.
 * Returns true if a valid frame was copied.
 */
bool thermal_camera_get_frame(uint16_t *buf);

/** Check if thermal camera is connected and streaming. */
bool thermal_camera_is_active(void);

/** Get negotiated resolution. Returns 0 before init. */
unsigned thermal_camera_width(void);
unsigned thermal_camera_height(void);
size_t thermal_camera_frame_size(void);
