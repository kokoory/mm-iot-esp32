/*
 * MIPI-CSI Camera + HW JPEG Encoder for ESP32-P4
 *
 * Pipeline: OV5647 → MIPI-CSI → ISP (RAW8→RGB565) → HW JPEG → HTTP Stream
 *
 * Reference hardware: Waveshare ESP32-P4-WIFI6 + OV5647 (RPi Camera v1)
 */

#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the MIPI-CSI camera and HW JPEG encoder pipeline.
 *
 * Configures: LDO → SCCB/I2C → OV5647 sensor → CSI → ISP → JPEG encoder.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t camera_h264_init(void);

/**
 * Start the HTTP server for MJPEG video streaming.
 *
 * Endpoints:
 *   GET /         - MJPEG stream (HW JPEG encoded frames)
 *   GET /status   - JSON status (fps, resolution, pipeline info)
 *
 * @return HTTP server handle, or NULL on failure.
 */
httpd_handle_t camera_stream_server_start(void);

/**
 * Get the current camera frame rate.
 * @return Frames per second.
 */
float camera_get_fps(void);

#ifdef __cplusplus
}
#endif
