/*
 * MIPI-CSI Camera + H.264 Encoder for ESP32-P4
 *
 * Uses the ESP32-P4 hardware H.264 encoder to compress MIPI-CSI camera
 * frames and stream them over HTTP via Wi-Fi HaLow.
 *
 * Reference hardware: Waveshare ESP32-P4-WIFI6 with MIPI-CSI camera
 * (e.g., OV5647, SC2336, or similar 2-lane MIPI camera)
 */

#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the MIPI-CSI camera and H.264 encoder.
 *
 * Configures the ISP pipeline: MIPI-CSI → ISP → H.264 encoder.
 * The H.264 output is stored in a ring buffer for streaming.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t camera_h264_init(void);

/**
 * Start the HTTP server for H.264 video streaming.
 *
 * Endpoints:
 *   GET /         - MJPEG fallback stream (ISP JPEG output)
 *   GET /h264     - Raw H.264 NAL unit stream
 *   GET /status   - JSON status (fps, bitrate, resolution)
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
