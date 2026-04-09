/*
 * USB Webcam (Logitech C920) — MJPEG over HTTP/TCP
 *
 * Pipeline: USB UVC → MJPEG frames → double buffer → HTTP /mjpeg (port 81)
 *
 * Hardware: Logitech C920 (or any UVC-compatible USB webcam)
 */

#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize USB webcam pipeline.
 *
 * Configures: USB Host → UVC driver → MJPEG frame capture.
 * Webcam outputs MJPEG natively — no ISP or HW encoder needed.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t camera_h264_init(void);

/**
 * Start the HTTP server for video and thermal streaming.
 *
 * HTTP endpoints:
 *   GET /         - Landing page with embedded MJPEG viewer
 *   GET /status   - JSON status (fps, resolution, encoder info)
 *   GET /thermal  - Thermal camera viewer (FLIR Lepton)
 *   GET /thermal/raw - Raw Y16 thermal data
 *
 * MJPEG server (port 81):
 *   GET /mjpeg    - MJPEG multipart stream (browser/VLC)
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
