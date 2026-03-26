/*
 * MIPI-CSI Camera + HW JPEG/H.264 Encoder for ESP32-P4
 *
 * Pipelines:
 *   MJPEG: OV5647 → MIPI-CSI → ISP (RAW8→RGB565) → HW JPEG → HTTP /
 *   H.264: OV5647 → MIPI-CSI → ISP (RAW8→RGB565) → HW H.264 → UDP RTP :5600
 *
 * Hardware: Waveshare ESP32-P4-WIFI6 + OV5647 (RPi Camera v1)
 */

#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the MIPI-CSI camera and encoder pipeline.
 *
 * Configures: LDO → SCCB/I2C → OV5647 → CSI → ISP → JPEG + H.264 encoders.
 * Also creates a UDP RTP socket for H.264 streaming (broadcast port 5600).
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t camera_h264_init(void);

/**
 * Start the HTTP server and UDP RTP streaming.
 *
 * HTTP endpoints:
 *   GET /         - MJPEG stream (HW JPEG, for browser viewing)
 *   GET /status   - JSON status (fps, resolution, encoder info)
 *
 * UDP RTP:
 *   H.264 NALs broadcast to port 5600 (QGC/VLC)
 *   VLC: rtp://@:5600  or use SDP file
 *   QGC: Video Source = UDP h.264, port 5600
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
