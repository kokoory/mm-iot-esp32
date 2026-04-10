/*
 * FLIR Lepton 3.x Thermal Camera Driver for ESP32-P4
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interfaces:
 *   - VoSPI (SPI3_HOST, 20MHz): 160x120 thermal frame readout
 *   - CCI (I2C): Lepton command & control interface
 *
 * PureThermal Breakout Board pinout:
 *   Pin 1: SCL   Pin 2: SDA   Pin 3: VIN   Pin 4: GND   Pin 5: CLK
 *   Pin 6: MISO  Pin 7: MOSI  Pin 8: CS    Pin 9: VSYNC Pin 10: EN
 *
 * Frame format (Lepton 3.x, VoSPI):
 *   4 segments x 60 packets = 240 packets per frame
 *   Each packet: 4B header + 160B payload (80 pixels x 16-bit)
 *   Segment number in packet 20's ID word (bits 4-6)
 *   Total: 160 x 120 pixels, 14-bit radiometric data
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Pin Configuration ─────────────────────────────────── */
/* SPI (VoSPI) - uses SPI3_HOST (SPI2 is used by HaLow) */
#define LEPTON_SPI_CLK      48
#define LEPTON_SPI_MISO     49
#define LEPTON_SPI_CS       50
#define LEPTON_SPI_FREQ_HZ  (20 * 1000 * 1000)  /* 20 MHz max */

/* I2C (CCI) - Command & Control Interface */
#define LEPTON_I2C_SCL      46
#define LEPTON_I2C_SDA      47
#define LEPTON_I2C_FREQ_HZ  400000
#define LEPTON_I2C_ADDR     0x2A    /* 7-bit address */

/* Control pins */
#define LEPTON_VSYNC_PIN    21
#define LEPTON_EN_PIN       20

/* ── Frame Constants ───────────────────────────────────── */
#define LEPTON_WIDTH        160
#define LEPTON_HEIGHT       120
#define LEPTON_PIXELS       (LEPTON_WIDTH * LEPTON_HEIGHT)  /* 19200 */
#define LEPTON_FRAME_SIZE   (LEPTON_PIXELS * 2)             /* 38400 bytes (16-bit) */

/* VoSPI protocol constants */
#define LEPTON_PACKET_SIZE      164     /* 4B header + 160B payload */
#define LEPTON_PAYLOAD_SIZE     160     /* 80 pixels x 2 bytes */
#define LEPTON_PACKETS_PER_SEG  60
#define LEPTON_SEGMENTS         4       /* Lepton 3.x: 4 segments */

/* ── Data Types ────────────────────────────────────────── */

/**
 * Lepton telemetry data (from frame footer or CCI)
 */
typedef struct {
    float fpa_temp_k;           /* FPA (sensor) temperature in Kelvin */
    float housing_temp_k;       /* Housing temperature in Kelvin */
    uint32_t frame_count;       /* Lepton internal frame counter */
    uint32_t status;            /* Lepton status word */
} lepton_telemetry_t;

/**
 * Lepton driver statistics
 */
typedef struct {
    uint32_t frames_captured;   /* Total frames successfully captured */
    uint32_t sync_errors;       /* VoSPI sync lost count */
    uint32_t crc_errors;        /* Packet CRC errors */
    uint32_t discard_packets;   /* Discard packets received */
    float fps;                  /* Current frames per second */
    float capture_time_ms;      /* Average frame capture time */
} lepton_stats_t;

/* ── API Functions ─────────────────────────────────────── */

/**
 * Initialize the Lepton thermal camera driver.
 *
 * Sets up:
 *   - SPI3 bus for VoSPI frame readout
 *   - I2C for CCI command interface
 *   - EN pin to power on the Lepton module
 *   - Frame buffer in PSRAM
 *
 * @return ESP_OK on success
 */
esp_err_t lepton_init(void);

/**
 * Start the Lepton capture task.
 * Continuously reads frames via VoSPI in background.
 *
 * @return ESP_OK on success
 */
esp_err_t lepton_start(void);

/**
 * Get pointer to the latest complete thermal frame.
 * Frame data is 16-bit raw radiometric (14-bit effective).
 * Buffer is double-buffered; pointer remains valid until next call.
 *
 * @param[out] frame_out    Pointer to frame data (160x120 uint16_t)
 * @param[out] frame_len    Frame size in bytes
 * @return ESP_OK if frame available, ESP_ERR_NOT_FOUND if no frame yet
 */
esp_err_t lepton_get_frame(const uint16_t **frame_out, size_t *frame_len);

/**
 * Get driver statistics.
 */
void lepton_get_stats(lepton_stats_t *stats);

/**
 * Get current FPS.
 */
float lepton_get_fps(void);

/**
 * Run AGC (Automatic Gain Control) on raw frame → 8-bit grayscale.
 * Useful for display/streaming as pseudocolor or grayscale.
 *
 * @param raw16     Input: 160x120 raw 16-bit frame
 * @param gray8     Output: 160x120 8-bit grayscale
 * @param len       Number of pixels (19200)
 */
void lepton_agc_linear(const uint16_t *raw16, uint8_t *gray8, size_t len);

#ifdef __cplusplus
}
#endif
