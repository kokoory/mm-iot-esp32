/*
 * SBUS RC Receiver Driver
 *
 * Futaba SBUS protocol: 100kbps, 8E2 (inverted UART)
 * Frame: 25 bytes = [0x0F] [16ch x 11bit packed] [flags] [0x00]
 * Period: ~7ms (analog) or ~14ms (digital)
 *
 * Channels 1-16: 11 bits each (172-1811, center ~992)
 * Flags byte: bit0 = ch17, bit1 = ch18, bit2 = frame_lost, bit3 = failsafe
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define SBUS_NUM_CHANNELS   16
#define SBUS_FRAME_SIZE     25

typedef struct {
    uint16_t channels[SBUS_NUM_CHANNELS]; /* raw 11-bit values (172-1811) */
    bool frame_lost;
    bool failsafe;
    uint64_t last_frame_us;               /* timestamp of last valid frame */
} sbus_data_t;

/**
 * Initialize SBUS receiver on configured UART.
 * Returns 0 on success, -1 on failure.
 */
int sbus_init(void);

/**
 * Read latest SBUS data (non-blocking).
 * Returns 0 if new data available, -1 if no new frame.
 */
int sbus_read(sbus_data_t *data);

/**
 * Convert raw SBUS channel (172-1811) to normalized float (-1.0 to +1.0).
 */
float sbus_channel_to_float(uint16_t raw);
