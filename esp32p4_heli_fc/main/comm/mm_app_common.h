/*
 * Copyright 2023 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

void app_wlan_init(void);
void app_wlan_start(void);
void app_wlan_stop(void);
bool app_wlan_is_connected(void);
void app_wlan_arp_send(void);

/* TX flow control state (updated by Morse Micro callback) */
bool app_wlan_tx_is_paused(void);
uint32_t app_wlan_tx_pause_count(void);

/* Link diagnostics */
int32_t app_wlan_get_rssi(void);
void app_wlan_print_link_stats(void);

/**
 * Get current link quality info for adaptive rate control.
 * Returns estimated usable throughput in bytes/sec based on
 * current MCS rate, bandwidth, and packet loss.
 */
typedef struct {
    uint8_t  mcs;          /* Current MCS level (0-9), 0xFF if unknown */
    uint8_t  bw_mhz;       /* Channel bandwidth in MHz (1, 2, 4) */
    uint8_t  sgi;          /* 1=short guard interval, 0=long */
    uint8_t  loss_pct;     /* Packet loss percentage (0-100) */
    uint32_t throughput_bps; /* Estimated usable throughput in bytes/sec */
} app_wlan_link_quality_t;

void app_wlan_get_link_quality(app_wlan_link_quality_t *out);
