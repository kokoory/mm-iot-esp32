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
