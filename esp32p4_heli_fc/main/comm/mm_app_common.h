/*
 * Copyright 2023 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>

void app_wlan_init(void);
void app_wlan_start(void);
void app_wlan_stop(void);
bool app_wlan_is_connected(void);
void app_wlan_arp_send(void);
