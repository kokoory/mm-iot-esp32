/*
 * HaLow Communication Module - Public API
 */
#pragma once
#include "../rpc/rpc_core.h"

/**
 * Initialize Wi-Fi HaLow (morselib). MUST be called from app_main (Core 0).
 * This is blocking - waits for DHCP IP assignment.
 */
void halow_comm_init_wlan(void);

/**
 * Start communication tasks (GCS bridge, MAVLink, camera) on Core 1.
 * Call AFTER halow_comm_init_wlan() returns.
 */
void halow_comm_start(rpc_context_t *rpc);
