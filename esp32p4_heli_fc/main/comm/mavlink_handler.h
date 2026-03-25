/*
 * MAVLink Protocol Handler
 *
 * Bridges RPC telemetry from Core 0 (FC) to MAVLink messages
 * sent via UDP to the Ground Control Station, and receives
 * MAVLink commands from GCS and forwards them as RPC commands.
 */
#pragma once
#include "../rpc/rpc_core.h"

/**
 * Configuration for MAVLink handler telemetry rates (in Hz).
 */
typedef struct {
    uint8_t heartbeat_hz;   /* Heartbeat rate (default 1) */
    uint8_t attitude_hz;    /* Attitude rate (default 10) */
    uint8_t gps_hz;         /* GPS rate (default 5) */
    uint8_t battery_hz;     /* Battery + SYS_STATUS rate (default 2) */
    uint8_t vfr_hud_hz;     /* VFR HUD rate (default 2) */
} mavlink_handler_config_t;

/**
 * Initialize the MAVLink handler.
 * Must be called after rpc_init() and gcs_bridge_init().
 */
void mavlink_handler_init(rpc_context_t *ctx, const mavlink_handler_config_t *config);

/**
 * MAVLink handler task - main loop for MAVLink processing.
 * Should be pinned to Core 1. Receives RPC telemetry, encodes
 * MAVLink messages, sends via GCS bridge. Also receives MAVLink
 * commands and forwards them as RPC commands to Core 0.
 *
 * param: pointer to rpc_context_t
 */
void mavlink_handler_task(void *param);
