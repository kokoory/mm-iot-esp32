/*
 * RPC Telemetry Helpers - Package uORB data into RPC telemetry messages
 *
 * These helpers are used by Core 0 (FC) to send flight data to Core 1 (HaLow).
 */
#pragma once
#include "rpc_core.h"
#include "uorb/topics/vehicle_attitude.h"
#include "uorb/topics/sensor_gps.h"
#include "uorb/topics/vehicle_local_position.h"
#include "uorb/topics/battery_status.h"
#include "uorb/topics/vehicle_status.h"

/**
 * Send attitude data as RPC telemetry.
 * Returns 0 on success, -1 on failure.
 */
int rpc_telem_send_attitude(rpc_context_t *ctx, const vehicle_attitude_t *att);

/**
 * Send GPS data as RPC telemetry.
 * Returns 0 on success, -1 on failure.
 */
int rpc_telem_send_gps(rpc_context_t *ctx, const sensor_gps_t *gps);

/**
 * Send altitude data from local position as RPC telemetry.
 * Returns 0 on success, -1 on failure.
 */
int rpc_telem_send_altitude(rpc_context_t *ctx, const vehicle_local_position_t *pos);

/**
 * Send battery status as RPC telemetry.
 * Returns 0 on success, -1 on failure.
 */
int rpc_telem_send_battery(rpc_context_t *ctx, const battery_status_t *bat);

/**
 * Send vehicle status as RPC telemetry.
 * Returns 0 on success, -1 on failure.
 */
int rpc_telem_send_status(rpc_context_t *ctx, const vehicle_status_t *status);
