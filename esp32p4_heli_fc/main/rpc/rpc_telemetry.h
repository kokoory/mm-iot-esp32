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
#include "uorb/topics/sensor_imu.h"
#include "uorb/topics/sensor_mag.h"
#include "uorb/topics/sensor_baro.h"

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

/**
 * Send raw IMU data as RPC telemetry (for HIGHRES_IMU and VIBRATION).
 */
int rpc_telem_send_imu_raw(rpc_context_t *ctx, const sensor_imu_t *imu);

/**
 * Send raw magnetometer data as RPC telemetry.
 */
int rpc_telem_send_mag_raw(rpc_context_t *ctx, const sensor_mag_t *mag);

/**
 * Send raw barometer data as RPC telemetry.
 */
int rpc_telem_send_baro_raw(rpc_context_t *ctx, const sensor_baro_t *baro);

/**
 * Send estimator status as RPC telemetry.
 */
int rpc_telem_send_estimator(rpc_context_t *ctx,
                             uint16_t flags,
                             float vel_ratio, float pos_horiz_ratio,
                             float pos_vert_ratio,
                             float pos_horiz_accuracy, float pos_vert_accuracy);
