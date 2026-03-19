/*
 * uORB topic: Vehicle Status
 * Overall vehicle state including arm state, flight mode, and failsafe
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../../common/flight_modes.h"

typedef struct {
    uint64_t timestamp_us;
    arm_state_t arm_state;
    flight_mode_t flight_mode;
    failsafe_state_t failsafe;
    bool sensor_imu_ok;
    bool sensor_baro_ok;
    bool sensor_mag_ok;
    bool sensor_gps_ok;
    bool rc_ok;
    bool battery_ok;
} vehicle_status_t;
