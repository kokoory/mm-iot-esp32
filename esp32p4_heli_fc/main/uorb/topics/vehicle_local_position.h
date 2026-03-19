/*
 * uORB topic: Vehicle Local Position
 * Estimated local position and velocity
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint64_t timestamp_us;
    float x, y, z;          /* meters, NED frame */
    float vx, vy, vz;       /* m/s, NED frame */
    float alt;               /* altitude above home, meters (positive up) */
    float climb_rate;        /* m/s (positive up) */
    bool  z_valid;
    bool  v_z_valid;
} vehicle_local_position_t;
