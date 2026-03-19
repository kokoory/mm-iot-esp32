/*
 * uORB topic: Vehicle Attitude
 * Estimated attitude as quaternion and Euler angles
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint64_t timestamp_us;
    float q[4];             /* quaternion [w, x, y, z] */
    float roll;             /* radians */
    float pitch;            /* radians */
    float yaw;              /* radians */
    float rollspeed;        /* rad/s */
    float pitchspeed;       /* rad/s */
    float yawspeed;         /* rad/s */
} vehicle_attitude_t;
