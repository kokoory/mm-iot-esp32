/*
 * uORB topic: Actuator Controls
 * Normalized control outputs from the flight controller
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint64_t timestamp_us;
    float roll;         /* -1.0 to 1.0 */
    float pitch;        /* -1.0 to 1.0 */
    float yaw;          /* -1.0 to 1.0 */
    float collective;   /* -1.0 to 1.0 (helicopter collective pitch) */
    float throttle;     /* 0.0 to 1.0 (main rotor throttle) */
    bool  land_detected; /* true when flight_ctrl detects touchdown */
} actuator_controls_t;
