/*
 * uORB topic: Sensor GPS
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint64_t timestamp_us;
    double latitude;        /* degrees (positive = N) */
    double longitude;       /* degrees (positive = E) */
    float altitude_msl;     /* meters */
    float ground_speed;     /* m/s */
    float course;           /* degrees true north */
    uint8_t fix_type;       /* 0=no fix, 2=2D, 3=3D */
    uint8_t satellites;
    float hdop;
} sensor_gps_t;
