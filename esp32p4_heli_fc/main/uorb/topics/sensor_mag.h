/*
 * uORB topic: Sensor Magnetometer
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint64_t timestamp_us;
    float mag_x, mag_y, mag_z;  /* Gauss */
} sensor_mag_t;
