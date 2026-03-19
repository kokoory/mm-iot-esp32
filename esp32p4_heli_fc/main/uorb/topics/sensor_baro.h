/*
 * uORB topic: Sensor Barometer
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint64_t timestamp_us;
    float pressure_pa;      /* Pascal */
    float temperature;      /* degrees C */
    float altitude_msl;     /* meters (calculated from pressure) */
} sensor_baro_t;
