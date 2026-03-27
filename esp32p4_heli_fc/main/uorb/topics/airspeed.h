/*
 * Airspeed sensor topic (from differential pressure sensor + pitot tube)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint64_t timestamp_us;
    float differential_pressure_pa;   /* Raw differential pressure (Pa) */
    float indicated_airspeed;         /* IAS in m/s */
    float temperature;                /* Ambient temp from baro for TAS correction (deg C) */
    bool  valid;                      /* Sensor healthy and reading plausible */
} airspeed_t;
