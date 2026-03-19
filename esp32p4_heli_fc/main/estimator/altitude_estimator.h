/*
 * Altitude Estimator
 * Complementary filter fusing barometer altitude and accelerometer
 */
#pragma once

#include <stdbool.h>

typedef struct {
    float altitude;         /* estimated altitude in meters (positive up) */
    float climb_rate;       /* estimated climb rate in m/s (positive up) */
    float alpha;            /* complementary filter coefficient (0.98 default) */
    float baro_alt_prev;    /* previous barometer altitude */
    bool  initialized;      /* true after first baro reading */
    float alt_offset;       /* barometer altitude offset (set at init) */
} alt_estimator_t;

/**
 * Initialize the altitude estimator.
 */
void alt_estimator_init(alt_estimator_t *est);

/**
 * Update with new barometer altitude reading.
 * @param alt_baro Barometer altitude in meters MSL
 */
void alt_estimator_update_baro(alt_estimator_t *est, float alt_baro);

/**
 * Update with world-frame Z acceleration and integrate.
 * @param accel_z World-frame vertical acceleration in m/s^2 (positive up, gravity removed)
 * @param dt      Time step in seconds
 */
void alt_estimator_update_accel(alt_estimator_t *est, float accel_z, float dt);

/**
 * Get current altitude and climb rate estimates.
 */
void alt_estimator_get(const alt_estimator_t *est, float *altitude, float *climb_rate);
