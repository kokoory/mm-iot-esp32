/*
 * Altitude Estimator Implementation
 *
 * Complementary filter:
 *   altitude = alpha * (altitude + climb_rate * dt) + (1 - alpha) * baro_altitude
 *   climb_rate = alpha * (climb_rate + accel_z * dt) + (1 - alpha) * baro_climb_rate
 *
 * The accelerometer provides high-frequency response while the barometer
 * provides low-frequency drift correction.
 */

#include "altitude_estimator.h"

#define DEFAULT_ALPHA  0.98f

void alt_estimator_init(alt_estimator_t *est)
{
    est->altitude = 0.0f;
    est->climb_rate = 0.0f;
    est->alpha = DEFAULT_ALPHA;
    est->baro_alt_prev = 0.0f;
    est->initialized = false;
    est->alt_offset = 0.0f;
}

void alt_estimator_update_baro(alt_estimator_t *est, float alt_baro)
{
    if (!est->initialized) {
        /* First reading: set offset so altitude starts at zero */
        est->alt_offset = alt_baro;
        est->baro_alt_prev = alt_baro;
        est->altitude = 0.0f;
        est->climb_rate = 0.0f;
        est->initialized = true;
        return;
    }

    /* Relative altitude from barometer */
    float baro_alt_rel = alt_baro - est->alt_offset;

    /* Barometer-derived climb rate (finite difference, will be smoothed) */
    /* Note: baro update rate is ~100Hz so this is somewhat noisy */
    float baro_climb = (alt_baro - est->baro_alt_prev) * 100.0f; /* approximate */

    /* Complementary filter for altitude:
     * Trust accelerometer-integrated path at high freq (alpha),
     * correct drift with barometer at low freq (1-alpha) */
    est->altitude = est->alpha * est->altitude +
                    (1.0f - est->alpha) * baro_alt_rel;

    /* Complementary filter for climb rate */
    est->climb_rate = est->alpha * est->climb_rate +
                      (1.0f - est->alpha) * baro_climb;

    est->baro_alt_prev = alt_baro;
}

void alt_estimator_update_accel(alt_estimator_t *est, float accel_z, float dt)
{
    if (!est->initialized || dt <= 0.0f) {
        return;
    }

    /* Integrate acceleration to update climb rate */
    est->climb_rate += accel_z * dt;

    /* Integrate climb rate to update altitude */
    est->altitude += est->climb_rate * dt;
}

void alt_estimator_get(const alt_estimator_t *est, float *altitude, float *climb_rate)
{
    if (altitude) {
        *altitude = est->altitude;
    }
    if (climb_rate) {
        *climb_rate = est->climb_rate;
    }
}
