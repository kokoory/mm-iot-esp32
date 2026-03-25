/*
 * AHRS - Attitude and Heading Reference System
 *
 * Extended Kalman Filter (EKF) for attitude estimation.
 * State vector: quaternion (4) + gyro bias (3) = 7 states
 *
 * Prediction: quaternion integration with bias-corrected gyro
 * Measurement updates: accelerometer (gravity) and magnetometer (heading)
 */
#pragma once

#include <stdbool.h>

/* EKF state dimension: 4 (quaternion) + 3 (gyro bias) = 7 */
#define AHRS_STATE_DIM 7

typedef struct {
    /* Quaternion state (w, x, y, z) */
    float q0, q1, q2, q3;

    /* Gyro bias estimate (rad/s) */
    float gyro_bias[3];

    /* Error covariance matrix P (7x7, stored as flat array) */
    float P[AHRS_STATE_DIM * AHRS_STATE_DIM];

    /* Process noise parameters */
    float gyro_noise;       /* gyro measurement noise (rad/s) */
    float gyro_bias_noise;  /* gyro bias random walk (rad/s^2) */
    float accel_noise;      /* accelerometer noise (m/s^2) */
    float mag_noise;        /* magnetometer noise (normalized) */

    /* Filter tuning */
    float beta;             /* kept for API compatibility; not used in EKF */

    /* Magnetic reference direction in NED frame */
    float mag_ref_x, mag_ref_z;
    bool mag_ref_valid;
} ahrs_t;

/**
 * Initialize AHRS with identity quaternion and zero bias.
 * @param beta Unused in EKF mode; kept for API compatibility
 */
void ahrs_init(ahrs_t *ahrs, float beta);

/**
 * Full 9-DOF EKF update with gyro, accel, and magnetometer.
 * @param gyro  Angular rates [x, y, z] in rad/s
 * @param accel Accelerometer [x, y, z] in m/s^2 (will be normalized)
 * @param mag   Magnetometer [x, y, z] in any consistent units (will be normalized)
 * @param dt    Time step in seconds
 */
void ahrs_update(ahrs_t *ahrs,
                 const float gyro[3],
                 const float accel[3],
                 const float mag[3],
                 float dt);

/**
 * 6-DOF EKF update with gyro and accel only (no magnetometer).
 */
void ahrs_update_imu(ahrs_t *ahrs,
                     const float gyro[3],
                     const float accel[3],
                     float dt);

/**
 * Get current quaternion estimate.
 * @param q Output array [w, x, y, z]
 */
void ahrs_get_quaternion(const ahrs_t *ahrs, float q[4]);

/**
 * Get current Euler angles.
 * @param roll  Output roll in radians
 * @param pitch Output pitch in radians
 * @param yaw   Output yaw in radians
 */
void ahrs_get_euler(const ahrs_t *ahrs, float *roll, float *pitch, float *yaw);

/**
 * Get current gyro bias estimate.
 * @param bias Output array [x, y, z] in rad/s
 */
void ahrs_get_gyro_bias(const ahrs_t *ahrs, float bias[3]);
