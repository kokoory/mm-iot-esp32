/*
 * AHRS - Attitude and Heading Reference System
 * Madgwick filter for attitude estimation from IMU + magnetometer
 */
#pragma once

typedef struct {
    float q0, q1, q2, q3;  /* quaternion (w, x, y, z) */
    float beta;             /* filter gain (convergence rate) */
} ahrs_t;

/**
 * Initialize AHRS with identity quaternion.
 * @param beta Filter gain, default 0.1. Higher = faster convergence, more noise.
 */
void ahrs_init(ahrs_t *ahrs, float beta);

/**
 * Full 9-DOF Madgwick update with gyro, accel, and magnetometer.
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
 * 6-DOF Madgwick update with gyro and accel only (no magnetometer).
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
