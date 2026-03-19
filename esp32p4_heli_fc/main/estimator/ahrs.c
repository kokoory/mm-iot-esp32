/*
 * AHRS - Madgwick Filter Implementation
 *
 * Based on Sebastian Madgwick's gradient descent algorithm for IMU/MARG
 * sensor fusion. Reference:
 *   S. Madgwick, "An efficient orientation filter for inertial and
 *   inertial/magnetic sensor arrays", 2010.
 */

#include "ahrs.h"
#include "../common/math_utils.h"
#include <math.h>

void ahrs_init(ahrs_t *ahrs, float beta)
{
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->beta = beta;
}

void ahrs_update(ahrs_t *ahrs,
                 const float gyro[3],
                 const float accel[3],
                 const float mag[3],
                 float dt)
{
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];

    float ax = accel[0], ay = accel[1], az = accel[2];
    float mx = mag[0], my = mag[1], mz = mag[2];

    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;
    float norm;

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* Normalize accelerometer measurement */
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        /* Accelerometer data invalid, integrate gyro only */
        goto integrate;
    }
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* Normalize magnetometer measurement */
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm < 1e-6f) {
        /* No valid magnetometer data, fall back to IMU-only update */
        ahrs_update_imu(ahrs, gyro, accel, dt);
        return;
    }
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    /* Auxiliary variables to avoid repeated computation */
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    /* Reference direction of Earth's magnetic field */
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 +
         mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 -
         mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 +
         _2q1mx * q2 - my * q1q1 + my * q2q2 +
         _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 +
           _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 -
           mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    /* Gradient descent corrective step */
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
          _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
          _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
          _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
          _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
          4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
          _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
          _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
          4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
          _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
          _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    /* Normalize step magnitude */
    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm > 1e-6f) {
        norm = 1.0f / norm;
        s0 *= norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
    }

    /* Apply feedback step */
    qDot1 -= ahrs->beta * s0;
    qDot2 -= ahrs->beta * s1;
    qDot3 -= ahrs->beta * s2;
    qDot4 -= ahrs->beta * s3;

integrate:
    /* Integrate rate of change of quaternion */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* Normalize quaternion */
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 1e-6f) {
        norm = 1.0f / norm;
        ahrs->q0 = q0 * norm;
        ahrs->q1 = q1 * norm;
        ahrs->q2 = q2 * norm;
        ahrs->q3 = q3 * norm;
    }
}

void ahrs_update_imu(ahrs_t *ahrs,
                     const float gyro[3],
                     const float accel[3],
                     float dt)
{
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float ax = accel[0], ay = accel[1], az = accel[2];
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;
    float norm;

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* Normalize accelerometer measurement */
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        goto integrate_imu;
    }
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* Auxiliary variables */
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    /* Gradient descent corrective step (accelerometer only) */
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    /* Normalize step */
    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm > 1e-6f) {
        norm = 1.0f / norm;
        s0 *= norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
    }

    /* Apply feedback */
    qDot1 -= ahrs->beta * s0;
    qDot2 -= ahrs->beta * s1;
    qDot3 -= ahrs->beta * s2;
    qDot4 -= ahrs->beta * s3;

integrate_imu:
    /* Integrate */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* Normalize */
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 1e-6f) {
        norm = 1.0f / norm;
        ahrs->q0 = q0 * norm;
        ahrs->q1 = q1 * norm;
        ahrs->q2 = q2 * norm;
        ahrs->q3 = q3 * norm;
    }
}

void ahrs_get_quaternion(const ahrs_t *ahrs, float q[4])
{
    q[0] = ahrs->q0;
    q[1] = ahrs->q1;
    q[2] = ahrs->q2;
    q[3] = ahrs->q3;
}

void ahrs_get_euler(const ahrs_t *ahrs, float *roll, float *pitch, float *yaw)
{
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;

    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf((float)(M_PI / 2.0), sinp);
    } else {
        *pitch = asinf(sinp);
    }

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}
