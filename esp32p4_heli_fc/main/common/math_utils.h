#pragma once

#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(deg) ((deg) * (float)(M_PI / 180.0))
#define RAD_TO_DEG(rad) ((rad) * (float)(180.0 / M_PI))

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct {
    float x;
    float y;
    float z;
} vec3f_t;

typedef struct {
    float roll;     // radians
    float pitch;    // radians
    float yaw;      // radians
} euler_t;

/**
 * Constrain a float value between min and max.
 */
static inline float constrain_f(float val, float min_val, float max_val)
{
    if (val < min_val) {
        return min_val;
    }
    if (val > max_val) {
        return max_val;
    }
    return val;
}

/**
 * Wrap an angle in radians to the range [-PI, PI].
 */
static inline float wrap_pi(float angle)
{
    while (angle > (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < -(float)M_PI) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

/**
 * Convert a quaternion to Euler angles (ZYX convention).
 * Output angles are in radians.
 */
static inline void quaternion_to_euler(const quaternion_t *q, euler_t *euler)
{
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf((float)(M_PI / 2.0), sinp);  // gimbal lock
    } else {
        euler->pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

/**
 * Convert Euler angles (ZYX convention) to a quaternion.
 * Input angles are in radians.
 */
static inline void euler_to_quaternion(const euler_t *euler, quaternion_t *q)
{
    float cr = cosf(euler->roll * 0.5f);
    float sr = sinf(euler->roll * 0.5f);
    float cp = cosf(euler->pitch * 0.5f);
    float sp = sinf(euler->pitch * 0.5f);
    float cy = cosf(euler->yaw * 0.5f);
    float sy = sinf(euler->yaw * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

/**
 * Normalize a quaternion to unit length.
 */
static inline void quaternion_normalize(quaternion_t *q)
{
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 1e-6f) {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}

/**
 * Compute the length of a 3D vector.
 */
static inline float vec3f_length(const vec3f_t *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

/**
 * Compute the dot product of two 3D vectors.
 */
static inline float vec3f_dot(const vec3f_t *a, const vec3f_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

/**
 * Compute the cross product of two 3D vectors.
 */
static inline vec3f_t vec3f_cross(const vec3f_t *a, const vec3f_t *b)
{
    vec3f_t result;
    result.x = a->y * b->z - a->z * b->y;
    result.y = a->z * b->x - a->x * b->z;
    result.z = a->x * b->y - a->y * b->x;
    return result;
}
