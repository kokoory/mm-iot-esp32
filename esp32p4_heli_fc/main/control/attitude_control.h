/*
 * Attitude (Angle) Controller
 * Outer-loop P controller that generates rate setpoints from angle error.
 */
#pragma once

typedef struct {
    float kp_roll;
    float kp_pitch;
    float kp_yaw;
} attitude_controller_t;

/**
 * Initialize attitude controller with default gains.
 */
void attitude_control_init(attitude_controller_t *ac);

/**
 * Compute rate setpoints from attitude error.
 * @param att_sp   Desired attitude [roll, pitch, yaw] in radians
 * @param att_meas Measured attitude [roll, pitch, yaw] in radians
 * @param rate_sp  Output rate setpoints [roll, pitch, yaw] in rad/s
 */
void attitude_control_update(const attitude_controller_t *ac,
                             const float att_sp[3],
                             const float att_meas[3],
                             float rate_sp[3]);
