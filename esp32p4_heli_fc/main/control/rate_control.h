/*
 * 3-axis Angular Rate Controller
 * Three independent PID controllers for roll/pitch/yaw rate
 */
#pragma once

#include "pid.h"

typedef struct {
    pid_controller_t roll;
    pid_controller_t pitch;
    pid_controller_t yaw;
} rate_controller_t;

/**
 * Initialize rate controllers with default helicopter gains.
 * dt = controller timestep in seconds.
 */
void rate_control_init(rate_controller_t *rc, float dt);

/**
 * Compute rate controller output.
 * @param rate_sp  Desired angular rates [roll, pitch, yaw] in rad/s
 * @param rate_meas Measured angular rates [roll, pitch, yaw] in rad/s
 * @param output   Control output [roll, pitch, yaw], range [-1, 1]
 */
void rate_control_update(rate_controller_t *rc,
                         const float rate_sp[3],
                         const float rate_meas[3],
                         float output[3]);

/**
 * Reset all three rate PID controllers.
 */
void rate_control_reset(rate_controller_t *rc);
