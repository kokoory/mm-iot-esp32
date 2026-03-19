/*
 * Generic PID Controller
 * With anti-windup clamping and optional derivative filtering
 */
#pragma once

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_min, output_max;
    float integral_max;
    float dt;
} pid_controller_t;

/**
 * Initialize PID controller with gains and timestep.
 * Limits default to +/-1.0, integral limit defaults to output limit.
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt);

/**
 * Set output limits (symmetric min/max clamp).
 */
void pid_set_limits(pid_controller_t *pid, float min, float max);

/**
 * Set maximum absolute value of integral accumulator (anti-windup).
 */
void pid_set_integral_limit(pid_controller_t *pid, float limit);

/**
 * Compute one PID update step.
 * Returns the clamped controller output.
 */
float pid_update(pid_controller_t *pid, float setpoint, float measurement);

/**
 * Reset integrator and derivative state.
 */
void pid_reset(pid_controller_t *pid);
