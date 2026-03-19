/*
 * Position / Altitude Controller
 * Cascaded altitude hold: altitude -> climb_rate -> collective output
 */
#pragma once

#include "pid.h"

typedef struct {
    /* Outer loop: altitude -> climb rate setpoint (P controller) */
    float alt_kp;
    float max_climb_rate;   /* m/s */

    /* Inner loop: climb rate -> collective output (PID) */
    pid_controller_t climb_rate_pid;
} pos_controller_t;

/**
 * Initialize position/altitude controller with defaults.
 */
void pos_control_init(pos_controller_t *pc, float dt);

/**
 * Run altitude hold controller.
 * @param alt_sp         Desired altitude (m, positive up)
 * @param alt_meas       Measured altitude (m, positive up)
 * @param climb_rate     Measured climb rate (m/s, positive up)
 * @param collective_out Output collective command [-1, 1]
 */
void pos_control_update_altitude(pos_controller_t *pc,
                                 float alt_sp,
                                 float alt_meas,
                                 float climb_rate,
                                 float *collective_out);

/**
 * Reset altitude controller state.
 */
void pos_control_reset(pos_controller_t *pc);
