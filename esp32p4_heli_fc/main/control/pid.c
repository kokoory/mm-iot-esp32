/*
 * Generic PID Controller Implementation
 */

#include "pid.h"
#include "../common/math_utils.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = -1.0f;
    pid->output_max = 1.0f;
    pid->integral_max = 1.0f;
}

void pid_set_limits(pid_controller_t *pid, float min, float max)
{
    pid->output_min = min;
    pid->output_max = max;
}

void pid_set_integral_limit(pid_controller_t *pid, float limit)
{
    pid->integral_max = (limit > 0.0f) ? limit : -limit;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* Proportional term */
    float p_term = pid->kp * error;

    /* Integral term with anti-windup clamping */
    pid->integral += error * pid->dt;
    pid->integral = constrain_f(pid->integral, -pid->integral_max, pid->integral_max);
    float i_term = pid->ki * pid->integral;

    /* Derivative term (on error) */
    float derivative = 0.0f;
    if (pid->dt > 0.0f) {
        derivative = (error - pid->prev_error) / pid->dt;
    }
    float d_term = pid->kd * derivative;

    pid->prev_error = error;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;

    /* Anti-windup: if output is saturated, stop integrating in that direction */
    if (output > pid->output_max) {
        output = pid->output_max;
        /* Back-calculate: prevent integral from pushing further into saturation */
        if (error > 0.0f && pid->ki > 0.0f) {
            pid->integral -= error * pid->dt;
        }
    } else if (output < pid->output_min) {
        output = pid->output_min;
        if (error < 0.0f && pid->ki > 0.0f) {
            pid->integral -= error * pid->dt;
        }
    }

    return output;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
