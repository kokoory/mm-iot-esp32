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
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;
    pid->d_filter_alpha = 0.2f;  /* 20% new value, 80% old — cuts high-freq noise */
    pid->has_prev_measurement = false;
    pid->output_min = -1.0f;
    pid->output_max = 1.0f;
    pid->integral_max = 1.0f;
    pid->ff = 0.0f;
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

void pid_set_feedforward(pid_controller_t *pid, float ff)
{
    pid->ff = ff;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* Proportional term */
    float p_term = pid->kp * error;

    /* Feedforward term */
    float ff_term = pid->ff * setpoint;

    /* Integral term with anti-windup clamping */
    pid->integral += error * pid->dt;
    pid->integral = constrain_f(pid->integral, -pid->integral_max, pid->integral_max);
    float i_term = pid->ki * pid->integral;

    /* Derivative term on measurement (avoids derivative kick on setpoint change) */
    float derivative = 0.0f;
    if (pid->dt > 0.0f && pid->has_prev_measurement) {
        float raw_deriv = -(measurement - pid->prev_measurement) / pid->dt;
        /* Low-pass filter on derivative to reduce high-frequency noise */
        pid->d_filtered += pid->d_filter_alpha * (raw_deriv - pid->d_filtered);
        derivative = pid->d_filtered;
    }
    float d_term = pid->kd * derivative;

    pid->prev_error = error;
    pid->prev_measurement = measurement;
    pid->has_prev_measurement = true;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term + ff_term;

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
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;
    pid->has_prev_measurement = false;
}
