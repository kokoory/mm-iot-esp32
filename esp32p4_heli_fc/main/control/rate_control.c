/*
 * 3-axis Angular Rate Controller Implementation
 *
 * PX4-compatible: Feed-forward dominant for helicopters.
 * Overall gain K scales all P/I/D terms: output = K*(P+I+D) + FF*setpoint
 */

#include "rate_control.h"
#include "../common/param.h"

void rate_control_init(rate_controller_t *rc, float dt)
{
    float k = param_get(PARAM_RATE_K);

    pid_init(&rc->roll,
             param_get(PARAM_ROLL_RATE_KP) * k,
             param_get(PARAM_ROLL_RATE_KI) * k,
             param_get(PARAM_ROLL_RATE_KD) * k, dt);
    pid_set_limits(&rc->roll, -1.0f, 1.0f);
    pid_set_integral_limit(&rc->roll, 0.3f);
    pid_set_feedforward(&rc->roll, param_get(PARAM_ROLL_RATE_FF));

    pid_init(&rc->pitch,
             param_get(PARAM_PITCH_RATE_KP) * k,
             param_get(PARAM_PITCH_RATE_KI) * k,
             param_get(PARAM_PITCH_RATE_KD) * k, dt);
    pid_set_limits(&rc->pitch, -1.0f, 1.0f);
    pid_set_integral_limit(&rc->pitch, 0.3f);
    pid_set_feedforward(&rc->pitch, param_get(PARAM_PITCH_RATE_FF));

    pid_init(&rc->yaw,
             param_get(PARAM_YAW_RATE_KP) * k,
             param_get(PARAM_YAW_RATE_KI) * k,
             param_get(PARAM_YAW_RATE_KD) * k, dt);
    pid_set_limits(&rc->yaw, -1.0f, 1.0f);
    pid_set_integral_limit(&rc->yaw, 0.5f);
    pid_set_feedforward(&rc->yaw, param_get(PARAM_YAW_RATE_FF));
}

void rate_control_update(rate_controller_t *rc,
                         const float rate_sp[3],
                         const float rate_meas[3],
                         float output[3])
{
    output[0] = pid_update(&rc->roll, rate_sp[0], rate_meas[0]);
    output[1] = pid_update(&rc->pitch, rate_sp[1], rate_meas[1]);
    output[2] = pid_update(&rc->yaw, rate_sp[2], rate_meas[2]);
}

void rate_control_reset(rate_controller_t *rc)
{
    pid_reset(&rc->roll);
    pid_reset(&rc->pitch);
    pid_reset(&rc->yaw);
}
