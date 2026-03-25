/*
 * Position / Altitude Controller Implementation
 * Cascaded control: outer P loop (altitude->climb rate), inner PID (climb rate->collective)
 */

#include "pos_control.h"
#include "../common/math_utils.h"
#include "../common/param.h"

void pos_control_init(pos_controller_t *pc, float dt)
{
    pc->alt_kp = param_get(PARAM_ALT_KP);
    pc->max_climb_rate = param_get(PARAM_MAX_CLIMB_RATE);

    pid_init(&pc->climb_rate_pid,
             param_get(PARAM_CLIMB_RATE_KP),
             param_get(PARAM_CLIMB_RATE_KI),
             param_get(PARAM_CLIMB_RATE_KD),
             dt);
    pid_set_limits(&pc->climb_rate_pid, -1.0f, 1.0f);
    pid_set_integral_limit(&pc->climb_rate_pid, 0.5f);
}

void pos_control_update_altitude(pos_controller_t *pc,
                                 float alt_sp,
                                 float alt_meas,
                                 float climb_rate,
                                 float *collective_out)
{
    /* Outer loop: altitude error -> climb rate setpoint */
    float alt_error = alt_sp - alt_meas;
    float climb_rate_sp = pc->alt_kp * alt_error;

    /* Clamp climb rate setpoint */
    climb_rate_sp = constrain_f(climb_rate_sp,
                                -pc->max_climb_rate,
                                pc->max_climb_rate);

    /* Inner loop: climb rate error -> collective output */
    *collective_out = pid_update(&pc->climb_rate_pid, climb_rate_sp, climb_rate);
}

void pos_control_reset(pos_controller_t *pc)
{
    pid_reset(&pc->climb_rate_pid);
}
