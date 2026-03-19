/*
 * Position / Altitude Controller Implementation
 * Cascaded control: outer P loop (altitude->climb rate), inner PID (climb rate->collective)
 */

#include "pos_control.h"
#include "../common/math_utils.h"

#define DEFAULT_ALT_KP          1.0f
#define DEFAULT_CLIMB_RATE_KP   0.5f
#define DEFAULT_CLIMB_RATE_KI   0.1f
#define DEFAULT_CLIMB_RATE_KD   0.0f
#define DEFAULT_MAX_CLIMB_RATE  2.0f    /* m/s */

void pos_control_init(pos_controller_t *pc, float dt)
{
    pc->alt_kp = DEFAULT_ALT_KP;
    pc->max_climb_rate = DEFAULT_MAX_CLIMB_RATE;

    pid_init(&pc->climb_rate_pid,
             DEFAULT_CLIMB_RATE_KP,
             DEFAULT_CLIMB_RATE_KI,
             DEFAULT_CLIMB_RATE_KD,
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
