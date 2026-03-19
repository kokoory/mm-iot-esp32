/*
 * 3-axis Angular Rate Controller Implementation
 */

#include "rate_control.h"

/* Default gains for helicopter */
#define ROLL_RATE_KP   0.15f
#define ROLL_RATE_KI   0.05f
#define ROLL_RATE_KD   0.003f

#define PITCH_RATE_KP  0.15f
#define PITCH_RATE_KI  0.05f
#define PITCH_RATE_KD  0.003f

#define YAW_RATE_KP    0.3f
#define YAW_RATE_KI    0.1f
#define YAW_RATE_KD    0.0f

void rate_control_init(rate_controller_t *rc, float dt)
{
    pid_init(&rc->roll, ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD, dt);
    pid_set_limits(&rc->roll, -1.0f, 1.0f);
    pid_set_integral_limit(&rc->roll, 0.3f);

    pid_init(&rc->pitch, PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD, dt);
    pid_set_limits(&rc->pitch, -1.0f, 1.0f);
    pid_set_integral_limit(&rc->pitch, 0.3f);

    pid_init(&rc->yaw, YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD, dt);
    pid_set_limits(&rc->yaw, -1.0f, 1.0f);
    pid_set_integral_limit(&rc->yaw, 0.5f);
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
