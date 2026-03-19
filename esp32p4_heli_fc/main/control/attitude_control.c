/*
 * Attitude (Angle) Controller Implementation
 * Simple proportional controller: rate_sp = Kp * angle_error
 * Yaw error is wrapped to [-PI, PI]
 */

#include "attitude_control.h"
#include "../common/math_utils.h"

#define DEFAULT_KP_ROLL   4.5f
#define DEFAULT_KP_PITCH  4.5f
#define DEFAULT_KP_YAW    2.0f

void attitude_control_init(attitude_controller_t *ac)
{
    ac->kp_roll  = DEFAULT_KP_ROLL;
    ac->kp_pitch = DEFAULT_KP_PITCH;
    ac->kp_yaw   = DEFAULT_KP_YAW;
}

void attitude_control_update(const attitude_controller_t *ac,
                             const float att_sp[3],
                             const float att_meas[3],
                             float rate_sp[3])
{
    /* Roll error */
    float roll_error = att_sp[0] - att_meas[0];
    rate_sp[0] = ac->kp_roll * roll_error;

    /* Pitch error */
    float pitch_error = att_sp[1] - att_meas[1];
    rate_sp[1] = ac->kp_pitch * pitch_error;

    /* Yaw error - wrapped to [-PI, PI] */
    float yaw_error = wrap_pi(att_sp[2] - att_meas[2]);
    rate_sp[2] = ac->kp_yaw * yaw_error;
}
