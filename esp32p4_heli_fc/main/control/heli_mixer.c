/*
 * Helicopter CCPM Swashplate Mixer Implementation
 *
 * 120-degree CCPM: three servos placed at 0, 120, and 240 degrees.
 * Each servo output = collective + cyclic_pitch * cos(angle) + cyclic_roll * sin(angle)
 *
 * Throttle is derived from a throttle curve indexed by collective stick position.
 * Collective pitch is derived from a collective curve indexed by stick position.
 */

#include "heli_mixer.h"
#include "../common/math_utils.h"
#include <math.h>

/* Servo angles for 120-degree CCPM (degrees) */
#define SERVO1_ANGLE_DEG    0.0f
#define SERVO2_ANGLE_DEG  120.0f
#define SERVO3_ANGLE_DEG  240.0f

/**
 * Interpolate a 5-point curve.
 * input: 0.0 to 1.0 (maps to curve indices 0..4)
 * Returns interpolated value.
 */
static float interpolate_curve(const float curve[5], float input)
{
    /* Clamp input to [0, 1] */
    input = constrain_f(input, 0.0f, 1.0f);

    /* Scale to index range [0, 4] */
    float idx_f = input * 4.0f;
    int idx_lo = (int)idx_f;
    if (idx_lo >= 4) {
        return curve[4];
    }

    float frac = idx_f - (float)idx_lo;
    return curve[idx_lo] + frac * (curve[idx_lo + 1] - curve[idx_lo]);
}

void heli_mixer_init(heli_mixer_config_t *config)
{
    config->ccpm_angle_offset = 0.0f;
    config->collective_range  = 1.0f;
    config->cyclic_range      = 1.0f;
    config->servo_center_us   = 1500.0f;
    config->servo_range_us    = 500.0f;
    config->esc_min_us        = 1000.0f;
    config->esc_max_us        = 2000.0f;
    config->tail_esc_idle_us  = 1100.0f;
    config->main_esc_idle_us  = 1100.0f;

    /* Normal mode throttle curve */
    config->throttle_curve[0] = 0.0f;
    config->throttle_curve[1] = 0.4f;
    config->throttle_curve[2] = 0.55f;
    config->throttle_curve[3] = 0.7f;
    config->throttle_curve[4] = 0.85f;

    /* Linear collective curve */
    config->collective_curve[0] = -1.0f;
    config->collective_curve[1] = -0.5f;
    config->collective_curve[2] =  0.0f;
    config->collective_curve[3] =  0.5f;
    config->collective_curve[4] =  1.0f;
}

void heli_mixer_update(const heli_mixer_config_t *config,
                       const actuator_controls_t *controls,
                       heli_mixer_output_t *output)
{
    float roll_in  = constrain_f(controls->roll, -1.0f, 1.0f);
    float pitch_in = constrain_f(controls->pitch, -1.0f, 1.0f);
    float yaw_in   = constrain_f(controls->yaw, -1.0f, 1.0f);
    float coll_in  = constrain_f(controls->collective, -1.0f, 1.0f);

    /*
     * Map collective stick input [−1, 1] → curve index [0, 1].
     * Stick −1 maps to index 0, stick +1 maps to index 1.
     */
    float coll_normalized = (coll_in + 1.0f) * 0.5f;  /* 0..1 */

    /* Look up collective pitch from collective curve */
    float collective = interpolate_curve(config->collective_curve, coll_normalized);

    /* Look up throttle from throttle curve, indexed by same collective stick */
    float throttle = interpolate_curve(config->throttle_curve, coll_normalized);

    /* Apply collective and cyclic range scaling */
    float coll_servo = collective * config->collective_range;
    float pitch_servo = pitch_in * config->cyclic_range;
    float roll_servo  = roll_in  * config->cyclic_range;

    /* Compute three CCPM servo positions.
     * Each servo at angle theta_i:
     *   servo = collective + pitch * cos(theta_i + offset) + roll * sin(theta_i + offset)
     */
    float offset_rad = DEG_TO_RAD(config->ccpm_angle_offset);

    float angle1 = DEG_TO_RAD(SERVO1_ANGLE_DEG) + offset_rad;
    float angle2 = DEG_TO_RAD(SERVO2_ANGLE_DEG) + offset_rad;
    float angle3 = DEG_TO_RAD(SERVO3_ANGLE_DEG) + offset_rad;

    float servo1_norm = coll_servo + pitch_servo * cosf(angle1) + roll_servo * sinf(angle1);
    float servo2_norm = coll_servo + pitch_servo * cosf(angle2) + roll_servo * sinf(angle2);
    float servo3_norm = coll_servo + pitch_servo * cosf(angle3) + roll_servo * sinf(angle3);

    /* Clamp normalized servo values to [-1, 1] */
    servo1_norm = constrain_f(servo1_norm, -1.0f, 1.0f);
    servo2_norm = constrain_f(servo2_norm, -1.0f, 1.0f);
    servo3_norm = constrain_f(servo3_norm, -1.0f, 1.0f);

    /* Convert to microseconds */
    output->servo1_us = config->servo_center_us + servo1_norm * config->servo_range_us;
    output->servo2_us = config->servo_center_us + servo2_norm * config->servo_range_us;
    output->servo3_us = config->servo_center_us + servo3_norm * config->servo_range_us;

    /* Tail ESC: map yaw control [-1, 1] to ESC range [esc_min, esc_max] */
    float yaw_normalized = (yaw_in + 1.0f) * 0.5f;  /* 0..1 */
    output->tail_esc_us = config->esc_min_us +
                          yaw_normalized * (config->esc_max_us - config->esc_min_us);

    /* Main ESC: throttle from throttle curve */
    output->main_esc_us = config->esc_min_us +
                          throttle * (config->esc_max_us - config->esc_min_us);

    /* Clamp all outputs */
    output->servo1_us   = constrain_f(output->servo1_us, config->esc_min_us, config->esc_max_us);
    output->servo2_us   = constrain_f(output->servo2_us, config->esc_min_us, config->esc_max_us);
    output->servo3_us   = constrain_f(output->servo3_us, config->esc_min_us, config->esc_max_us);
    output->tail_esc_us = constrain_f(output->tail_esc_us, config->esc_min_us, config->esc_max_us);
    output->main_esc_us = constrain_f(output->main_esc_us, config->esc_min_us, config->esc_max_us);
}
