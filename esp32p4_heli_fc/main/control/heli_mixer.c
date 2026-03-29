/*
 * Helicopter CCPM Swashplate Mixer Implementation
 *
 * PX4-compatible: 120-degree CCPM with throttle/pitch curves,
 * yaw compensation (collective + throttle FF), servo linearization,
 * and throttle spoolup ramp.
 *
 * References:
 *   - PX4 ActuatorEffectivenessHelicopter.cpp
 *   - PX4 16001_helicopter airframe defaults
 */

#include "heli_mixer.h"
#include "../common/math_utils.h"
#include "../common/param.h"
#include <math.h>

/* Servo angles for 120-degree CCPM (degrees) */
#define SERVO1_ANGLE_DEG    0.0f
#define SERVO2_ANGLE_DEG  120.0f
#define SERVO3_ANGLE_DEG  240.0f

/**
 * Interpolate a 5-point curve.
 * input: 0.0 to 1.0 (maps to curve indices 0..4)
 */
static float interpolate_curve(const float curve[5], float input)
{
    input = constrain_f(input, 0.0f, 1.0f);
    float idx_f = input * 4.0f;
    int idx_lo = (int)idx_f;
    if (idx_lo >= 4) {
        return curve[4];
    }
    float frac = idx_f - (float)idx_lo;
    return curve[idx_lo] + frac * (curve[idx_lo + 1] - curve[idx_lo]);
}

/**
 * Inverse-sine servo linearization (PX4 CA_MAX_SVO_THROW).
 * Compensates for non-linear swashplate linkage geometry.
 * throw_deg = max servo throw angle in degrees (0 = disabled).
 */
static float servo_linearize(float input, float throw_deg)
{
    if (throw_deg < 1.0f) return input;  /* disabled */
    float throw_rad = DEG_TO_RAD(throw_deg);
    return asinf(constrain_f(input, -1.0f, 1.0f) * sinf(throw_rad)) / throw_rad;
}

void heli_mixer_init(heli_mixer_config_t *config)
{
    config->ccpm_angle_offset = param_get(PARAM_CCPM_ANGLE_OFFSET);
    config->collective_range  = param_get(PARAM_COLLECTIVE_RANGE);
    config->cyclic_range      = param_get(PARAM_CYCLIC_RANGE);
    config->servo_center_us   = param_get(PARAM_SERVO_CENTER_US);
    config->servo_range_us    = param_get(PARAM_SERVO_RANGE_US);
    config->esc_min_us        = param_get(PARAM_ESC_MIN_US);
    config->esc_max_us        = param_get(PARAM_ESC_MAX_US);
    config->tail_esc_idle_us  = param_get(PARAM_ESC_MIN_US) + 100.0f;
    config->main_esc_idle_us  = param_get(PARAM_ESC_MIN_US) + 100.0f;

    /* Yaw compensation */
    config->tail_coll_ff        = param_get(PARAM_TAIL_COLL_FF);
    config->tail_coll_ff_offset = param_get(PARAM_TAIL_COLL_FF_OFS);
    config->tail_thr_ff         = param_get(PARAM_TAIL_THR_FF);
    config->yaw_ccw             = (param_get(PARAM_YAW_CCW) > 0.5f);

    /* Servo linearization */
    config->servo_throw_deg = param_get(PARAM_SERVO_LINEARIZE);

    /* Spoolup */
    config->spoolup_time_s = param_get(PARAM_SPOOLUP_TIME);

    /* PX4-compatible throttle curve (flat at 1.0 = constant full throttle,
     * suitable for ESCs with built-in governor) */
    config->throttle_curve[0] = 0.0f;
    config->throttle_curve[1] = 0.4f;
    config->throttle_curve[2] = 0.55f;
    config->throttle_curve[3] = 0.7f;
    config->throttle_curve[4] = 0.85f;

    /* PX4-compatible collective pitch curve */
    config->collective_curve[0] = -1.0f;
    config->collective_curve[1] = -0.5f;
    config->collective_curve[2] =  0.0f;
    config->collective_curve[3] =  0.5f;
    config->collective_curve[4] =  1.0f;
}

void heli_mixer_update(const heli_mixer_config_t *config,
                       const actuator_controls_t *controls,
                       float spoolup_progress,
                       heli_mixer_output_t *output)
{
    float roll_in  = constrain_f(controls->roll, -1.0f, 1.0f);
    float pitch_in = constrain_f(controls->pitch, -1.0f, 1.0f);
    float yaw_in   = constrain_f(controls->yaw, -1.0f, 1.0f);
    float coll_in  = constrain_f(controls->collective, -1.0f, 1.0f);

    /* Clamp spoolup to [0, 1] */
    spoolup_progress = constrain_f(spoolup_progress, 0.0f, 1.0f);

    /*
     * Map collective stick input [−1, 1] → curve index [0, 1].
     */
    float coll_normalized = (coll_in + 1.0f) * 0.5f;  /* 0..1 */

    /* Look up collective pitch from collective curve */
    float collective = interpolate_curve(config->collective_curve, coll_normalized);

    /* Look up throttle from throttle curve */
    float throttle = interpolate_curve(config->throttle_curve, coll_normalized);

    /* Apply spoolup ramp to throttle */
    throttle *= spoolup_progress;

    /* Apply collective and cyclic range scaling */
    float coll_servo = collective * config->collective_range;
    float pitch_servo = pitch_in * config->cyclic_range;
    float roll_servo  = roll_in  * config->cyclic_range;

    /* Compute three CCPM servo positions.
     * PX4 formula: servo = collective + cos(angle)*pitch*arm - sin(angle)*roll*arm
     * (arm_length = 1.0 for all servos in 120-deg CCPM)
     */
    float offset_rad = DEG_TO_RAD(config->ccpm_angle_offset);

    float angle1 = DEG_TO_RAD(SERVO1_ANGLE_DEG) + offset_rad;
    float angle2 = DEG_TO_RAD(SERVO2_ANGLE_DEG) + offset_rad;
    float angle3 = DEG_TO_RAD(SERVO3_ANGLE_DEG) + offset_rad;

    float servo1_norm = coll_servo + pitch_servo * cosf(angle1) + roll_servo * sinf(angle1);
    float servo2_norm = coll_servo + pitch_servo * cosf(angle2) + roll_servo * sinf(angle2);
    float servo3_norm = coll_servo + pitch_servo * cosf(angle3) + roll_servo * sinf(angle3);

    /* Apply inverse-sine servo linearization if configured */
    servo1_norm = servo_linearize(servo1_norm, config->servo_throw_deg);
    servo2_norm = servo_linearize(servo2_norm, config->servo_throw_deg);
    servo3_norm = servo_linearize(servo3_norm, config->servo_throw_deg);

    /* Clamp normalized servo values to [-1, 1] */
    servo1_norm = constrain_f(servo1_norm, -1.0f, 1.0f);
    servo2_norm = constrain_f(servo2_norm, -1.0f, 1.0f);
    servo3_norm = constrain_f(servo3_norm, -1.0f, 1.0f);

    /* Convert to microseconds */
    output->servo1_us = config->servo_center_us + servo1_norm * config->servo_range_us;
    output->servo2_us = config->servo_center_us + servo2_norm * config->servo_range_us;
    output->servo3_us = config->servo_center_us + servo3_norm * config->servo_range_us;

    /* ── Tail ESC: yaw + PX4-compatible compensation ──
     *
     * PX4 formula:
     *   tail = yaw_cmd + CP_S * abs(collective - CP_O) + TH_S * throttle
     *
     * CP_S compensates torque from collective pitch changes
     * CP_O is the collective pitch at minimum torque
     * TH_S compensates torque from throttle changes
     * YAW_CCW reverses compensation direction
     */
    float yaw_comp = config->tail_coll_ff * fabsf(collective - config->tail_coll_ff_offset)
                   + config->tail_thr_ff * throttle;

    /* Reverse compensation for CCW main rotor */
    if (config->yaw_ccw) {
        yaw_comp = -yaw_comp;
    }

    float tail_cmd = constrain_f(yaw_in + yaw_comp, -1.0f, 1.0f);
    float tail_normalized = (tail_cmd + 1.0f) * 0.5f;  /* 0..1 */
    output->tail_esc_us = config->esc_min_us +
                          tail_normalized * (config->esc_max_us - config->esc_min_us);

    /* Main ESC: throttle (already spoolup-ramped above) */
    output->main_esc_us = config->esc_min_us +
                          throttle * (config->esc_max_us - config->esc_min_us);

    /* Clamp all outputs */
    float servo_min = config->servo_center_us - config->servo_range_us;
    float servo_max = config->servo_center_us + config->servo_range_us;
    output->servo1_us   = constrain_f(output->servo1_us, servo_min, servo_max);
    output->servo2_us   = constrain_f(output->servo2_us, servo_min, servo_max);
    output->servo3_us   = constrain_f(output->servo3_us, servo_min, servo_max);
    output->tail_esc_us = constrain_f(output->tail_esc_us, config->esc_min_us, config->esc_max_us);
    output->main_esc_us = constrain_f(output->main_esc_us, config->esc_min_us, config->esc_max_us);
}
