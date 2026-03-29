/*
 * Parameter System - PX4-inspired runtime parameter management
 *
 * Features:
 *   - Named float parameters with default values, min/max ranges
 *   - NVS persistence (save/load from flash)
 *   - MAVLink PARAM protocol support (PARAM_VALUE, PARAM_SET, PARAM_REQUEST_*)
 *   - Runtime PID tuning from GCS without recompile
 *
 * Usage:
 *   param_init();
 *   float kp = param_get(PARAM_ROLL_RATE_KP);
 *   param_set(PARAM_ROLL_RATE_KP, 0.2f);
 *   param_save_all();
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Parameter IDs - must match param_table[] order in param.c */
typedef enum {
    /* Rate controller PID gains */
    PARAM_ROLL_RATE_KP = 0,
    PARAM_ROLL_RATE_KI,
    PARAM_ROLL_RATE_KD,
    PARAM_ROLL_RATE_FF,
    PARAM_PITCH_RATE_KP,
    PARAM_PITCH_RATE_KI,
    PARAM_PITCH_RATE_KD,
    PARAM_PITCH_RATE_FF,
    PARAM_YAW_RATE_KP,
    PARAM_YAW_RATE_KI,
    PARAM_YAW_RATE_KD,
    PARAM_YAW_RATE_FF,
    PARAM_RATE_K,               /* Overall rate gain scale factor */

    /* Attitude controller gains */
    PARAM_ATT_ROLL_KP,
    PARAM_ATT_PITCH_KP,
    PARAM_ATT_YAW_KP,

    /* Altitude / position controller */
    PARAM_ALT_KP,
    PARAM_CLIMB_RATE_KP,
    PARAM_CLIMB_RATE_KI,
    PARAM_CLIMB_RATE_KD,
    PARAM_MAX_CLIMB_RATE,

    /* Attitude limits */
    PARAM_MAX_ROLL_DEG,
    PARAM_MAX_PITCH_DEG,
    PARAM_MAX_YAW_RATE_DEG,
    PARAM_MAX_ROLL_RATE_DEG,    /* ACRO mode max roll rate (deg/s) */
    PARAM_MAX_PITCH_RATE_DEG,   /* ACRO mode max pitch rate (deg/s) */

    /* AHRS / estimator */
    PARAM_AHRS_BETA,
    PARAM_ALT_CF_ALPHA,

    /* Helicopter mixer */
    PARAM_CCPM_ANGLE_OFFSET,
    PARAM_COLLECTIVE_RANGE,
    PARAM_CYCLIC_RANGE,
    PARAM_SERVO_CENTER_US,
    PARAM_SERVO_RANGE_US,
    PARAM_ESC_MIN_US,
    PARAM_ESC_MAX_US,
    PARAM_TAIL_COLL_FF,
    PARAM_TAIL_COLL_FF_OFS,     /* Collective offset for min torque (yaw compensation) */
    PARAM_TAIL_THR_FF,          /* Throttle -> tail feedforward */
    PARAM_YAW_CCW,              /* Main rotor CCW (1) or CW (0) */
    PARAM_SERVO_LINEARIZE,      /* Servo throw angle (deg) for inverse-sine linearization, 0=off */
    PARAM_SPOOLUP_TIME,         /* Throttle spool-up time after arming (seconds) */

    /* Battery thresholds */
    PARAM_BATT_LOW_V,
    PARAM_BATT_CRIT_V,
    PARAM_BATT_VDIV_RATIO,

    /* RC input */
    PARAM_RC_DEADZONE,          /* Stick deadzone for roll/pitch/yaw (0-1) */
    PARAM_RC_ARM_THRESHOLD,     /* CH5 threshold: above = arm (normalized) */
    PARAM_RC_COLL_ARM_MAX,      /* Max collective position to allow arming */

    /* System */
    PARAM_SENSOR_TIMEOUT_MS,
    PARAM_GCS_TIMEOUT_MS,       /* GCS heartbeat loss timeout (ms) */

    PARAM_COUNT
} param_id_t;

/* Parameter metadata (read-only) */
typedef struct {
    const char *name;       /* 16-char max for MAVLink */
    float default_val;
    float min_val;
    float max_val;
} param_meta_t;

/**
 * Initialize the parameter system.
 * Loads saved values from NVS, or uses defaults for unsaved params.
 */
void param_init(void);

/**
 * Get a parameter value.
 */
float param_get(param_id_t id);

/**
 * Set a parameter value (clamped to min/max).
 * Returns true if value changed.
 */
bool param_set(param_id_t id, float value);

/**
 * Reset a parameter to its default value.
 */
void param_reset(param_id_t id);

/**
 * Reset all parameters to defaults.
 */
void param_reset_all(void);

/**
 * Save all modified parameters to NVS flash.
 * Returns 0 on success, -1 on failure.
 */
int param_save_all(void);

/**
 * Get parameter count.
 */
uint16_t param_count(void);

/**
 * Get parameter metadata by index.
 * Returns NULL if index out of range.
 */
const param_meta_t *param_get_meta(param_id_t id);

/**
 * Find parameter ID by name.
 * Returns PARAM_COUNT if not found.
 */
param_id_t param_find(const char *name);

#ifdef __cplusplus
}
#endif
