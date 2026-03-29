/*
 * PID Autotune - Relay Feedback Method
 *
 * Implements the Astrom-Hagglund relay feedback method for automated
 * PID tuning. Applies a relay (bang-bang) signal to an axis, measures
 * the resulting oscillation period and amplitude, then computes optimal
 * PID gains using the Ziegler-Nichols or relay tuning formulas.
 *
 * Usage:
 *   1. Enter autotune mode (e.g., via MAVLink command)
 *   2. Call autotune_start() with the desired axis
 *   3. In each control loop iteration, call autotune_update()
 *      - It returns the relay override output for the axis being tuned
 *   4. When autotune_is_complete() returns true, read the gains
 *      with autotune_get_gains() and apply them
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Axes that can be autotuned */
typedef enum {
    AUTOTUNE_AXIS_ROLL = 0,
    AUTOTUNE_AXIS_PITCH,
    AUTOTUNE_AXIS_YAW,
} autotune_axis_t;

/* Autotune state */
typedef enum {
    AUTOTUNE_STATE_IDLE = 0,
    AUTOTUNE_STATE_RUNNING,
    AUTOTUNE_STATE_COMPLETE,
    AUTOTUNE_STATE_FAILED,
} autotune_state_t;

/* Computed PID gains from autotune */
typedef struct {
    float kp;
    float ki;
    float kd;
} autotune_gains_t;

/* Autotune configuration */
typedef struct {
    float relay_amplitude;      /* relay output magnitude (rad/s, default 0.3) */
    float dt;                   /* control loop timestep (seconds) */
    uint32_t settle_cycles;     /* number of oscillation cycles before measuring (default 4) */
    uint32_t measure_cycles;    /* number of cycles to measure (default 4) */
    float max_duration_s;       /* maximum autotune duration (default 30s) */
} autotune_config_t;

/**
 * Initialize the autotune module with default configuration.
 */
void autotune_init(float dt);

/**
 * Start autotuning on the specified axis.
 * The vehicle should be hovering in stabilize mode.
 */
void autotune_start(autotune_axis_t axis);

/**
 * Stop and cancel the autotune process.
 */
void autotune_stop(void);

/**
 * Update the autotune state machine. Called every control loop iteration.
 *
 * @param rate_measurement Current angular rate on the tuning axis (rad/s)
 * @param relay_output     Output: the relay signal to apply as rate setpoint override
 * @return true if relay_output should be used as override, false if autotune is not active
 */
bool autotune_update(float rate_measurement, float *relay_output);

/**
 * Get current autotune state.
 */
autotune_state_t autotune_get_state(void);

/**
 * Check if autotune has completed successfully.
 */
bool autotune_is_complete(void);

/**
 * Get the computed PID gains. Only valid when state == AUTOTUNE_STATE_COMPLETE.
 */
autotune_gains_t autotune_get_gains(void);

/**
 * Get the axis currently being tuned.
 */
autotune_axis_t autotune_get_axis(void);
