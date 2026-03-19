/*
 * Helicopter CCPM Swashplate Mixer
 * 120-degree CCPM with throttle-collective curve
 */
#pragma once

#include <stdint.h>
#include "../uorb/topics/actuator_controls.h"

typedef struct {
    float servo1_us;    /* PWM pulse width in microseconds - servo at 0 deg */
    float servo2_us;    /* PWM pulse width in microseconds - servo at 120 deg */
    float servo3_us;    /* PWM pulse width in microseconds - servo at 240 deg */
    float tail_esc_us;  /* Tail rotor ESC pulse width */
    float main_esc_us;  /* Main rotor ESC pulse width */
} heli_mixer_output_t;

typedef struct {
    float ccpm_angle_offset;     /* phase angle offset in degrees */
    float collective_range;       /* collective pitch range (servo travel) */
    float cyclic_range;          /* cyclic pitch range (servo travel) */
    float servo_center_us;       /* center pulse width, typically 1500 */
    float servo_range_us;        /* +/- range from center, typically 500 */
    float esc_min_us;            /* ESC minimum pulse, 1000 */
    float esc_max_us;            /* ESC maximum pulse, 2000 */
    float tail_esc_idle_us;      /* tail ESC idle pulse, 1100 */
    float main_esc_idle_us;      /* main ESC idle pulse, 1100 */
    /* Throttle-collective curve (5 points) */
    float throttle_curve[5];     /* throttle at 0%, 25%, 50%, 75%, 100% collective */
    /* Collective pitch curve (5 points) */
    float collective_curve[5];   /* collective pitch at 0%, 25%, 50%, 75%, 100% stick */
} heli_mixer_config_t;

/**
 * Initialize mixer with default helicopter configuration.
 */
void heli_mixer_init(heli_mixer_config_t *config);

/**
 * Compute servo and ESC outputs from normalized actuator controls.
 */
void heli_mixer_update(const heli_mixer_config_t *config,
                       const actuator_controls_t *controls,
                       heli_mixer_output_t *output);
