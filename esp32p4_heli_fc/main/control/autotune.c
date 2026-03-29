/*
 * PID Autotune - Relay Feedback Method Implementation
 *
 * Uses Astrom-Hagglund relay feedback to determine the ultimate gain (Ku)
 * and ultimate period (Tu), then applies Ziegler-Nichols "some overshoot"
 * formulas to compute PID gains:
 *
 *   Kp = 0.33 * Ku
 *   Ki = Kp / (0.5 * Tu)
 *   Kd = Kp * (0.125 * Tu)
 *
 * These conservative gains (vs classic Z-N) are better suited for
 * rotorcraft rate controllers.
 */

#include "autotune.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "autotune";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Internal state */
static autotune_state_t s_state = AUTOTUNE_STATE_IDLE;
static autotune_axis_t  s_axis = AUTOTUNE_AXIS_ROLL;
static autotune_config_t s_config;
static autotune_gains_t s_gains;

/* Relay state */
static float    s_relay_sign;           /* +1 or -1 */
static float    s_elapsed_s;            /* total elapsed time */
static float    s_rate_baseline;        /* rate at start */

/* Zero-crossing detection */
static uint32_t s_zero_cross_count;     /* number of zero crossings detected */
static float    s_last_rate;            /* previous rate measurement */
static float    s_cross_times[32];      /* timestamps of zero crossings */

/* Amplitude measurement */
static float    s_peak_pos;             /* max positive rate deviation */
static float    s_peak_neg;             /* max negative rate deviation */
static float    s_sum_amplitude;        /* sum of half-cycle amplitudes */
static uint32_t s_amplitude_count;      /* number of half-cycles measured */

void autotune_init(float dt)
{
    memset(&s_config, 0, sizeof(s_config));
    s_config.relay_amplitude = 0.3f;
    s_config.dt = dt;
    s_config.settle_cycles = 4;
    s_config.measure_cycles = 4;
    s_config.max_duration_s = 30.0f;

    s_state = AUTOTUNE_STATE_IDLE;
    memset(&s_gains, 0, sizeof(s_gains));
}

void autotune_start(autotune_axis_t axis)
{
    if (s_state == AUTOTUNE_STATE_RUNNING) {
        ESP_LOGW(TAG, "Autotune already running");
        return;
    }

    s_axis = axis;
    s_state = AUTOTUNE_STATE_RUNNING;

    /* Reset relay state */
    s_relay_sign = 1.0f;
    s_elapsed_s = 0.0f;
    s_rate_baseline = 0.0f;
    s_zero_cross_count = 0;
    s_last_rate = 0.0f;
    s_peak_pos = 0.0f;
    s_peak_neg = 0.0f;
    s_sum_amplitude = 0.0f;
    s_amplitude_count = 0;
    memset(s_cross_times, 0, sizeof(s_cross_times));
    memset(&s_gains, 0, sizeof(s_gains));

    ESP_LOGI(TAG, "Autotune started: axis=%d amplitude=%.2f",
             axis, s_config.relay_amplitude);
}

void autotune_stop(void)
{
    if (s_state == AUTOTUNE_STATE_RUNNING) {
        s_state = AUTOTUNE_STATE_IDLE;
        ESP_LOGI(TAG, "Autotune stopped");
    }
}

bool autotune_update(float rate_measurement, float *relay_output)
{
    if (s_state != AUTOTUNE_STATE_RUNNING) {
        return false;
    }

    float dt = s_config.dt;
    s_elapsed_s += dt;

    /* Timeout check */
    if (s_elapsed_s > s_config.max_duration_s) {
        ESP_LOGW(TAG, "Autotune timeout after %.1fs", s_elapsed_s);
        s_state = AUTOTUNE_STATE_FAILED;
        return false;
    }

    /* Capture baseline on first sample */
    if (s_elapsed_s <= dt * 1.5f) {
        s_rate_baseline = rate_measurement;
        s_last_rate = rate_measurement - s_rate_baseline;
        *relay_output = s_relay_sign * s_config.relay_amplitude;
        return true;
    }

    float rate = rate_measurement - s_rate_baseline;

    /* Zero-crossing detection (relay switches on zero crossing) */
    if ((s_last_rate >= 0.0f && rate < 0.0f) ||
        (s_last_rate < 0.0f && rate >= 0.0f)) {

        /* Record zero crossing time */
        if (s_zero_cross_count < 32) {
            s_cross_times[s_zero_cross_count] = s_elapsed_s;
        }
        s_zero_cross_count++;

        /* Switch relay direction */
        s_relay_sign = (rate < 0.0f) ? 1.0f : -1.0f;

        /* Record peak amplitude from the half-cycle that just ended */
        uint32_t total_needed = (s_config.settle_cycles + s_config.measure_cycles) * 2;
        if (s_zero_cross_count > s_config.settle_cycles * 2) {
            /* We're in the measurement phase */
            float half_amp = (s_peak_pos - s_peak_neg) / 2.0f;
            s_sum_amplitude += half_amp;
            s_amplitude_count++;
        }

        /* Reset peak tracking for next half-cycle */
        s_peak_pos = 0.0f;
        s_peak_neg = 0.0f;

        /* Check if we have enough data */
        if (s_zero_cross_count >= total_needed && s_amplitude_count >= 2) {
            /* Compute ultimate period Tu from zero-crossing times */
            uint32_t settle_crossings = s_config.settle_cycles * 2;
            uint32_t measure_crossings = s_zero_cross_count - settle_crossings;
            if (measure_crossings < 2) {
                s_state = AUTOTUNE_STATE_FAILED;
                ESP_LOGW(TAG, "Not enough crossings for measurement");
                return false;
            }

            /* Period = time span / number of full cycles */
            uint32_t idx_start = settle_crossings;
            uint32_t idx_end = (s_zero_cross_count < 32) ? s_zero_cross_count - 1 : 31;
            float time_span = s_cross_times[idx_end] - s_cross_times[idx_start];
            uint32_t full_cycles = (idx_end - idx_start) / 2;

            if (full_cycles == 0 || time_span <= 0.0f) {
                s_state = AUTOTUNE_STATE_FAILED;
                ESP_LOGW(TAG, "Invalid period measurement");
                return false;
            }

            float Tu = time_span / (float)full_cycles;
            float avg_amplitude = s_sum_amplitude / (float)s_amplitude_count;

            /* Ultimate gain: Ku = 4*d / (pi*a)
             * where d = relay amplitude, a = oscillation amplitude */
            if (avg_amplitude < 0.001f) {
                s_state = AUTOTUNE_STATE_FAILED;
                ESP_LOGW(TAG, "Oscillation amplitude too small");
                return false;
            }

            float Ku = (4.0f * s_config.relay_amplitude) / (M_PI * avg_amplitude);

            /* Conservative Ziegler-Nichols ("some overshoot") for rate PID */
            s_gains.kp = 0.33f * Ku;
            s_gains.ki = s_gains.kp / (0.5f * Tu);
            s_gains.kd = s_gains.kp * (0.125f * Tu);

            s_state = AUTOTUNE_STATE_COMPLETE;
            ESP_LOGI(TAG, "Autotune complete: Tu=%.3fs Ku=%.3f amp=%.4f",
                     Tu, Ku, avg_amplitude);
            ESP_LOGI(TAG, "  Gains: Kp=%.4f Ki=%.4f Kd=%.6f",
                     s_gains.kp, s_gains.ki, s_gains.kd);
            return false;
        }
    }

    /* Track peak rates in current half-cycle */
    if (rate > s_peak_pos) s_peak_pos = rate;
    if (rate < s_peak_neg) s_peak_neg = rate;

    s_last_rate = rate;

    /* Output relay signal */
    *relay_output = s_relay_sign * s_config.relay_amplitude;
    return true;
}

autotune_state_t autotune_get_state(void)
{
    return s_state;
}

bool autotune_is_complete(void)
{
    return s_state == AUTOTUNE_STATE_COMPLETE;
}

autotune_gains_t autotune_get_gains(void)
{
    return s_gains;
}

autotune_axis_t autotune_get_axis(void)
{
    return s_axis;
}
