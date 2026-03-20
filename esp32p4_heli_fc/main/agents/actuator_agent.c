/*
 * Actuator Output Agent Implementation
 *
 * Runs at 500 Hz on Core 0 with the highest FC priority (7).
 * Uses ESP-IDF MCPWM driver to output:
 *   - 3x CCPM swashplate servos (50 Hz PWM, 1000-2000 us)
 *   - 1x tail rotor ESC (50 Hz PWM, 1000-2000 us)
 *   - 1x main rotor ESC (50 Hz PWM, 1000-2000 us)
 *
 * When armed: runs the helicopter CCPM mixer and outputs PWM.
 * When disarmed: servos to center (1500 us), ESCs to minimum (1000 us).
 */

#include "actuator_agent.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm_prelude.h"


#include "../common/board_config.h"
#include "../common/flight_modes.h"
#include "../common/math_utils.h"
#include "../uorb/uorb.h"
#include "../uorb/topics/actuator_controls.h"
#include "../uorb/topics/vehicle_status.h"
#include "../control/heli_mixer.h"

static const char *TAG = "actuator_agent";

/* PWM config */
#define SERVO_PWM_FREQ_HZ       50      /* 50 Hz = 20 ms period */
#define SERVO_TIMEBASE_PERIOD   20000   /* 20000 us = 20 ms */
#define SERVO_TIMEBASE_RES_HZ   1000000 /* 1 MHz = 1 us resolution */

/* Number of PWM channels */
#define NUM_PWM_CHANNELS  5

/* Safe (disarmed) values */
#define SERVO_SAFE_US   1500
#define ESC_SAFE_US     1000

/* Watchdog: if no new actuator_controls for this long, disarm outputs */
#define ACTUATOR_TIMEOUT_US  100000  /* 100 ms */

/* MCPWM handles */
static mcpwm_timer_handle_t     s_timers[NUM_PWM_CHANNELS];
static mcpwm_oper_handle_t      s_operators[NUM_PWM_CHANNELS];
static mcpwm_cmpr_handle_t      s_comparators[NUM_PWM_CHANNELS];
static mcpwm_gen_handle_t       s_generators[NUM_PWM_CHANNELS];

/* Pin assignments indexed by channel */
static const int s_pwm_pins[NUM_PWM_CHANNELS] = {
    PIN_SWASH_SERVO_1,
    PIN_SWASH_SERVO_2,
    PIN_SWASH_SERVO_3,
    PIN_TAIL_ESC,
    PIN_MAIN_ESC,
};

/* ------------------------------------------------------------------ */
static int init_mcpwm(void)
{
    /*
     * Create one timer + operator + comparator + generator per channel.
     * All timers run at 50 Hz with 1 us resolution.
     * ESP32-P4 supports max 3 timers/operators per MCPWM group, so:
     *   Group 0: channels 0-2 (3 swashplate servos)
     *   Group 1: channels 3-4 (tail ESC, main ESC)
     */
    for (int i = 0; i < NUM_PWM_CHANNELS; i++) {
        int group = (i < 3) ? 0 : 1;
        /* Timer */
        mcpwm_timer_config_t timer_cfg = {
            .group_id = group,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = SERVO_TIMEBASE_RES_HZ,
            .period_ticks = SERVO_TIMEBASE_PERIOD,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        esp_err_t err = mcpwm_new_timer(&timer_cfg, &s_timers[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create MCPWM timer %d: %s", i, esp_err_to_name(err));
            return -1;
        }

        /* Operator */
        mcpwm_operator_config_t oper_cfg = {
            .group_id = group,
        };
        err = mcpwm_new_operator(&oper_cfg, &s_operators[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create MCPWM operator %d: %s", i, esp_err_to_name(err));
            return -1;
        }

        /* Connect operator to timer */
        err = mcpwm_operator_connect_timer(s_operators[i], s_timers[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect operator to timer %d: %s", i, esp_err_to_name(err));
            return -1;
        }

        /* Comparator */
        mcpwm_comparator_config_t cmp_cfg = {
            .flags.update_cmp_on_tez = true,
        };
        err = mcpwm_new_comparator(s_operators[i], &cmp_cfg, &s_comparators[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create comparator %d: %s", i, esp_err_to_name(err));
            return -1;
        }

        /* Generator */
        mcpwm_generator_config_t gen_cfg = {
            .gen_gpio_num = s_pwm_pins[i],
        };
        err = mcpwm_new_generator(s_operators[i], &gen_cfg, &s_generators[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create generator %d: %s", i, esp_err_to_name(err));
            return -1;
        }

        /* Set generator actions:
         *   - Go high on timer zero (start of period)
         *   - Go low on comparator match (end of pulse)
         */
        err = mcpwm_generator_set_action_on_timer_event(s_generators[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                          MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_HIGH));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set timer action %d: %s", i, esp_err_to_name(err));
            return -1;
        }

        err = mcpwm_generator_set_action_on_compare_event(s_generators[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                            s_comparators[i],
                                            MCPWM_GEN_ACTION_LOW));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set compare action %d: %s", i, esp_err_to_name(err));
            return -1;
        }

        /* Set initial safe compare value */
        uint32_t safe_val = (i < 3) ? SERVO_SAFE_US : ESC_SAFE_US;
        mcpwm_comparator_set_compare_value(s_comparators[i], safe_val);

        /* Enable and start timer */
        mcpwm_timer_enable(s_timers[i]);
        mcpwm_timer_start_stop(s_timers[i], MCPWM_TIMER_START_NO_STOP);
    }

    ESP_LOGI(TAG, "MCPWM initialized: %d channels", NUM_PWM_CHANNELS);
    return 0;
}

static void set_pwm_us(int channel, float pulse_us)
{
    if (channel < 0 || channel >= NUM_PWM_CHANNELS) return;
    uint32_t cmp = (uint32_t)constrain_f(pulse_us, 800.0f, 2200.0f);
    mcpwm_comparator_set_compare_value(s_comparators[channel], cmp);
}

/* ------------------------------------------------------------------ */
static void actuator_task(void *param)
{
    (void)param;

    /* Initialize MCPWM */
    if (init_mcpwm() != 0) {
        ESP_LOGE(TAG, "MCPWM init failed, task aborting");
        vTaskDelete(NULL);
        return;
    }

    /* Wait for topics to be advertised */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Subscribe to topics */
    orb_subscription_t *act_sub  = orb_subscribe(ORB_ID_ACTUATOR_CONTROLS);
    orb_subscription_t *stat_sub = orb_subscribe(ORB_ID_VEHICLE_STATUS);

    /* Initialize mixer */
    heli_mixer_config_t mixer_config;
    heli_mixer_init(&mixer_config);

    /* Local state */
    actuator_controls_t act = {0};
    vehicle_status_t    status = {0};

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();

        /* Read latest data */
        orb_copy(act_sub, &act);
        orb_copy(stat_sub, &status);

        /* Watchdog: check actuator_controls freshness */
        bool act_timeout = (act.timestamp_us > 0) &&
                           ((now_us - act.timestamp_us) > ACTUATOR_TIMEOUT_US);

        if (status.arm_state == ARM_STATE_ARMED && !act_timeout) {
            /* Armed: run mixer and output */
            heli_mixer_output_t mix_out;
            heli_mixer_update(&mixer_config, &act, &mix_out);

            set_pwm_us(0, mix_out.servo1_us);
            set_pwm_us(1, mix_out.servo2_us);
            set_pwm_us(2, mix_out.servo3_us);
            set_pwm_us(3, mix_out.tail_esc_us);
            set_pwm_us(4, mix_out.main_esc_us);
        } else {
            /* Disarmed: safe values */
            set_pwm_us(0, SERVO_SAFE_US);
            set_pwm_us(1, SERVO_SAFE_US);
            set_pwm_us(2, SERVO_SAFE_US);
            set_pwm_us(3, ESC_SAFE_US);
            set_pwm_us(4, ESC_SAFE_US);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
    }
}

void actuator_agent_start(void)
{
    ESP_LOGI(TAG, "Starting actuator agent on Core %d, priority %d",
             FC_CORE, ACTUATOR_TASK_PRIORITY);

    xTaskCreatePinnedToCore(
        actuator_task,
        "actuator_agent",
        ACTUATOR_TASK_STACK,
        NULL,
        ACTUATOR_TASK_PRIORITY,
        NULL,
        FC_CORE
    );
}
