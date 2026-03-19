/*
 * System Monitor Agent Implementation
 *
 * Runs at 10 Hz on Core 0.
 * Responsibilities:
 *   1. Read battery voltage via ADC and publish BATTERY_STATUS
 *   2. Check sensor health by timestamp freshness
 *   3. Determine failsafe state
 *   4. Handle arm/disarm logic with safety checks
 *   5. Publish VEHICLE_STATUS
 *   6. Blink status LED based on vehicle state
 *   7. Check for RPC commands (arm/disarm, mode change)
 */

#include "sysmon_agent.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

#include "../common/board_config.h"
#include "../common/flight_modes.h"
#include "../common/math_utils.h"
#include "../uorb/uorb.h"
#include "../uorb/topics/sensor_imu.h"
#include "../uorb/topics/sensor_baro.h"
#include "../uorb/topics/sensor_mag.h"
#include "../uorb/topics/sensor_gps.h"
#include "../uorb/topics/rc_channels.h"
#include "../uorb/topics/battery_status.h"
#include "../uorb/topics/vehicle_status.h"
#include "../rpc/rpc_core.h"
#include "../rpc/rpc_messages.h"
#include "../rpc/rpc_telemetry.h"
#include "../uorb/topics/vehicle_attitude.h"
#include "../uorb/topics/vehicle_local_position.h"

static const char *TAG = "sysmon_agent";

/* Battery config */
#define BATT_LOW_VOLTAGE        10.5f   /* 3S LiPo low: 3.5V/cell */
#define BATT_CRITICAL_VOLTAGE   9.6f    /* 3S LiPo critical: 3.2V/cell */
#define BATT_VOLTAGE_FILTER_K   0.05f   /* low-pass filter constant */
#define BATT_ADC_VREF           3.3f    /* ADC reference voltage */
#define BATT_ADC_BITS           4095.0f /* 12-bit ADC */

/* Sensor timeout in microseconds */
#define SENSOR_TIMEOUT_US       500000  /* 500 ms */
#define RC_TIMEOUT_US           1000000 /* 1 second */

/* LED timing */
#define LED_SLOW_BLINK_MS       500     /* 1 Hz blink: 500ms on, 500ms off */
#define LED_FAST_BLINK_MS       100     /* 5 Hz blink */

/* ------------------------------------------------------------------ */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

static int init_adc(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        return -1;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, BATT_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
        return -1;
    }

    ESP_LOGI(TAG, "ADC initialized for battery monitoring");
    return 0;
}

static float read_battery_voltage(void)
{
    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, BATT_ADC_CHANNEL, &raw);
    if (err != ESP_OK) {
        return 0.0f;
    }
    /* Convert ADC raw to voltage, then apply divider ratio */
    float adc_voltage = ((float)raw / BATT_ADC_BITS) * BATT_ADC_VREF;
    return adc_voltage * BATT_VOLTAGE_DIVIDER_RATIO;
}

static void init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_STATUS_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_STATUS_LED, 0);
}

static void set_led(bool on)
{
    gpio_set_level(PIN_STATUS_LED, on ? 1 : 0);
}

/* ------------------------------------------------------------------ */
static void sysmon_task(void *param)
{
    rpc_context_t *rpc = (rpc_context_t *)param;

    /* Wait for topics to be advertised by other agents */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Initialize hardware */
    init_adc();
    init_led();

    /* Advertise topics */
    orb_advertise(ORB_ID_BATTERY_STATUS, sizeof(battery_status_t));
    orb_advertise(ORB_ID_VEHICLE_STATUS, sizeof(vehicle_status_t));

    /* Subscribe to sensor topics for health monitoring */
    orb_subscription_t *imu_sub  = orb_subscribe(ORB_ID_SENSOR_IMU);
    orb_subscription_t *baro_sub = orb_subscribe(ORB_ID_SENSOR_BARO);
    orb_subscription_t *mag_sub  = orb_subscribe(ORB_ID_SENSOR_MAG);
    orb_subscription_t *gps_sub  = orb_subscribe(ORB_ID_SENSOR_GPS);
    orb_subscription_t *rc_sub   = orb_subscribe(ORB_ID_RC_CHANNELS);

    /* Subscribe to topics for RPC telemetry forwarding */
    orb_subscription_t *att_sub    = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE);
    orb_subscription_t *lpos_sub   = orb_subscribe(ORB_ID_VEHICLE_LOCAL_POSITION);

    /* State */
    arm_state_t     arm_state     = ARM_STATE_DISARMED;
    flight_mode_t   flight_mode   = FLIGHT_MODE_STABILIZE;
    failsafe_state_t failsafe     = FAILSAFE_NONE;
    float           batt_filtered = 0.0f;
    bool            batt_first    = true;

    /* Timestamps of last received sensor data */
    uint64_t last_imu_ts  = 0;
    uint64_t last_baro_ts = 0;
    uint64_t last_mag_ts  = 0;
    uint64_t last_gps_ts  = 0;
    uint64_t last_rc_ts   = 0;

    /* LED state */
    uint32_t led_counter = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        uint32_t now_ms = (uint32_t)(now_us / 1000);

        /* ---- 1. Battery monitoring ---- */
        float batt_v = read_battery_voltage();
        if (batt_first && batt_v > 1.0f) {
            batt_filtered = batt_v;
            batt_first = false;
        } else if (batt_v > 1.0f) {
            batt_filtered += BATT_VOLTAGE_FILTER_K * (batt_v - batt_filtered);
        }

        battery_status_t batt_msg = {0};
        batt_msg.timestamp_us = now_us;
        batt_msg.voltage_v = batt_v;
        batt_msg.voltage_filtered = batt_filtered;

        /* Estimate cell count from voltage */
        if (batt_filtered > 16.8f) {
            batt_msg.cell_count = 6; /* 6S */
        } else if (batt_filtered > 12.6f) {
            batt_msg.cell_count = 4; /* 4S */
        } else if (batt_filtered > 8.4f) {
            batt_msg.cell_count = 3; /* 3S */
        } else if (batt_filtered > 4.2f) {
            batt_msg.cell_count = 2; /* 2S */
        } else {
            batt_msg.cell_count = 1;
        }

        if (batt_msg.cell_count > 0) {
            batt_msg.voltage_per_cell = batt_filtered / (float)batt_msg.cell_count;
        }

        batt_msg.warning = (batt_filtered < BATT_LOW_VOLTAGE) && (batt_filtered > 1.0f);
        batt_msg.critical = (batt_filtered < BATT_CRITICAL_VOLTAGE) && (batt_filtered > 1.0f);

        orb_publish(ORB_ID_BATTERY_STATUS, &batt_msg);

        /* ---- 2. Check sensor health (timestamp freshness) ---- */
        {
            sensor_imu_t imu;
            if (orb_copy(imu_sub, &imu) == 0) {
                last_imu_ts = imu.timestamp_us;
            }
            sensor_baro_t baro;
            if (orb_copy(baro_sub, &baro) == 0) {
                last_baro_ts = baro.timestamp_us;
            }
            sensor_mag_t mag;
            if (orb_copy(mag_sub, &mag) == 0) {
                last_mag_ts = mag.timestamp_us;
            }
            sensor_gps_t gps;
            if (orb_copy(gps_sub, &gps) == 0) {
                last_gps_ts = gps.timestamp_us;
            }
            rc_channels_t rc;
            if (orb_copy(rc_sub, &rc) == 0) {
                last_rc_ts = rc.timestamp_us;
            }
        }

        bool imu_ok  = (last_imu_ts > 0)  && ((now_us - last_imu_ts) < SENSOR_TIMEOUT_US);
        bool baro_ok = (last_baro_ts > 0) && ((now_us - last_baro_ts) < SENSOR_TIMEOUT_US);
        bool mag_ok  = (last_mag_ts > 0)  && ((now_us - last_mag_ts) < SENSOR_TIMEOUT_US);
        bool gps_ok  = (last_gps_ts > 0)  && ((now_us - last_gps_ts) < SENSOR_TIMEOUT_US);
        bool rc_ok   = (last_rc_ts > 0)   && ((now_us - last_rc_ts) < RC_TIMEOUT_US);

        /* ---- 3. Determine failsafe state ---- */
        failsafe = FAILSAFE_NONE;

        if (batt_msg.critical) {
            failsafe = FAILSAFE_BATTERY_CRITICAL;
        } else if (batt_msg.warning) {
            failsafe = FAILSAFE_BATTERY_LOW;
        }

        if (!rc_ok && arm_state == ARM_STATE_ARMED) {
            failsafe = FAILSAFE_RC_LOST;
        }

        if (!imu_ok) {
            failsafe = FAILSAFE_SENSOR_FAILURE;
        }

        /* ---- 4. Arm/disarm logic ---- */
        /* Auto-disarm on critical failsafe */
        if (failsafe == FAILSAFE_BATTERY_CRITICAL ||
            failsafe == FAILSAFE_SENSOR_FAILURE) {
            if (arm_state == ARM_STATE_ARMED) {
                ESP_LOGW(TAG, "FAILSAFE: auto-disarming (failsafe=%d)", failsafe);
                arm_state = ARM_STATE_DISARMED;
            }
        }

        /* ---- 7. Check RPC commands ---- */
        if (rpc != NULL) {
            rpc_command_msg_t cmd;
            while (rpc_receive_command(rpc, &cmd, 0) == 0) {
                switch (cmd.msg_type) {
                case RPC_CMD_ARM:
                    if (cmd.data.arm_cmd.arm) {
                        /* Safety checks for arming */
                        if (!imu_ok) {
                            ESP_LOGW(TAG, "ARM rejected: IMU not healthy");
                        } else if (failsafe != FAILSAFE_NONE &&
                                   failsafe != FAILSAFE_BATTERY_LOW) {
                            ESP_LOGW(TAG, "ARM rejected: failsafe active (%d)", failsafe);
                        } else {
                            ESP_LOGI(TAG, "ARMED via RPC");
                            arm_state = ARM_STATE_ARMED;
                        }
                    }
                    break;

                case RPC_CMD_DISARM:
                    ESP_LOGI(TAG, "DISARMED via RPC");
                    arm_state = ARM_STATE_DISARMED;
                    break;

                case RPC_CMD_SET_MODE:
                    if (cmd.data.mode_cmd.mode < FLIGHT_MODE_COUNT) {
                        flight_mode = (flight_mode_t)cmd.data.mode_cmd.mode;
                        ESP_LOGI(TAG, "Flight mode set to %d via RPC", flight_mode);
                    }
                    break;

                default:
                    break;
                }
            }
        }

        /* ---- 5. Publish vehicle status ---- */
        vehicle_status_t status_msg = {0};
        status_msg.timestamp_us = now_us;
        status_msg.arm_state = arm_state;
        status_msg.flight_mode = flight_mode;
        status_msg.failsafe = failsafe;
        status_msg.sensor_imu_ok = imu_ok;
        status_msg.sensor_baro_ok = baro_ok;
        status_msg.sensor_mag_ok = mag_ok;
        status_msg.sensor_gps_ok = gps_ok;
        status_msg.rc_ok = rc_ok;
        status_msg.battery_ok = !batt_msg.warning && !batt_msg.critical;

        orb_publish(ORB_ID_VEHICLE_STATUS, &status_msg);

        /* ---- 5b. Forward telemetry via RPC to Core 1 ---- */
        if (rpc != NULL) {
            /* Attitude (every cycle = 10 Hz) */
            vehicle_attitude_t att_data;
            if (orb_copy(att_sub, &att_data) == 0) {
                rpc_telem_send_attitude(rpc, &att_data);
            }

            /* GPS */
            sensor_gps_t gps_data;
            if (orb_copy(gps_sub, &gps_data) == 0) {
                rpc_telem_send_gps(rpc, &gps_data);
            }

            /* Altitude from local position */
            vehicle_local_position_t lpos_data;
            if (orb_copy(lpos_sub, &lpos_data) == 0) {
                rpc_telem_send_altitude(rpc, &lpos_data);
            }

            /* Battery */
            rpc_telem_send_battery(rpc, &batt_msg);

            /* Vehicle status */
            rpc_telem_send_status(rpc, &status_msg);
        }

        /* ---- 6. LED indication ---- */
        led_counter++;
        switch (arm_state) {
        case ARM_STATE_DISARMED:
            if (failsafe != FAILSAFE_NONE) {
                /* Fast blink (5 Hz): toggle every 100ms = every 1 cycle at 10Hz */
                set_led((led_counter % 2) == 0);
            } else {
                /* Slow blink (1 Hz): toggle every 500ms = every 5 cycles at 10Hz */
                set_led((led_counter % 10) < 5);
            }
            break;

        case ARM_STATE_ARMED:
            /* Solid on */
            set_led(true);
            break;

        default:
            set_led(false);
            break;
        }

        /* 10 Hz loop */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}

void sysmon_agent_start(rpc_context_t *rpc)
{
    ESP_LOGI(TAG, "Starting system monitor agent on Core %d, priority %d",
             FC_CORE, SYSMON_TASK_PRIORITY);

    xTaskCreatePinnedToCore(
        sysmon_task,
        "sysmon_agent",
        SYSMON_TASK_STACK,
        (void *)rpc,
        SYSMON_TASK_PRIORITY,
        NULL,
        FC_CORE
    );
}
