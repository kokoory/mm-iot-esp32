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
#include "../common/param.h"
#include "../uorb/topics/vehicle_attitude.h"
#include "../uorb/topics/vehicle_local_position.h"

static const char *TAG = "sysmon_agent";

/* Battery config */
#define BATT_VOLTAGE_FILTER_K   0.05f   /* low-pass filter constant */
#define BATT_ADC_VREF           3.3f    /* ADC reference voltage */
#define BATT_ADC_BITS           4095.0f /* 12-bit ADC */

/* RC timeout in microseconds (fixed, not a tunable param) */
#define RC_TIMEOUT_US           1000000 /* 1 second */

/* LED timing */
#define LED_SLOW_BLINK_MS       500     /* 1 Hz blink: 500ms on, 500ms off */
#define LED_FAST_BLINK_MS       100     /* 5 Hz blink */

/* ------------------------------------------------------------------ */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_channel_t s_batt_adc_channel = ADC_CHANNEL_0;

static int init_adc(void)
{
    /* Discover ADC unit and channel for the battery GPIO at runtime */
    adc_unit_t adc_unit;
    esp_err_t err = adc_oneshot_io_to_channel(PIN_BATT_ADC, &adc_unit, &s_batt_adc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d is not a valid ADC pin: %s", PIN_BATT_ADC, esp_err_to_name(err));
        return -1;
    }
    ESP_LOGI(TAG, "Battery ADC: GPIO %d -> unit %d, channel %d",
             PIN_BATT_ADC, adc_unit, s_batt_adc_channel);

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = adc_unit,
    };
    err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        return -1;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, s_batt_adc_channel, &chan_cfg);
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
    esp_err_t err = adc_oneshot_read(s_adc_handle, s_batt_adc_channel, &raw);
    if (err != ESP_OK) {
        return 0.0f;
    }
    /* Convert ADC raw to voltage, then apply divider ratio */
    float adc_voltage = ((float)raw / BATT_ADC_BITS) * BATT_ADC_VREF;
    return adc_voltage * param_get(PARAM_BATT_VDIV_RATIO);
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

/* ── Param helpers ────────────────────────────────────────────── */

static void send_param_value_rpc(rpc_context_t *rpc, param_id_t id)
{
    const param_meta_t *meta = param_get_meta(id);
    if (meta == NULL) return;

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_PARAM_VALUE;
    msg.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    strncpy(msg.data.param_value.name, meta->name, 16);
    msg.data.param_value.name[16] = '\0';
    msg.data.param_value.value = param_get(id);
    msg.data.param_value.type = 9; /* MAV_PARAM_TYPE_REAL32 */
    msg.data.param_value.count = param_count();
    msg.data.param_value.index = (uint16_t)id;

    rpc_send_telemetry(rpc, &msg);
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

        batt_msg.warning = (batt_filtered < param_get(PARAM_BATT_LOW_V)) && (batt_filtered > 1.0f);
        batt_msg.critical = (batt_filtered < param_get(PARAM_BATT_CRIT_V)) && (batt_filtered > 1.0f);

        orb_publish(ORB_ID_BATTERY_STATUS, &batt_msg);

        /* ---- 2. Check sensor health (timestamp freshness) ---- */
        /* Save GPS data for reuse in RPC forwarding (avoid double-consume) */
        sensor_gps_t gps_data;
        bool gps_updated = false;
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
            if (orb_copy(gps_sub, &gps_data) == 0) {
                last_gps_ts = gps_data.timestamp_us;
                gps_updated = true;
            }
            rc_channels_t rc;
            if (orb_copy(rc_sub, &rc) == 0) {
                last_rc_ts = rc.timestamp_us;
            }
        }

        bool imu_ok  = (last_imu_ts > 0)  && ((now_us - last_imu_ts) < (uint64_t)(param_get(PARAM_SENSOR_TIMEOUT_MS) * 1000.0f));
        bool baro_ok = (last_baro_ts > 0) && ((now_us - last_baro_ts) < (uint64_t)(param_get(PARAM_SENSOR_TIMEOUT_MS) * 1000.0f));
        bool mag_ok  = (last_mag_ts > 0)  && ((now_us - last_mag_ts) < (uint64_t)(param_get(PARAM_SENSOR_TIMEOUT_MS) * 1000.0f));
        bool gps_ok  = (last_gps_ts > 0)  && ((now_us - last_gps_ts) < (uint64_t)(param_get(PARAM_SENSOR_TIMEOUT_MS) * 1000.0f));
        bool rc_ok   = (last_rc_ts > 0)   && ((now_us - last_rc_ts) < RC_TIMEOUT_US);

        /* ---- 3. Determine failsafe state (highest severity wins) ---- */
        /* Priority: SENSOR_FAILURE > BATTERY_CRITICAL > RC_LOST > BATTERY_LOW > NONE */
        failsafe_state_t new_failsafe = FAILSAFE_NONE;

        if (batt_msg.warning) {
            new_failsafe = FAILSAFE_BATTERY_LOW;
        }
        if (!rc_ok && arm_state == ARM_STATE_ARMED) {
            new_failsafe = FAILSAFE_RC_LOST;
        }
        if (batt_msg.critical) {
            new_failsafe = FAILSAFE_BATTERY_CRITICAL;
        }
        if (!imu_ok) {
            new_failsafe = FAILSAFE_SENSOR_FAILURE;
        }

        /* Log failsafe transitions */
        if (new_failsafe != failsafe && arm_state == ARM_STATE_ARMED) {
            ESP_LOGW(TAG, "FAILSAFE: %d -> %d", failsafe, new_failsafe);
        }
        failsafe = new_failsafe;

        /* ---- 4. Failsafe actions ---- */
        if (arm_state == ARM_STATE_ARMED) {
            switch (failsafe) {
            case FAILSAFE_RC_LOST:
                /* RC lost: trigger RTH if GPS available, otherwise LAND */
                if (flight_mode != FLIGHT_MODE_RTH &&
                    flight_mode != FLIGHT_MODE_LAND) {
                    if (gps_ok) {
                        ESP_LOGW(TAG, "FAILSAFE RC_LOST: switching to RTH");
                        flight_mode = FLIGHT_MODE_RTH;
                    } else {
                        ESP_LOGW(TAG, "FAILSAFE RC_LOST: no GPS, switching to LAND");
                        flight_mode = FLIGHT_MODE_LAND;
                    }
                }
                break;

            case FAILSAFE_BATTERY_LOW:
                /* Battery low: warn only (GCS notification via MAVLink) */
                break;

            case FAILSAFE_BATTERY_CRITICAL:
                /* Battery critical: force LAND immediately */
                if (flight_mode != FLIGHT_MODE_LAND) {
                    ESP_LOGW(TAG, "FAILSAFE BATTERY_CRITICAL: forcing LAND");
                    flight_mode = FLIGHT_MODE_LAND;
                }
                break;

            case FAILSAFE_SENSOR_FAILURE:
                /* IMU failure: immediate disarm (no reliable control possible) */
                ESP_LOGE(TAG, "FAILSAFE SENSOR_FAILURE: IMU lost, disarming!");
                arm_state = ARM_STATE_DISARMED;
                break;

            case FAILSAFE_GCS_LOST:
                /* GCS link lost: continue current mode, warn only.
                 * Pilot should have RC as backup. */
                break;

            case FAILSAFE_NONE:
            default:
                break;
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
                        } else if (!rc_ok) {
                            ESP_LOGW(TAG, "ARM rejected: no RC signal");
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

                case RPC_CMD_RC_OVERRIDE: {
                    /* Publish RC override from GCS to uORB RC_CHANNELS topic */
                    rc_channels_t rc_msg = {0};
                    rc_msg.timestamp_us = now_us;
                    rc_msg.channel_count = 8;
                    rc_msg.signal_lost = false;
                    for (int i = 0; i < 8; i++) {
                        rc_msg.channels[i] = (float)cmd.data.rc_override.channels[i] / 1000.0f;
                    }
                    orb_publish(ORB_ID_RC_CHANNELS, &rc_msg);
                    break;
                }

                case RPC_CMD_PARAM_REQUEST_READ: {
                    param_id_t pid;
                    if (cmd.data.param_request.index >= 0 &&
                        cmd.data.param_request.index < (int16_t)param_count()) {
                        pid = (param_id_t)cmd.data.param_request.index;
                    } else {
                        pid = param_find(cmd.data.param_request.name);
                    }
                    if (pid < PARAM_COUNT) {
                        send_param_value_rpc(rpc, pid);
                    } else {
                        ESP_LOGW(TAG, "PARAM_REQUEST_READ: param not found");
                    }
                    break;
                }

                case RPC_CMD_PARAM_REQUEST_LIST: {
                    ESP_LOGI(TAG, "Sending all %d params", param_count());
                    for (int i = 0; i < param_count(); i++) {
                        send_param_value_rpc(rpc, (param_id_t)i);
                    }
                    break;
                }

                case RPC_CMD_PARAM_SET: {
                    param_id_t pid = param_find(cmd.data.param_set.name);
                    if (pid < PARAM_COUNT) {
                        param_set(pid, cmd.data.param_set.value);
                        send_param_value_rpc(rpc, pid);
                    } else {
                        ESP_LOGW(TAG, "PARAM_SET: param '%s' not found",
                                 cmd.data.param_set.name);
                    }
                    break;
                }

                case RPC_CMD_PARAM_SAVE:
                    ESP_LOGI(TAG, "Saving params to NVS");
                    param_save_all();
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

            /* GPS (reuse data from health check to avoid double-consume) */
            if (gps_updated) {
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
