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
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
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
#include "../uorb/topics/actuator_controls.h"
#include "mission_mgr.h"

static const char *TAG = "sysmon_agent";

/* Battery config */
#define BATT_VOLTAGE_FILTER_K   0.05f   /* low-pass filter constant */
#define BATT_ADC_VREF           3.3f    /* ADC reference voltage */
#define BATT_ADC_BITS           4095.0f /* 12-bit ADC */

/* RC timeout in microseconds (fixed, not a tunable param) */
#define RC_TIMEOUT_US           1000000 /* 1 second */

/* RC switch thresholds for mode mapping (3-position switch) */
#define RC_SW_LOW_THRESHOLD     -0.3f   /* below = position 0 */
#define RC_SW_HIGH_THRESHOLD     0.3f   /* above = position 2, between = position 1 */

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

/* MAV_SEVERITY levels */
#define MAV_SEVERITY_EMERGENCY  0
#define MAV_SEVERITY_ALERT      1
#define MAV_SEVERITY_CRITICAL   2
#define MAV_SEVERITY_ERROR      3
#define MAV_SEVERITY_WARNING    4
#define MAV_SEVERITY_NOTICE     5
#define MAV_SEVERITY_INFO       6

static void send_statustext_rpc(rpc_context_t *rpc, uint8_t severity, const char *text)
{
    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_STATUSTEXT;
    msg.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    msg.data.statustext.severity = severity;
    strncpy(msg.data.statustext.text, text, sizeof(msg.data.statustext.text) - 1);

    rpc_send_telemetry(rpc, &msg);
}

/* ------------------------------------------------------------------ */
static void sysmon_task(void *param)
{
    rpc_context_t *rpc = (rpc_context_t *)param;

    /* Wait for topics to be advertised by other agents */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Initialize mission manager */
    mission_mgr_init();

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
    orb_subscription_t *act_sub    = orb_subscribe(ORB_ID_ACTUATOR_CONTROLS);

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
    uint64_t last_gcs_cmd_ts = 0; /* last RPC command from GCS */

    /* Latest RC channel data for arm/mode switch */
    rc_channels_t last_rc = {0};
    bool rc_arm_prev = false;   /* previous arm switch state (for edge detect) */

    /* Previous sensor health for transition detection */
    bool prev_baro_ok = false;
    bool prev_mag_ok  = false;

    /* Home position (captured on first arm with GPS 3D fix) */
    double home_lat = 0.0, home_lon = 0.0;
    float home_alt_msl = 0.0f;
    bool home_set = false;
    bool home_sent = false; /* true once RPC_MSG_HOME_POSITION sent */

    /* Mission current periodic send timer */
    uint32_t last_mission_current_ms = 0;
    #define MISSION_CURRENT_INTERVAL_MS 1000 /* send every 1 s in mission mode */

    /* Auto-disarm on landing: confirm land_detected for 3 seconds before disarm */
    uint32_t land_detected_start_ms = 0;
    #define LAND_DISARM_CONFIRM_MS 3000 /* 3 second confirmation */

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
        sensor_imu_t imu_data;
        bool imu_updated = false;
        sensor_baro_t baro_data;
        bool baro_updated = false;
        sensor_mag_t mag_data;
        bool mag_updated = false;
        {
            if (orb_copy(imu_sub, &imu_data) == 0) {
                last_imu_ts = imu_data.timestamp_us;
                imu_updated = true;
            }
            if (orb_copy(baro_sub, &baro_data) == 0) {
                last_baro_ts = baro_data.timestamp_us;
                baro_updated = true;
            }
            if (orb_copy(mag_sub, &mag_data) == 0) {
                last_mag_ts = mag_data.timestamp_us;
                mag_updated = true;
            }
            if (orb_copy(gps_sub, &gps_data) == 0) {
                last_gps_ts = gps_data.timestamp_us;
                gps_updated = true;
            }
            rc_channels_t rc;
            if (orb_copy(rc_sub, &rc) == 0) {
                last_rc_ts = rc.timestamp_us;
                last_rc = rc;
            }
        }

        /* ---- Capture home position on first arm with GPS 3D fix ---- */
        if (arm_state == ARM_STATE_ARMED && !home_set &&
            gps_updated && gps_data.fix_type >= 3 && gps_data.satellites >= 6) {
            home_lat = gps_data.latitude;
            home_lon = gps_data.longitude;
            home_alt_msl = gps_data.altitude_msl;
            home_set = true;
            home_sent = false;
            ESP_LOGI(TAG, "HOME set: lat=%.7f lon=%.7f alt_msl=%.1f",
                     home_lat, home_lon, home_alt_msl);

            /* Send home position to GCS via RPC */
            if (rpc != NULL) {
                rpc_telemetry_msg_t hmsg;
                memset(&hmsg, 0, sizeof(hmsg));
                hmsg.msg_type = RPC_MSG_HOME_POSITION;
                hmsg.timestamp_ms = now_ms;
                hmsg.data.home_position.lat = (int32_t)(home_lat * 1e7);
                hmsg.data.home_position.lon = (int32_t)(home_lon * 1e7);
                hmsg.data.home_position.alt = (int32_t)(home_alt_msl * 1000.0f); /* mm MSL */
                rpc_send_telemetry(rpc, &hmsg);
                home_sent = true;
            }
        }
        /* Clear home on disarm so it re-captures on next arm */
        if (arm_state == ARM_STATE_DISARMED && home_set) {
            home_set = false;
            home_sent = false;
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

        /* Log failsafe transitions and notify GCS */
        if (new_failsafe != failsafe && arm_state == ARM_STATE_ARMED) {
            ESP_LOGW(TAG, "FAILSAFE: %d -> %d", failsafe, new_failsafe);
            if (new_failsafe == FAILSAFE_NONE) {
                send_statustext_rpc(rpc, MAV_SEVERITY_INFO, "Failsafe cleared");
            } else if (new_failsafe == FAILSAFE_RC_LOST) {
                send_statustext_rpc(rpc, MAV_SEVERITY_CRITICAL, "RC signal lost");
            } else if (new_failsafe == FAILSAFE_BATTERY_LOW) {
                send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "Battery low");
            } else if (new_failsafe == FAILSAFE_BATTERY_CRITICAL) {
                send_statustext_rpc(rpc, MAV_SEVERITY_CRITICAL, "Battery critical - landing");
            } else if (new_failsafe == FAILSAFE_SENSOR_FAILURE) {
                send_statustext_rpc(rpc, MAV_SEVERITY_EMERGENCY, "Sensor failure - disarming");
            }
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
                /* IMU failure: switch to LAND mode for controlled descent.
                 * Immediate disarm at altitude is more dangerous than attempting
                 * a descent with last known attitude. The actuator watchdog (100ms)
                 * provides a secondary safety net if control loops also fail. */
                if (flight_mode != FLIGHT_MODE_LAND) {
                    ESP_LOGE(TAG, "FAILSAFE SENSOR_FAILURE: IMU lost, forcing LAND!");
                    flight_mode = FLIGHT_MODE_LAND;
                    send_statustext_rpc(rpc, MAV_SEVERITY_EMERGENCY,
                                        "Sensor failure - emergency landing");
                }
                break;

            case FAILSAFE_GCS_LOST:
                /* GCS link lost: continue current mode, warn only.
                 * Pilot should have RC as backup. */
                break;

            case FAILSAFE_NONE:
            default:
                break;
            }

            /* ---- 4b. Sensor degradation: mode fallback (independent of failsafe) ---- */
            /* Baro lost while in altitude-dependent mode → fall back to STABILIZE */
            if (prev_baro_ok && !baro_ok) {
                if (flight_mode == FLIGHT_MODE_ALT_HOLD ||
                    flight_mode == FLIGHT_MODE_LOITER) {
                    ESP_LOGW(TAG, "Baro lost: %s -> STABILIZE",
                             flight_mode == FLIGHT_MODE_ALT_HOLD ? "ALT_HOLD" : "LOITER");
                    flight_mode = FLIGHT_MODE_STABILIZE;
                    send_statustext_rpc(rpc, MAV_SEVERITY_WARNING,
                                        "Baro lost - fallback STABILIZE");
                }
            }
            /* Mag lost while armed → notify pilot (yaw hold degrades) */
            if (prev_mag_ok && !mag_ok) {
                ESP_LOGW(TAG, "Mag lost: heading hold degraded");
                send_statustext_rpc(rpc, MAV_SEVERITY_WARNING,
                                    "Mag lost - heading unreliable");
            }
            /* GPS lost in GPS-dependent modes → fall back to ALT_HOLD or STABILIZE */
            if (!gps_ok && (flight_mode == FLIGHT_MODE_LOITER)) {
                ESP_LOGW(TAG, "GPS lost in LOITER: -> %s",
                         baro_ok ? "ALT_HOLD" : "STABILIZE");
                flight_mode = baro_ok ? FLIGHT_MODE_ALT_HOLD : FLIGHT_MODE_STABILIZE;
                send_statustext_rpc(rpc, MAV_SEVERITY_WARNING,
                                    "GPS lost - mode fallback");
            }
        }
        prev_baro_ok = baro_ok;
        prev_mag_ok  = mag_ok;

        /* ---- 4c. Auto-disarm on landing ---- */
        if (arm_state == ARM_STATE_ARMED) {
            actuator_controls_t act_msg = {0};
            orb_copy(act_sub, &act_msg);
            if (act_msg.land_detected) {
                if (land_detected_start_ms == 0) {
                    land_detected_start_ms = now_ms;
                }
                if ((now_ms - land_detected_start_ms) > LAND_DISARM_CONFIRM_MS) {
                    ESP_LOGI(TAG, "AUTO-DISARM: land_detected confirmed for %dms",
                             LAND_DISARM_CONFIRM_MS);
                    arm_state = ARM_STATE_DISARMED;
                    send_statustext_rpc(rpc, MAV_SEVERITY_INFO,
                                        "Vehicle disarmed (landing)");
                    land_detected_start_ms = 0;
                }
            } else {
                land_detected_start_ms = 0;
            }
        } else {
            land_detected_start_ms = 0;
        }

        /* ---- 4d. RC arm switch (CH5) and mode switch (CH4) ---- */
        if (rc_ok && last_rc.channel_count >= 6) {
            /* --- Arm switch: CH5 rising edge above threshold --- */
            float arm_ch = last_rc.channels[5];
            float arm_threshold = param_get(PARAM_RC_ARM_THRESHOLD);
            bool rc_arm_now = (arm_ch > arm_threshold);

            if (rc_arm_now && !rc_arm_prev && arm_state == ARM_STATE_DISARMED) {
                /* Attempt arm via RC switch */
                float rc_coll = last_rc.channels[2] * 2.0f - 1.0f; /* normalize like flight_ctrl */
                float coll_arm_max = param_get(PARAM_RC_COLL_ARM_MAX);

                if (!imu_ok) {
                    ESP_LOGW(TAG, "RC ARM rejected: IMU not healthy");
                    send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "ARM fail: no IMU");
                } else if (failsafe != FAILSAFE_NONE && failsafe != FAILSAFE_BATTERY_LOW) {
                    ESP_LOGW(TAG, "RC ARM rejected: failsafe active (%d)", failsafe);
                    send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "ARM fail: failsafe");
                } else if (rc_coll > coll_arm_max) {
                    ESP_LOGW(TAG, "RC ARM rejected: collective too high (%.2f > %.2f)",
                             rc_coll, coll_arm_max);
                    send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "ARM fail: coll high");
                } else {
                    ESP_LOGI(TAG, "ARMED via RC switch");
                    arm_state = ARM_STATE_ARMED;
                    send_statustext_rpc(rpc, MAV_SEVERITY_INFO, "Vehicle armed (RC)");
                }
            } else if (!rc_arm_now && rc_arm_prev && arm_state == ARM_STATE_ARMED) {
                /* Disarm via RC switch */
                ESP_LOGI(TAG, "DISARMED via RC switch");
                arm_state = ARM_STATE_DISARMED;
                send_statustext_rpc(rpc, MAV_SEVERITY_INFO, "Vehicle disarmed (RC)");
            }
            rc_arm_prev = rc_arm_now;

            /* --- Mode switch: CH4 (3-position) --- */
            float mode_ch = last_rc.channels[4];
            flight_mode_t rc_mode;
            if (mode_ch < RC_SW_LOW_THRESHOLD) {
                rc_mode = FLIGHT_MODE_STABILIZE;     /* position 0: STABILIZE */
            } else if (mode_ch > RC_SW_HIGH_THRESHOLD) {
                rc_mode = FLIGHT_MODE_ALT_HOLD;      /* position 2: ALT_HOLD */
            } else {
                rc_mode = FLIGHT_MODE_LOITER;        /* position 1: LOITER (requires GPS) */
            }

            /* Only apply RC mode if no failsafe is overriding */
            if (failsafe == FAILSAFE_NONE || failsafe == FAILSAFE_BATTERY_LOW) {
                if (rc_mode != flight_mode) {
                    /* Validate GPS modes */
                    if (rc_mode == FLIGHT_MODE_LOITER && !gps_ok) {
                        /* Silently refuse GPS mode without GPS */
                    } else {
                        ESP_LOGI(TAG, "Flight mode %d -> %d via RC switch", flight_mode, rc_mode);
                        flight_mode = rc_mode;
                    }
                }
            }
        }

        /* ---- 6b. GCS link timeout detection ---- */
        if (rpc != NULL && last_gcs_cmd_ts > 0) {
            uint64_t gcs_timeout_us = (uint64_t)(param_get(PARAM_GCS_TIMEOUT_MS) * 1000.0f);
            bool gcs_ok = (now_us - last_gcs_cmd_ts) < gcs_timeout_us;
            if (!gcs_ok && failsafe != FAILSAFE_GCS_LOST &&
                arm_state == ARM_STATE_ARMED) {
                failsafe = FAILSAFE_GCS_LOST;
                ESP_LOGW(TAG, "GCS link lost (%.1fs timeout)",
                         param_get(PARAM_GCS_TIMEOUT_MS) / 1000.0f);
                send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "GCS link lost");
            }
        }

        /* ---- 7. Check RPC commands ---- */
        if (rpc != NULL) {
            rpc_command_msg_t cmd;
            while (rpc_receive_command(rpc, &cmd, 0) == 0) {
                last_gcs_cmd_ts = now_us; /* track GCS heartbeat */

                switch (cmd.msg_type) {
                case RPC_CMD_ARM:
                    if (cmd.data.arm_cmd.arm) {
                        /* Safety checks for arming */
                        float rc_coll = last_rc.channels[2] * 2.0f - 1.0f;
                        float coll_arm_max = param_get(PARAM_RC_COLL_ARM_MAX);

                        if (!imu_ok) {
                            ESP_LOGW(TAG, "ARM rejected: IMU not healthy");
                            send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "ARM fail: no IMU");
                        } else if (!rc_ok) {
                            ESP_LOGW(TAG, "ARM rejected: no RC signal");
                            send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "ARM fail: no RC");
                        } else if (failsafe != FAILSAFE_NONE &&
                                   failsafe != FAILSAFE_BATTERY_LOW) {
                            ESP_LOGW(TAG, "ARM rejected: failsafe active (%d)", failsafe);
                            send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "ARM fail: failsafe");
                        } else if (rc_coll > coll_arm_max) {
                            ESP_LOGW(TAG, "ARM rejected: collective too high (%.2f)", rc_coll);
                            send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "ARM fail: coll high");
                        } else {
                            ESP_LOGI(TAG, "ARMED via RPC");
                            arm_state = ARM_STATE_ARMED;
                            send_statustext_rpc(rpc, MAV_SEVERITY_INFO, "Vehicle armed");
                        }
                    }
                    break;

                case RPC_CMD_DISARM:
                    ESP_LOGI(TAG, "DISARMED via RPC");
                    arm_state = ARM_STATE_DISARMED;
                    send_statustext_rpc(rpc, MAV_SEVERITY_INFO, "Vehicle disarmed");
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
                    send_statustext_rpc(rpc, MAV_SEVERITY_INFO, "Params saved to flash");
                    break;

                case RPC_CMD_REBOOT:
                    if (arm_state != ARM_STATE_ARMED) {
                        send_statustext_rpc(rpc, MAV_SEVERITY_WARNING, "Rebooting...");
                        ESP_LOGW(TAG, "REBOOT requested via RPC");
                        vTaskDelay(pdMS_TO_TICKS(500)); /* give time for statustext to send */
                        esp_restart();
                    } else {
                        ESP_LOGW(TAG, "REBOOT rejected: vehicle is armed");
                    }
                    break;

                case RPC_CMD_MISSION_COUNT: {
                    uint16_t count = cmd.data.mission_count_cmd.count;
                    ESP_LOGI(TAG, "MISSION_COUNT: expecting %u items", count);
                    mission_mgr_set_count(count);

                    /* ACK the count to GCS (MAV_MISSION_ACCEPTED = 0) */
                    rpc_telemetry_msg_t ack;
                    memset(&ack, 0, sizeof(ack));
                    ack.msg_type = RPC_MSG_MISSION_ACK;
                    ack.timestamp_ms = now_ms;
                    ack.data.mission_ack.result = 0; /* MAV_MISSION_ACCEPTED */
                    rpc_send_telemetry(rpc, &ack);
                    break;
                }

                case RPC_CMD_MISSION_ITEM: {
                    uint16_t seq = cmd.data.mission_item_cmd.seq;
                    mission_item_t item;
                    item.seq          = seq;
                    item.frame        = cmd.data.mission_item_cmd.frame;
                    item.command      = cmd.data.mission_item_cmd.command;
                    item.autocontinue = cmd.data.mission_item_cmd.autocontinue;
                    item.param1       = cmd.data.mission_item_cmd.param1;
                    item.param2       = cmd.data.mission_item_cmd.param2;
                    item.param3       = cmd.data.mission_item_cmd.param3;
                    item.param4       = cmd.data.mission_item_cmd.param4;
                    item.x            = cmd.data.mission_item_cmd.x;
                    item.y            = cmd.data.mission_item_cmd.y;
                    item.z            = cmd.data.mission_item_cmd.z;
                    mission_mgr_store_item(seq, &item);

                    /* ACK each item */
                    rpc_telemetry_msg_t ack;
                    memset(&ack, 0, sizeof(ack));
                    ack.msg_type = RPC_MSG_MISSION_ACK;
                    ack.timestamp_ms = now_ms;
                    ack.data.mission_ack.result = 0; /* MAV_MISSION_ACCEPTED */
                    rpc_send_telemetry(rpc, &ack);
                    break;
                }

                case RPC_CMD_MISSION_REQUEST_LIST: {
                    ESP_LOGI(TAG, "MISSION_REQUEST_LIST: sending count=%d",
                             mission_mgr_get_count());

                    /* Send mission count */
                    rpc_telemetry_msg_t cnt_msg;
                    memset(&cnt_msg, 0, sizeof(cnt_msg));
                    cnt_msg.msg_type = RPC_MSG_MISSION_COUNT;
                    cnt_msg.timestamp_ms = now_ms;
                    cnt_msg.data.mission_count.count = (uint16_t)mission_mgr_get_count();
                    rpc_send_telemetry(rpc, &cnt_msg);

                    /* Send each stored item */
                    for (int i = 0; i < mission_mgr_get_count(); i++) {
                        const mission_item_t *mi = mission_mgr_get_item((uint16_t)i);
                        if (mi == NULL) continue;

                        rpc_telemetry_msg_t item_msg;
                        memset(&item_msg, 0, sizeof(item_msg));
                        item_msg.msg_type = RPC_MSG_MISSION_ITEM;
                        item_msg.timestamp_ms = now_ms;
                        item_msg.data.mission_item.seq          = mi->seq;
                        item_msg.data.mission_item.frame        = mi->frame;
                        item_msg.data.mission_item.command      = mi->command;
                        item_msg.data.mission_item.current      = (mi->seq == mission_mgr_get_current()) ? 1 : 0;
                        item_msg.data.mission_item.autocontinue = mi->autocontinue;
                        item_msg.data.mission_item.param1       = mi->param1;
                        item_msg.data.mission_item.param2       = mi->param2;
                        item_msg.data.mission_item.param3       = mi->param3;
                        item_msg.data.mission_item.param4       = mi->param4;
                        item_msg.data.mission_item.x            = mi->x;
                        item_msg.data.mission_item.y            = mi->y;
                        item_msg.data.mission_item.z            = mi->z;
                        rpc_send_telemetry(rpc, &item_msg);
                    }
                    break;
                }

                case RPC_CMD_MISSION_CLEAR_ALL:
                    ESP_LOGI(TAG, "MISSION_CLEAR_ALL");
                    mission_mgr_clear();
                    {
                        rpc_telemetry_msg_t ack;
                        memset(&ack, 0, sizeof(ack));
                        ack.msg_type = RPC_MSG_MISSION_ACK;
                        ack.timestamp_ms = now_ms;
                        ack.data.mission_ack.result = 0; /* MAV_MISSION_ACCEPTED */
                        rpc_send_telemetry(rpc, &ack);
                    }
                    send_statustext_rpc(rpc, MAV_SEVERITY_INFO, "Mission cleared");
                    break;

                case RPC_CMD_MISSION_SET_CURRENT: {
                    uint16_t seq = cmd.data.mission_set_current_cmd.seq;
                    ESP_LOGI(TAG, "MISSION_SET_CURRENT: seq=%u", seq);
                    mission_mgr_set_current(seq);

                    /* Confirm the new current item to GCS */
                    rpc_telemetry_msg_t cur_msg;
                    memset(&cur_msg, 0, sizeof(cur_msg));
                    cur_msg.msg_type = RPC_MSG_MISSION_CURRENT;
                    cur_msg.timestamp_ms = now_ms;
                    cur_msg.data.mission_current.seq = mission_mgr_get_current();
                    rpc_send_telemetry(rpc, &cur_msg);
                    break;
                }

                case RPC_CMD_REQUEST_HOME_POSITION: {
                    ESP_LOGI(TAG, "REQUEST_HOME_POSITION");
                    if (home_set) {
                        rpc_telemetry_msg_t hmsg;
                        memset(&hmsg, 0, sizeof(hmsg));
                        hmsg.msg_type = RPC_MSG_HOME_POSITION;
                        hmsg.timestamp_ms = now_ms;
                        hmsg.data.home_position.lat = (int32_t)(home_lat * 1e7);
                        hmsg.data.home_position.lon = (int32_t)(home_lon * 1e7);
                        hmsg.data.home_position.alt = (int32_t)(home_alt_msl * 1000.0f);
                        rpc_send_telemetry(rpc, &hmsg);
                    } else {
                        send_statustext_rpc(rpc, MAV_SEVERITY_WARNING,
                                            "Home position not set");
                    }
                    break;
                }

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

            /* Raw sensor data for HIGHRES_IMU and VIBRATION */
            if (imu_updated) {
                rpc_telem_send_imu_raw(rpc, &imu_data);
            }
            if (mag_updated) {
                rpc_telem_send_mag_raw(rpc, &mag_data);
            }
            if (baro_updated) {
                rpc_telem_send_baro_raw(rpc, &baro_data);
            }

            /* Estimator status (construct from sensor health) */
            {
                uint16_t est_flags = 0;
                if (imu_ok)  est_flags |= (1 << 0) | (1 << 1) | (1 << 2); /* attitude + vel */
                if (gps_ok)  est_flags |= (1 << 3) | (1 << 4) | (1 << 5); /* pos horiz/vert */
                if (baro_ok) est_flags |= (1 << 6); /* AGL */
                if (gps_ok)  est_flags |= (1 << 10) | (1 << 11); /* pred pos */
                float h_acc = gps_ok ? gps_data.hdop * 0.5f : 999.0f;
                float v_acc = baro_ok ? 1.0f : 999.0f;
                rpc_telem_send_estimator(rpc, est_flags,
                    imu_ok ? 1.0f : 0.0f,   /* vel_ratio */
                    gps_ok ? 1.0f : 0.0f,   /* pos_horiz_ratio */
                    baro_ok ? 1.0f : 0.0f,  /* pos_vert_ratio */
                    h_acc, v_acc);
            }

            /* Periodic MISSION_CURRENT when in mission mode */
            if (flight_mode == FLIGHT_MODE_MISSION &&
                (now_ms - last_mission_current_ms) >= MISSION_CURRENT_INTERVAL_MS) {
                last_mission_current_ms = now_ms;
                rpc_telemetry_msg_t mc_msg;
                memset(&mc_msg, 0, sizeof(mc_msg));
                mc_msg.msg_type = RPC_MSG_MISSION_CURRENT;
                mc_msg.timestamp_ms = now_ms;
                mc_msg.data.mission_current.seq = mission_mgr_get_current();
                rpc_send_telemetry(rpc, &mc_msg);
            }
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

    xTaskCreatePinnedToCoreWithCaps(
        sysmon_task,
        "sysmon_agent",
        SYSMON_TASK_STACK,
        (void *)rpc,
        SYSMON_TASK_PRIORITY,
        NULL,
        FC_CORE,
        MALLOC_CAP_SPIRAM
    );
}
