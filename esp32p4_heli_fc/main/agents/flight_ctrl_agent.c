/*
 * Flight Controller Agent Implementation
 *
 * Runs at 500 Hz on Core 0.
 * Subscribes to attitude, local position, RC channels, GPS, and vehicle status.
 * Computes actuator controls based on flight mode:
 *   - MANUAL:    RC inputs pass through directly
 *   - ACRO:      RC -> rate setpoints -> rate controller (no attitude stabilization)
 *   - STABILIZE: RC -> attitude setpoint -> attitude controller -> rate controller
 *   - ALT_HOLD:  same as STABILIZE + altitude controller for collective
 *   - LOITER:    GPS position hold + altitude hold (RC adjusts position)
 *   - RTH:       Autonomous return to home position and altitude
 *   - LAND:      Automated descent and disarm
 *
 * Publishes actuator_controls for the actuator agent.
 */

#include "flight_ctrl_agent.h"

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "../common/board_config.h"
#include "../common/flight_modes.h"
#include "../common/math_utils.h"
#include "../common/param.h"
#include "../uorb/uorb.h"
#include "../uorb/topics/vehicle_attitude.h"
#include "../uorb/topics/vehicle_local_position.h"
#include "../uorb/topics/rc_channels.h"
#include "../uorb/topics/vehicle_status.h"
#include "../uorb/topics/actuator_controls.h"
#include "../uorb/topics/sensor_gps.h"
#include "../uorb/topics/airspeed.h"
#include "../control/attitude_control.h"
#include "../control/rate_control.h"
#include "../control/pos_control.h"

static const char *TAG = "flight_ctrl";

/* Nominal controller timestep (500 Hz) */
#define CTRL_DT_NOMINAL  0.002f

/* Airspeed limiting */
#define AIRSPEED_MAX         20.0f  /* m/s max forward airspeed (~72 km/h) */
#define AIRSPEED_WARN        15.0f  /* m/s start pitch-back to slow down */

/* Landing parameters */
#define LAND_DESCENT_RATE    0.5f   /* m/s descent rate during landing */
#define LAND_DETECT_ALT      0.3f   /* altitude threshold for touchdown (meters) */
#define LAND_DETECT_TIME_MS  2000   /* time at low altitude before disarming */

/* RTH parameters */
#define RTH_ALTITUDE         20.0f  /* return altitude (meters above home) */
#define RTH_APPROACH_RADIUS  5.0f   /* distance at which to start descending (meters) */

/* Simple GPS position error to attitude setpoint conversion */
#define POS_KP              0.5f    /* position error (m) -> attitude angle (rad) */
#define POS_MAX_ANGLE       DEG_TO_RAD(15.0f)  /* max tilt for position control */

/* ── Helper: apply deadzone to RC stick ──────────────────────── */

static float apply_deadzone(float input, float dz)
{
    if (fabsf(input) < dz) return 0.0f;
    /* Re-scale so output is 0..1 immediately outside deadzone */
    float sign = (input > 0.0f) ? 1.0f : -1.0f;
    return sign * (fabsf(input) - dz) / (1.0f - dz);
}

/* ── Helper: stabilized attitude control ─────────────────────── */

static void compute_stabilized_controls(
    attitude_controller_t *att_ctrl,
    rate_controller_t *rate_ctrl,
    const vehicle_attitude_t *att,
    const float att_sp[3],
    float *yaw_sp,
    bool *yaw_sp_initialized,
    float rc_yaw,
    float ctrl_dt,
    actuator_controls_t *act)
{
    if (!*yaw_sp_initialized) {
        *yaw_sp = att->yaw;
        *yaw_sp_initialized = true;
    }
    *yaw_sp += rc_yaw * DEG_TO_RAD(param_get(PARAM_MAX_YAW_RATE_DEG)) * ctrl_dt;

    float sp[3] = {att_sp[0], att_sp[1], *yaw_sp};
    float meas[3] = {att->roll, att->pitch, att->yaw};
    float rate_sp[3];
    attitude_control_update(att_ctrl, sp, meas, rate_sp);

    float rate_meas[3] = {att->rollspeed, att->pitchspeed, att->yawspeed};
    float ctrl_out[3];
    rate_control_update(rate_ctrl, rate_sp, rate_meas, ctrl_out);

    act->roll  = ctrl_out[0];
    act->pitch = ctrl_out[1];
    act->yaw   = ctrl_out[2];
}

/* ── Helper: approximate flat-earth distance between GPS coords ─ */

static void gps_distance_ne(double lat1, double lon1, double lat2, double lon2,
                             float *north_m, float *east_m)
{
    /* Approximate meters per degree at given latitude */
    double lat_rad = lat1 * M_PI / 180.0;
    double m_per_deg_lat = 111132.0;
    double m_per_deg_lon = 111132.0 * cos(lat_rad);

    *north_m = (float)((lat2 - lat1) * m_per_deg_lat);
    *east_m  = (float)((lon2 - lon1) * m_per_deg_lon);
}

/* ------------------------------------------------------------------ */
static void flight_ctrl_task(void *param)
{
    (void)param;

    /* Wait for topics to be advertised by sensor agent */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* ---- Subscribe to topics ---- */
    orb_subscription_t *att_sub  = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE);
    orb_subscription_t *pos_sub  = orb_subscribe(ORB_ID_VEHICLE_LOCAL_POSITION);
    orb_subscription_t *rc_sub   = orb_subscribe(ORB_ID_RC_CHANNELS);
    orb_subscription_t *stat_sub = orb_subscribe(ORB_ID_VEHICLE_STATUS);
    orb_subscription_t *gps_sub  = orb_subscribe(ORB_ID_SENSOR_GPS);
    orb_subscription_t *as_sub   = orb_subscribe(ORB_ID_AIRSPEED);

    /* Advertise output topic */
    orb_advertise(ORB_ID_ACTUATOR_CONTROLS, sizeof(actuator_controls_t));

    /* ---- Initialize controllers ---- */
    attitude_controller_t att_ctrl;
    attitude_control_init(&att_ctrl);

    rate_controller_t rate_ctrl;
    rate_control_init(&rate_ctrl, CTRL_DT_NOMINAL);

    pos_controller_t pos_ctrl;
    pos_control_init(&pos_ctrl, CTRL_DT_NOMINAL);

    /* Local state */
    vehicle_attitude_t       att     = {0};
    vehicle_local_position_t pos     = {0};
    rc_channels_t            rc      = {0};
    vehicle_status_t         status  = {0};
    sensor_gps_t             gps     = {0};
    airspeed_t               arspd   = {0};

    /* Persistent flight mode state */
    float alt_hold_sp = 0.0f;
    float yaw_sp = 0.0f;
    bool yaw_sp_initialized = false;
    flight_mode_t prev_mode = FLIGHT_MODE_MANUAL;

    /* Home position (captured on first arm with GPS fix) */
    double home_lat = 0.0, home_lon = 0.0;
    float home_alt = 0.0f;
    bool home_set = false;

    /* Loiter hold position */
    double loiter_lat = 0.0, loiter_lon = 0.0;

    /* Landing state */
    uint32_t land_low_alt_start_ms = 0;
    bool land_detected = false;

    /* RTH state machine */
    enum { RTH_CLIMB, RTH_TRANSIT, RTH_DESCEND, RTH_LAND } rth_phase = RTH_CLIMB;

    /* ---- Main loop at 500 Hz ---- */
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t prev_us = (uint64_t)esp_timer_get_time();

    while (1) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        uint32_t now_ms = (uint32_t)(now_us / 1000);
        float ctrl_dt = (float)(now_us - prev_us) * 1.0e-6f;
        if (ctrl_dt <= 0.0f || ctrl_dt > 0.02f) ctrl_dt = CTRL_DT_NOMINAL;
        prev_us = now_us;

        /* ---- Read latest data from subscriptions ---- */
        orb_copy(att_sub, &att);
        orb_copy(pos_sub, &pos);
        orb_copy(rc_sub, &rc);
        orb_copy(stat_sub, &status);
        orb_copy(gps_sub, &gps);
        orb_copy(as_sub, &arspd);

        /* ---- Capture home on first arm with GPS 3D fix ---- */
        if (status.arm_state == ARM_STATE_ARMED && !home_set &&
            gps.fix_type >= 3 && gps.satellites >= 6) {
            home_lat = gps.latitude;
            home_lon = gps.longitude;
            home_alt = pos.alt;
            home_set = true;
            ESP_LOGI(TAG, "HOME set: lat=%.7f lon=%.7f alt=%.1f",
                     home_lat, home_lon, home_alt);
        }

        /* ---- Prepare actuator output ---- */
        actuator_controls_t act = {0};
        act.timestamp_us = now_us;

        /* If disarmed, output zero controls */
        if (status.arm_state != ARM_STATE_ARMED) {
            act.roll = 0.0f;
            act.pitch = 0.0f;
            act.yaw = 0.0f;
            act.collective = -1.0f;
            act.throttle = 0.0f;
            orb_publish(ORB_ID_ACTUATOR_CONTROLS, &act);
            rate_control_reset(&rate_ctrl);
            pos_control_reset(&pos_ctrl);
            yaw_sp_initialized = false;
            land_detected = false;
            prev_mode = status.flight_mode;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
            continue;
        }

        /* RC channel mapping with deadzone */
        float dz = param_get(PARAM_RC_DEADZONE);
        float rc_roll  = apply_deadzone(rc.channels[0], dz);
        float rc_pitch = apply_deadzone(rc.channels[1], dz);
        float rc_coll  = rc.channels[2] * 2.0f - 1.0f;
        float rc_yaw   = apply_deadzone(rc.channels[3], dz);

        /* ---- Airspeed limiting: reduce forward pitch when overspeed ---- */
        float airspeed_pitch_limit = 1.0f;  /* 1.0 = no limit */
        if (arspd.valid && arspd.indicated_airspeed > AIRSPEED_WARN) {
            /* Linear ramp: at WARN → 1.0, at MAX → 0.0 */
            airspeed_pitch_limit = 1.0f - (arspd.indicated_airspeed - AIRSPEED_WARN) /
                                           (AIRSPEED_MAX - AIRSPEED_WARN);
            if (airspeed_pitch_limit < 0.0f) airspeed_pitch_limit = 0.0f;
            /* Only limit nose-down pitch (positive pitch = forward flight) */
            if (rc_pitch > 0.0f) {
                rc_pitch *= airspeed_pitch_limit;
            }
        }

        switch (status.flight_mode) {
        case FLIGHT_MODE_MANUAL:
            act.roll  = constrain_f(rc_roll, -1.0f, 1.0f);
            act.pitch = constrain_f(rc_pitch, -1.0f, 1.0f);
            act.yaw   = constrain_f(rc_yaw, -1.0f, 1.0f);
            act.collective = constrain_f(rc_coll, -1.0f, 1.0f);
            act.throttle   = constrain_f(rc.channels[2], 0.0f, 1.0f);
            break;

        case FLIGHT_MODE_ACRO: {
            /* RC sticks map directly to rate setpoints */
            float rate_sp[3];
            rate_sp[0] = rc_roll  * DEG_TO_RAD(param_get(PARAM_MAX_ROLL_RATE_DEG));
            rate_sp[1] = rc_pitch * DEG_TO_RAD(param_get(PARAM_MAX_PITCH_RATE_DEG));
            rate_sp[2] = rc_yaw   * DEG_TO_RAD(param_get(PARAM_MAX_YAW_RATE_DEG));

            float rate_meas[3] = {att.rollspeed, att.pitchspeed, att.yawspeed};
            float ctrl_out[3];
            rate_control_update(&rate_ctrl, rate_sp, rate_meas, ctrl_out);

            act.roll  = ctrl_out[0];
            act.pitch = ctrl_out[1];
            act.yaw   = ctrl_out[2];
            act.collective = constrain_f(rc_coll, -1.0f, 1.0f);
            act.throttle   = constrain_f(rc.channels[2], 0.0f, 1.0f);
            break;
        }

        case FLIGHT_MODE_STABILIZE: {
            float att_sp[3];
            att_sp[0] = rc_roll  * DEG_TO_RAD(param_get(PARAM_MAX_ROLL_DEG));
            att_sp[1] = rc_pitch * DEG_TO_RAD(param_get(PARAM_MAX_PITCH_DEG));
            att_sp[2] = 0.0f; /* placeholder, yaw_sp managed inside */

            compute_stabilized_controls(&att_ctrl, &rate_ctrl, &att,
                att_sp, &yaw_sp, &yaw_sp_initialized, rc_yaw, ctrl_dt, &act);
            act.collective = constrain_f(rc_coll, -1.0f, 1.0f);
            act.throttle   = constrain_f(rc.channels[2], 0.0f, 1.0f);
            break;
        }

        case FLIGHT_MODE_ALT_HOLD: {
            if (prev_mode != FLIGHT_MODE_ALT_HOLD) {
                alt_hold_sp = pos.alt;
                pos_control_reset(&pos_ctrl);
            }

            float coll_deadzone = 0.1f;
            if (fabsf(rc_coll) > coll_deadzone) {
                alt_hold_sp += rc_coll * param_get(PARAM_MAX_CLIMB_RATE) * ctrl_dt;
            }

            float collective_out;
            pos_control_update_altitude(&pos_ctrl, alt_hold_sp, pos.alt,
                                        pos.climb_rate, &collective_out);

            float att_sp[3];
            att_sp[0] = rc_roll  * DEG_TO_RAD(param_get(PARAM_MAX_ROLL_DEG));
            att_sp[1] = rc_pitch * DEG_TO_RAD(param_get(PARAM_MAX_PITCH_DEG));
            att_sp[2] = 0.0f;

            compute_stabilized_controls(&att_ctrl, &rate_ctrl, &att,
                att_sp, &yaw_sp, &yaw_sp_initialized, rc_yaw, ctrl_dt, &act);
            act.collective = constrain_f(collective_out, -1.0f, 1.0f);
            act.throttle = constrain_f((collective_out + 1.0f) * 0.5f, 0.0f, 1.0f);
            break;
        }

        case FLIGHT_MODE_LOITER: {
            /* Capture loiter position on entry */
            if (prev_mode != FLIGHT_MODE_LOITER) {
                loiter_lat = gps.latitude;
                loiter_lon = gps.longitude;
                alt_hold_sp = pos.alt;
                pos_control_reset(&pos_ctrl);
                ESP_LOGI(TAG, "LOITER: hold lat=%.7f lon=%.7f alt=%.1f",
                         loiter_lat, loiter_lon, alt_hold_sp);
            }

            /* RC adjusts loiter position (velocity-like control) */
            float coll_deadzone = 0.1f;
            if (fabsf(rc_coll) > coll_deadzone) {
                alt_hold_sp += rc_coll * param_get(PARAM_MAX_CLIMB_RATE) * ctrl_dt;
            }

            /* Altitude control */
            float collective_out;
            pos_control_update_altitude(&pos_ctrl, alt_hold_sp, pos.alt,
                                        pos.climb_rate, &collective_out);

            /* Position control: GPS error -> attitude setpoints */
            float att_sp[3] = {0.0f, 0.0f, 0.0f};
            if (gps.fix_type >= 3) {
                float north_err, east_err;
                gps_distance_ne(gps.latitude, gps.longitude,
                               loiter_lat, loiter_lon,
                               &north_err, &east_err);

                /* Rotate error from NED to body frame using yaw */
                float cos_yaw = cosf(att.yaw);
                float sin_yaw = sinf(att.yaw);
                float body_fwd = cos_yaw * north_err + sin_yaw * east_err;
                float body_right = -sin_yaw * north_err + cos_yaw * east_err;

                /* Position error -> pitch/roll attitude setpoints */
                att_sp[1] = constrain_f(-body_fwd * POS_KP, -POS_MAX_ANGLE, POS_MAX_ANGLE);
                att_sp[0] = constrain_f(body_right * POS_KP, -POS_MAX_ANGLE, POS_MAX_ANGLE);
            }

            /* Add RC stick for manual override */
            att_sp[0] += rc_roll  * DEG_TO_RAD(10.0f);
            att_sp[1] += rc_pitch * DEG_TO_RAD(10.0f);

            compute_stabilized_controls(&att_ctrl, &rate_ctrl, &att,
                att_sp, &yaw_sp, &yaw_sp_initialized, rc_yaw, ctrl_dt, &act);
            act.collective = constrain_f(collective_out, -1.0f, 1.0f);
            act.throttle = constrain_f((collective_out + 1.0f) * 0.5f, 0.0f, 1.0f);
            break;
        }

        case FLIGHT_MODE_RTH: {
            if (!home_set || gps.fix_type < 3) {
                /* No home or no GPS: fall back to alt hold with level attitude */
                if (prev_mode != FLIGHT_MODE_RTH) {
                    alt_hold_sp = pos.alt;
                    pos_control_reset(&pos_ctrl);
                    ESP_LOGW(TAG, "RTH: no home/GPS, holding altitude");
                }
                float collective_out;
                pos_control_update_altitude(&pos_ctrl, alt_hold_sp, pos.alt,
                                            pos.climb_rate, &collective_out);
                float att_sp[3] = {0.0f, 0.0f, 0.0f};
                compute_stabilized_controls(&att_ctrl, &rate_ctrl, &att,
                    att_sp, &yaw_sp, &yaw_sp_initialized, 0.0f, ctrl_dt, &act);
                act.collective = constrain_f(collective_out, -1.0f, 1.0f);
                act.throttle = constrain_f((collective_out + 1.0f) * 0.5f, 0.0f, 1.0f);
                break;
            }

            /* RTH state machine */
            if (prev_mode != FLIGHT_MODE_RTH) {
                rth_phase = RTH_CLIMB;
                pos_control_reset(&pos_ctrl);
                ESP_LOGI(TAG, "RTH: starting return to home");
            }

            float north_err, east_err;
            gps_distance_ne(gps.latitude, gps.longitude,
                           home_lat, home_lon,
                           &north_err, &east_err);
            float dist_to_home = sqrtf(north_err*north_err + east_err*east_err);

            switch (rth_phase) {
            case RTH_CLIMB:
                alt_hold_sp = home_alt + RTH_ALTITUDE;
                if (pos.alt >= alt_hold_sp - 1.0f) {
                    rth_phase = RTH_TRANSIT;
                    ESP_LOGI(TAG, "RTH: altitude reached, transiting to home");
                }
                break;
            case RTH_TRANSIT:
                alt_hold_sp = home_alt + RTH_ALTITUDE;
                if (dist_to_home < RTH_APPROACH_RADIUS) {
                    rth_phase = RTH_DESCEND;
                    ESP_LOGI(TAG, "RTH: arrived at home, descending");
                }
                break;
            case RTH_DESCEND:
                alt_hold_sp -= LAND_DESCENT_RATE * ctrl_dt;
                if (pos.alt < home_alt + 2.0f) {
                    rth_phase = RTH_LAND;
                    ESP_LOGI(TAG, "RTH: near ground, landing");
                }
                break;
            case RTH_LAND:
                alt_hold_sp -= LAND_DESCENT_RATE * ctrl_dt;
                break;
            }

            /* Altitude control */
            float collective_out;
            pos_control_update_altitude(&pos_ctrl, alt_hold_sp, pos.alt,
                                        pos.climb_rate, &collective_out);

            /* Navigate toward home */
            float att_sp[3] = {0.0f, 0.0f, 0.0f};
            if (rth_phase == RTH_TRANSIT || rth_phase == RTH_DESCEND) {
                float cos_yaw = cosf(att.yaw);
                float sin_yaw = sinf(att.yaw);
                float body_fwd = cos_yaw * north_err + sin_yaw * east_err;
                float body_right = -sin_yaw * north_err + cos_yaw * east_err;

                att_sp[1] = constrain_f(-body_fwd * POS_KP, -POS_MAX_ANGLE, POS_MAX_ANGLE);
                att_sp[0] = constrain_f(body_right * POS_KP, -POS_MAX_ANGLE, POS_MAX_ANGLE);
            }

            compute_stabilized_controls(&att_ctrl, &rate_ctrl, &att,
                att_sp, &yaw_sp, &yaw_sp_initialized, 0.0f, ctrl_dt, &act);
            act.collective = constrain_f(collective_out, -1.0f, 1.0f);
            act.throttle = constrain_f((collective_out + 1.0f) * 0.5f, 0.0f, 1.0f);
            break;
        }

        case FLIGHT_MODE_LAND: {
            if (prev_mode != FLIGHT_MODE_LAND) {
                alt_hold_sp = pos.alt;
                pos_control_reset(&pos_ctrl);
                land_detected = false;
                land_low_alt_start_ms = 0;
                ESP_LOGI(TAG, "LAND: starting descent from %.1fm", pos.alt);
            }

            /* Constant descent rate */
            alt_hold_sp -= LAND_DESCENT_RATE * ctrl_dt;

            /* Altitude control */
            float collective_out;
            pos_control_update_altitude(&pos_ctrl, alt_hold_sp, pos.alt,
                                        pos.climb_rate, &collective_out);

            /* Level attitude during landing */
            float att_sp[3] = {0.0f, 0.0f, 0.0f};
            compute_stabilized_controls(&att_ctrl, &rate_ctrl, &att,
                att_sp, &yaw_sp, &yaw_sp_initialized, 0.0f, ctrl_dt, &act);

            /* Touchdown detection */
            if (pos.alt < LAND_DETECT_ALT) {
                if (land_low_alt_start_ms == 0) {
                    land_low_alt_start_ms = now_ms;
                }
                if ((now_ms - land_low_alt_start_ms) > LAND_DETECT_TIME_MS) {
                    land_detected = true;
                    act.collective = -1.0f;
                    act.throttle = 0.0f;
                    ESP_LOGI(TAG, "LAND: touchdown detected, disarming");
                    /* Sysmon will detect land_detected state or we can just set min collective */
                }
            } else {
                land_low_alt_start_ms = 0;
            }

            if (!land_detected) {
                act.collective = constrain_f(collective_out, -1.0f, 1.0f);
                act.throttle = constrain_f((collective_out + 1.0f) * 0.5f, 0.0f, 1.0f);
            }
            break;
        }

        default:
            act.roll = 0.0f;
            act.pitch = 0.0f;
            act.yaw = 0.0f;
            act.collective = -1.0f;
            act.throttle = 0.0f;
            break;
        }

        prev_mode = status.flight_mode;
        orb_publish(ORB_ID_ACTUATOR_CONTROLS, &act);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
    }
}

void flight_ctrl_agent_start(void)
{
    ESP_LOGI(TAG, "Starting flight controller agent on Core %d, priority %d",
             FC_CORE, FLIGHT_CTRL_PRIORITY);

    xTaskCreatePinnedToCore(
        flight_ctrl_task,
        "flight_ctrl",
        FLIGHT_CTRL_STACK,
        NULL,
        FLIGHT_CTRL_PRIORITY,
        NULL,
        FC_CORE
    );
}
