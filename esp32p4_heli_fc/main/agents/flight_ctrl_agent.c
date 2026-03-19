/*
 * Flight Controller Agent Implementation
 *
 * Runs at 500 Hz on Core 0.
 * Subscribes to attitude, local position, RC channels, and vehicle status.
 * Computes actuator controls based on flight mode:
 *   - MANUAL: RC inputs pass through directly
 *   - STABILIZE: RC -> attitude setpoint -> attitude controller -> rate controller
 *   - ALT_HOLD: same as STABILIZE + altitude controller for collective
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
#include "../uorb/uorb.h"
#include "../uorb/topics/vehicle_attitude.h"
#include "../uorb/topics/vehicle_local_position.h"
#include "../uorb/topics/rc_channels.h"
#include "../uorb/topics/vehicle_status.h"
#include "../uorb/topics/actuator_controls.h"
#include "../control/attitude_control.h"
#include "../control/rate_control.h"
#include "../control/pos_control.h"

static const char *TAG = "flight_ctrl";

/* Maximum attitude angle from RC input (radians) */
#define MAX_ROLL_ANGLE   DEG_TO_RAD(45.0f)
#define MAX_PITCH_ANGLE  DEG_TO_RAD(45.0f)
#define MAX_YAW_RATE     DEG_TO_RAD(180.0f)  /* rad/s */

/* Controller timestep (500 Hz) */
#define CTRL_DT  0.002f

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

    /* Advertise output topic */
    orb_advertise(ORB_ID_ACTUATOR_CONTROLS, sizeof(actuator_controls_t));

    /* ---- Initialize controllers ---- */
    attitude_controller_t att_ctrl;
    attitude_control_init(&att_ctrl);

    rate_controller_t rate_ctrl;
    rate_control_init(&rate_ctrl, CTRL_DT);

    pos_controller_t pos_ctrl;
    pos_control_init(&pos_ctrl, CTRL_DT);

    /* Local state */
    vehicle_attitude_t       att     = {0};
    vehicle_local_position_t pos     = {0};
    rc_channels_t            rc      = {0};
    vehicle_status_t         status  = {0};

    /* Altitude hold setpoint, captured when entering ALT_HOLD */
    float alt_hold_sp = 0.0f;
    flight_mode_t prev_mode = FLIGHT_MODE_MANUAL;

    /* ---- Main loop at 500 Hz ---- */
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();

        /* ---- Read latest data from subscriptions ---- */
        orb_copy(att_sub, &att);
        orb_copy(pos_sub, &pos);
        orb_copy(rc_sub, &rc);
        orb_copy(stat_sub, &status);

        /* ---- Prepare actuator output ---- */
        actuator_controls_t act = {0};
        act.timestamp_us = now_us;

        /* If disarmed, output zero controls */
        if (status.arm_state != ARM_STATE_ARMED) {
            act.roll = 0.0f;
            act.pitch = 0.0f;
            act.yaw = 0.0f;
            act.collective = -1.0f;  /* minimum collective */
            act.throttle = 0.0f;
            orb_publish(ORB_ID_ACTUATOR_CONTROLS, &act);
            /* Reset controllers when disarmed */
            rate_control_reset(&rate_ctrl);
            pos_control_reset(&pos_ctrl);
            prev_mode = status.flight_mode;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
            continue;
        }

        /* RC channel mapping:
         * ch[0] = roll      (-1 to 1)
         * ch[1] = pitch     (-1 to 1)
         * ch[2] = throttle/collective (0 to 1, mapped to -1..1 for collective)
         * ch[3] = yaw       (-1 to 1)
         */
        float rc_roll  = rc.channels[0];
        float rc_pitch = rc.channels[1];
        float rc_coll  = rc.channels[2] * 2.0f - 1.0f; /* 0..1 -> -1..1 */
        float rc_yaw   = rc.channels[3];

        switch (status.flight_mode) {
        case FLIGHT_MODE_MANUAL:
            /* Direct RC passthrough to actuator controls */
            act.roll  = constrain_f(rc_roll, -1.0f, 1.0f);
            act.pitch = constrain_f(rc_pitch, -1.0f, 1.0f);
            act.yaw   = constrain_f(rc_yaw, -1.0f, 1.0f);
            act.collective = constrain_f(rc_coll, -1.0f, 1.0f);
            act.throttle   = constrain_f(rc.channels[2], 0.0f, 1.0f);
            break;

        case FLIGHT_MODE_STABILIZE: {
            /* RC sticks -> attitude setpoints -> attitude ctrl -> rate ctrl */
            float att_sp[3];
            att_sp[0] = rc_roll  * MAX_ROLL_ANGLE;    /* roll angle setpoint */
            att_sp[1] = rc_pitch * MAX_PITCH_ANGLE;   /* pitch angle setpoint */
            att_sp[2] = att.yaw + rc_yaw * MAX_YAW_RATE * CTRL_DT; /* yaw rate-like */

            float att_meas[3] = {att.roll, att.pitch, att.yaw};

            /* Attitude controller -> rate setpoints */
            float rate_sp[3];
            attitude_control_update(&att_ctrl, att_sp, att_meas, rate_sp);

            /* Rate controller -> actuator outputs */
            float rate_meas[3] = {att.rollspeed, att.pitchspeed, att.yawspeed};
            float ctrl_out[3];
            rate_control_update(&rate_ctrl, rate_sp, rate_meas, ctrl_out);

            act.roll  = ctrl_out[0];
            act.pitch = ctrl_out[1];
            act.yaw   = ctrl_out[2];
            /* Collective and throttle from RC directly in stabilize mode */
            act.collective = constrain_f(rc_coll, -1.0f, 1.0f);
            act.throttle   = constrain_f(rc.channels[2], 0.0f, 1.0f);
            break;
        }

        case FLIGHT_MODE_ALT_HOLD: {
            /* Capture altitude setpoint on mode transition */
            if (prev_mode != FLIGHT_MODE_ALT_HOLD) {
                alt_hold_sp = pos.alt;
                pos_control_reset(&pos_ctrl);
            }

            /* If RC collective is near center (dead zone), hold altitude.
             * Otherwise, adjust altitude setpoint by RC input. */
            float coll_deadzone = 0.1f;
            if (fabsf(rc_coll) > coll_deadzone) {
                /* Modify altitude setpoint based on stick displacement */
                float climb_cmd = rc_coll * 2.0f; /* m/s max climb rate from stick */
                alt_hold_sp += climb_cmd * CTRL_DT;
            }

            /* Altitude controller -> collective */
            float collective_out;
            pos_control_update_altitude(&pos_ctrl, alt_hold_sp, pos.alt,
                                        pos.climb_rate, &collective_out);

            /* Attitude control (same as STABILIZE for roll/pitch/yaw) */
            float att_sp[3];
            att_sp[0] = rc_roll  * MAX_ROLL_ANGLE;
            att_sp[1] = rc_pitch * MAX_PITCH_ANGLE;
            att_sp[2] = att.yaw + rc_yaw * MAX_YAW_RATE * CTRL_DT;

            float att_meas[3] = {att.roll, att.pitch, att.yaw};
            float rate_sp[3];
            attitude_control_update(&att_ctrl, att_sp, att_meas, rate_sp);

            float rate_meas[3] = {att.rollspeed, att.pitchspeed, att.yawspeed};
            float ctrl_out[3];
            rate_control_update(&rate_ctrl, rate_sp, rate_meas, ctrl_out);

            act.roll  = ctrl_out[0];
            act.pitch = ctrl_out[1];
            act.yaw   = ctrl_out[2];
            act.collective = constrain_f(collective_out, -1.0f, 1.0f);
            /* Throttle derived from collective position via mixer throttle curve */
            act.throttle = constrain_f((collective_out + 1.0f) * 0.5f, 0.0f, 1.0f);
            break;
        }

        default:
            /* Unknown mode: safe defaults */
            act.roll = 0.0f;
            act.pitch = 0.0f;
            act.yaw = 0.0f;
            act.collective = -1.0f;
            act.throttle = 0.0f;
            break;
        }

        prev_mode = status.flight_mode;

        /* Publish actuator controls */
        orb_publish(ORB_ID_ACTUATOR_CONTROLS, &act);

        /* Wait for next 2 ms cycle */
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
