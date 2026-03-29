/*
 * MAVLink Protocol Handler - Implementation
 *
 * Bridges between RPC inter-core messages and MAVLink protocol.
 * Runs as a FreeRTOS task pinned to Core 1.
 *
 * Outbound: receives RPC telemetry -> encodes MAVLink -> sends via GCS bridge
 * Inbound:  receives MAVLink from GCS bridge -> decodes -> sends RPC commands
 *
 * Telemetry rates (configurable):
 *   Heartbeat:    1 Hz
 *   Attitude:    10 Hz
 *   GPS:          5 Hz
 *   Battery:      2 Hz
 *   System status: 1 Hz
 *   VFR HUD:      2 Hz
 */
#include "mavlink_handler.h"
#include "gcs_bridge.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/mavlink_msg.h"
#include "../rpc/rpc_messages.h"
#include "../common/flight_modes.h"
#include "../agents/mission_mgr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <math.h>

static const char *TAG = "mavlink_handler";

/* ── Internal state ───────────────────────────────────────────── */

static rpc_context_t *s_rpc_ctx = NULL;
static mavlink_handler_config_t s_config;
static mavlink_parser_t s_parser;

/* Cached telemetry from RPC (latest values) */
static rpc_telemetry_msg_t s_latest_attitude;
static rpc_telemetry_msg_t s_latest_gps;
static rpc_telemetry_msg_t s_latest_altitude;
static rpc_telemetry_msg_t s_latest_battery;
static rpc_telemetry_msg_t s_latest_status;
static rpc_telemetry_msg_t s_latest_rc;
static rpc_telemetry_msg_t s_latest_servo;

/* Param value queue (RPC_MSG_PARAM_VALUE from Core 0) */
#define PARAM_VALUE_QUEUE_DEPTH 16
static rpc_telemetry_msg_t s_param_value_queue[PARAM_VALUE_QUEUE_DEPTH];
static int s_param_value_count = 0;

/* Statustext queue (RPC_MSG_STATUSTEXT from Core 0) */
#define STATUSTEXT_QUEUE_DEPTH 8
static rpc_telemetry_msg_t s_statustext_queue[STATUSTEXT_QUEUE_DEPTH];
static int s_statustext_count = 0;

static bool s_has_attitude  = false;
static bool s_has_gps       = false;
static bool s_has_altitude  = false;
static bool s_has_battery   = false;
static bool s_has_status    = false;
static bool s_has_rc        = false;
static bool s_has_servo     = false;

/* Home position cache */
static bool s_has_home = false;
static int32_t s_home_lat = 0;
static int32_t s_home_lon = 0;
static int32_t s_home_alt = 0;

/* Mission protocol state */
typedef enum {
    MISSION_STATE_IDLE = 0,
    MISSION_STATE_UPLOADING,
    MISSION_STATE_DOWNLOADING,
} mission_state_t;

static mavlink_mission_item_int_t s_mission_items[MISSION_MAX_ITEMS]; /* from mission_mgr.h */
static uint16_t s_mission_count = 0;
static uint16_t s_mission_expected = 0;
static uint16_t s_mission_received = 0;
static uint16_t s_mission_current_seq = 0;
static mission_state_t s_mission_state = MISSION_STATE_IDLE;
static uint8_t s_mission_gcs_sysid = MAV_COMP_ID_GCS;
static uint8_t s_mission_gcs_compid = 0;

/* Timing for rate-limited sends */
static uint32_t s_last_heartbeat_ms = 0;
static uint32_t s_last_attitude_ms  = 0;
static uint32_t s_last_gps_ms       = 0;
static uint32_t s_last_battery_ms   = 0;
static uint32_t s_last_vfr_hud_ms   = 0;
static uint32_t s_last_servo_ms     = 0;
static uint32_t s_last_ext_state_ms = 0;
static uint32_t s_last_home_ms      = 0;

/* ── Helpers ──────────────────────────────────────────────────── */

static uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static uint32_t rate_to_interval_ms(uint8_t hz)
{
    if (hz == 0) return UINT32_MAX;
    return 1000 / hz;
}

static bool rate_check(uint32_t *last_ms, uint8_t hz)
{
    uint32_t now = get_time_ms();
    uint32_t interval = rate_to_interval_ms(hz);
    if ((now - *last_ms) >= interval) {
        *last_ms = now;
        return true;
    }
    return false;
}

/* Forward declarations */
static void send_mission_current(void);
static void send_autopilot_version(void);

static void send_mavlink_msg(mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_serialize(msg, buf, sizeof(buf));
    if (len > 0) {
        gcs_bridge_send(buf, (size_t)len);
    }
}

/* ── Drain all pending RPC telemetry into latest cache ────────── */

static void drain_rpc_telemetry(void)
{
    rpc_telemetry_msg_t telem;

    /* Drain all available telemetry messages (non-blocking) */
    while (rpc_receive_telemetry(s_rpc_ctx, &telem, 0) == 0) {
        switch (telem.msg_type) {
        case RPC_MSG_ATTITUDE:
            s_latest_attitude = telem;
            s_has_attitude = true;
            break;
        case RPC_MSG_GPS:
            s_latest_gps = telem;
            s_has_gps = true;
            break;
        case RPC_MSG_ALTITUDE:
            s_latest_altitude = telem;
            s_has_altitude = true;
            break;
        case RPC_MSG_BATTERY:
            s_latest_battery = telem;
            s_has_battery = true;
            break;
        case RPC_MSG_STATUS:
            s_latest_status = telem;
            s_has_status = true;
            break;
        case RPC_MSG_RC_CHANNELS:
            s_latest_rc = telem;
            s_has_rc = true;
            break;
        case RPC_MSG_PARAM_VALUE:
            if (s_param_value_count < PARAM_VALUE_QUEUE_DEPTH) {
                s_param_value_queue[s_param_value_count++] = telem;
            }
            break;
        case RPC_MSG_SERVO_OUTPUT:
            s_latest_servo = telem;
            s_has_servo = true;
            break;
        case RPC_MSG_STATUSTEXT:
            if (s_statustext_count < STATUSTEXT_QUEUE_DEPTH) {
                s_statustext_queue[s_statustext_count++] = telem;
            }
            break;
        case RPC_MSG_HOME_POSITION:
            s_home_lat = telem.data.home_position.lat;
            s_home_lon = telem.data.home_position.lon;
            s_home_alt = telem.data.home_position.alt;
            s_has_home = true;
            break;
        case RPC_MSG_MISSION_COUNT: {
            /* Core 0 is sending mission count for GCS download */
            mavlink_message_t msg;
            mavlink_msg_mission_count_encode(&msg, s_mission_gcs_sysid,
                s_mission_gcs_compid, telem.data.mission_count.count,
                MAV_MISSION_TYPE_MISSION);
            send_mavlink_msg(&msg);
            break;
        }
        case RPC_MSG_MISSION_ITEM: {
            /* Core 0 sends mission item for GCS download */
            mavlink_mission_item_int_t item;
            memset(&item, 0, sizeof(item));
            item.seq = telem.data.mission_item.seq;
            item.frame = telem.data.mission_item.frame;
            item.command = telem.data.mission_item.command;
            item.current = telem.data.mission_item.current;
            item.autocontinue = telem.data.mission_item.autocontinue;
            item.param1 = telem.data.mission_item.param1;
            item.param2 = telem.data.mission_item.param2;
            item.param3 = telem.data.mission_item.param3;
            item.param4 = telem.data.mission_item.param4;
            item.x = telem.data.mission_item.x;
            item.y = telem.data.mission_item.y;
            item.z = telem.data.mission_item.z;
            mavlink_message_t msg;
            mavlink_msg_mission_item_int_encode(&msg, s_mission_gcs_sysid,
                s_mission_gcs_compid, &item);
            send_mavlink_msg(&msg);
            break;
        }
        case RPC_MSG_MISSION_ACK: {
            mavlink_message_t msg;
            mavlink_msg_mission_ack_encode(&msg, s_mission_gcs_sysid,
                s_mission_gcs_compid, telem.data.mission_ack.result,
                MAV_MISSION_TYPE_MISSION);
            send_mavlink_msg(&msg);
            s_mission_state = MISSION_STATE_IDLE;
            break;
        }
        case RPC_MSG_MISSION_CURRENT:
            s_mission_current_seq = telem.data.mission_current.seq;
            send_mission_current();
            break;
        default:
            ESP_LOGW(TAG, "Unknown RPC telem type: 0x%02x", telem.msg_type);
            break;
        }
    }
}

/* ── Send MAVLink telemetry at configured rates ───────────────── */

/* Map internal flight_mode_t to PX4 custom_mode encoding */
static uint32_t flight_mode_to_px4_custom(uint8_t fm)
{
    switch (fm) {
    case FLIGHT_MODE_MANUAL:    return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_MANUAL, 0);
    case FLIGHT_MODE_STABILIZE: return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_STABILIZED, 0);
    case FLIGHT_MODE_ALT_HOLD:  return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_ALTCTL, 0);
    case FLIGHT_MODE_LOITER:    return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
    case FLIGHT_MODE_RTH:       return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL);
    case FLIGHT_MODE_LAND:      return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND);
    case FLIGHT_MODE_ACRO:      return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_ACRO, 0);
    case FLIGHT_MODE_MISSION:   return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION);
    default:                    return PX4_CUSTOM_MODE(PX4_CUSTOM_MAIN_MODE_MANUAL, 0);
    }
}

/* Map internal flight_mode_t to PX4 base_mode flags */
static uint8_t flight_mode_to_base_mode(uint8_t fm, bool armed)
{
    uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    switch (fm) {
    case FLIGHT_MODE_LOITER:
    case FLIGHT_MODE_RTH:
    case FLIGHT_MODE_LAND:
    case FLIGHT_MODE_MISSION:
        /* Auto modes */
        base |= MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED
              | MAV_MODE_FLAG_AUTO_ENABLED;
        break;
    default:
        /* Manual/Stabilize/AltHold/Acro */
        base |= MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    }

    if (armed) {
        base |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    return base;
}

/* Map PX4 custom_mode from QGC SET_MODE back to flight_mode_t */
static uint8_t px4_custom_to_flight_mode(uint32_t custom_mode)
{
    uint8_t main_mode = (custom_mode >> 16) & 0xFF;
    uint8_t sub_mode  = (custom_mode >> 24) & 0xFF;

    switch (main_mode) {
    case PX4_CUSTOM_MAIN_MODE_MANUAL:     return FLIGHT_MODE_MANUAL;
    case PX4_CUSTOM_MAIN_MODE_STABILIZED: return FLIGHT_MODE_STABILIZE;
    case PX4_CUSTOM_MAIN_MODE_ALTCTL:     return FLIGHT_MODE_ALT_HOLD;
    case PX4_CUSTOM_MAIN_MODE_ACRO:       return FLIGHT_MODE_ACRO;
    case PX4_CUSTOM_MAIN_MODE_AUTO:
        switch (sub_mode) {
        case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:  return FLIGHT_MODE_LOITER;
        case PX4_CUSTOM_SUB_MODE_AUTO_RTL:     return FLIGHT_MODE_RTH;
        case PX4_CUSTOM_SUB_MODE_AUTO_LAND:    return FLIGHT_MODE_LAND;
        case PX4_CUSTOM_SUB_MODE_AUTO_MISSION: return FLIGHT_MODE_MISSION;
        default:                               return FLIGHT_MODE_LOITER;
        }
    case PX4_CUSTOM_MAIN_MODE_POSCTL: return FLIGHT_MODE_LOITER;
    default:                          return FLIGHT_MODE_MANUAL;
    }
}

static void send_heartbeat(void)
{
    if (!rate_check(&s_last_heartbeat_ms, s_config.heartbeat_hz)) {
        return;
    }

    uint8_t system_status = MAV_STATE_STANDBY;
    uint8_t fm = FLIGHT_MODE_MANUAL;
    bool armed = false;

    if (s_has_status) {
        armed = s_latest_status.data.status.armed;
        fm = s_latest_status.data.status.flight_mode;
        if (armed) system_status = MAV_STATE_ACTIVE;
        if (s_latest_status.data.status.failsafe != 0) system_status = MAV_STATE_CRITICAL;
    }

    uint32_t custom_mode = flight_mode_to_px4_custom(fm);
    uint8_t base_mode = flight_mode_to_base_mode(fm, armed);

    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode(&msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_PX4,
                                 base_mode, custom_mode, system_status);
    send_mavlink_msg(&msg);
}

static void send_attitude(void)
{
    if (!s_has_attitude || !rate_check(&s_last_attitude_ms, s_config.attitude_hz)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_attitude_encode(&msg,
        s_latest_attitude.timestamp_ms,
        s_latest_attitude.data.attitude.roll,
        s_latest_attitude.data.attitude.pitch,
        s_latest_attitude.data.attitude.yaw,
        s_latest_attitude.data.attitude.rollspeed,
        s_latest_attitude.data.attitude.pitchspeed,
        s_latest_attitude.data.attitude.yawspeed);
    send_mavlink_msg(&msg);
}

static void send_gps(void)
{
    if (!s_has_gps || !rate_check(&s_last_gps_ms, s_config.gps_hz)) {
        return;
    }

    /* Convert to MAVLink units: lat/lon in degE7, alt in mm */
    int32_t lat = (int32_t)(s_latest_gps.data.gps.lat * 1.0e7);
    int32_t lon = (int32_t)(s_latest_gps.data.gps.lon * 1.0e7);
    int32_t alt = (int32_t)(s_latest_gps.data.gps.alt * 1000.0f);
    uint16_t eph = (uint16_t)(s_latest_gps.data.gps.hdop * 100.0f);
    uint16_t vel = (uint16_t)(s_latest_gps.data.gps.ground_speed * 100.0f);
    uint16_t cog = (uint16_t)(s_latest_gps.data.gps.course * 100.0f);

    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_encode(&msg,
        (uint64_t)s_latest_gps.timestamp_ms * 1000ULL,
        s_latest_gps.data.gps.fix_type,
        lat, lon, alt, eph, 0xFFFF, vel, cog,
        s_latest_gps.data.gps.satellites);
    send_mavlink_msg(&msg);

    /* Also send GLOBAL_POSITION_INT if we have altitude data */
    if (s_has_altitude) {
        int32_t alt_msl = (int32_t)(s_latest_altitude.data.altitude.alt_msl * 1000.0f);
        int32_t alt_rel = (int32_t)(s_latest_altitude.data.altitude.alt_rel * 1000.0f);

        mavlink_msg_global_position_int_encode(&msg,
            s_latest_gps.timestamp_ms,
            lat, lon, alt_msl, alt_rel,
            0, 0, 0, /* vx, vy, vz - not available from GPS alone */
            0xFFFF); /* heading unknown */
        send_mavlink_msg(&msg);
    }
}

static void send_battery(void)
{
    if (!s_has_battery || !rate_check(&s_last_battery_ms, s_config.battery_hz)) {
        return;
    }

    /* Send as SYS_STATUS (simpler, widely supported) */
    uint16_t voltage_mv = (uint16_t)(s_latest_battery.data.battery.voltage * 1000.0f);
    int16_t current_ca = (int16_t)(s_latest_battery.data.battery.current * 100.0f);
    int8_t remaining = (int8_t)s_latest_battery.data.battery.remaining_pct;

    /* Sensor bitmask: 3D gyro, 3D accel, 3D mag, GPS, battery */
    uint32_t sensors = 0;
    if (s_has_status) {
        if (s_latest_status.data.status.sensor_health & 0x01) sensors |= (1 << 0) | (1 << 1); /* IMU */
        if (s_latest_status.data.status.sensor_health & 0x04) sensors |= (1 << 2); /* MAG */
        if (s_latest_status.data.status.sensor_health & 0x08) sensors |= (1 << 5); /* GPS */
        if (s_latest_status.data.status.sensor_health & 0x02) sensors |= (1 << 3); /* BARO */
    }

    mavlink_message_t msg;
    mavlink_msg_sys_status_encode(&msg,
        sensors, sensors, sensors,
        0, voltage_mv, current_ca, remaining);
    send_mavlink_msg(&msg);
}

static void send_vfr_hud(void)
{
    if (!rate_check(&s_last_vfr_hud_ms, s_config.vfr_hud_hz)) {
        return;
    }

    /* VFR_HUD combines altitude, climb, groundspeed, heading */
    if (!s_has_altitude && !s_has_gps) {
        return;
    }

    float alt = 0.0f, climb = 0.0f, groundspeed = 0.0f;
    int16_t heading = 0;

    if (s_has_altitude) {
        alt = s_latest_altitude.data.altitude.alt_rel;
        climb = s_latest_altitude.data.altitude.climb_rate;
    }
    if (s_has_gps) {
        groundspeed = s_latest_gps.data.gps.ground_speed;
    }
    if (s_has_attitude) {
        /* Convert yaw to heading (0-360) */
        float yaw_deg = s_latest_attitude.data.attitude.yaw * 57.2957795f;
        if (yaw_deg < 0.0f) yaw_deg += 360.0f;
        heading = (int16_t)yaw_deg;
    }

    mavlink_message_t msg;
    mavlink_msg_vfr_hud_encode(&msg, groundspeed, groundspeed,
                               heading, 0, alt, climb);
    send_mavlink_msg(&msg);
}

static void send_servo_output(void)
{
    if (!s_has_servo || !rate_check(&s_last_servo_ms, 4)) { /* 4 Hz */
        return;
    }

    uint16_t servo[8] = {0};
    for (int i = 0; i < 5 && i < 8; i++) {
        servo[i] = s_latest_servo.data.servo_output.servo_us[i];
    }

    mavlink_message_t msg;
    mavlink_msg_servo_output_raw_encode(&msg,
        s_latest_servo.timestamp_ms * 1000, 0, servo);
    send_mavlink_msg(&msg);
}

static void send_statustext_messages(void)
{
    for (int i = 0; i < s_statustext_count; i++) {
        const rpc_telemetry_msg_t *st = &s_statustext_queue[i];
        mavlink_message_t msg;
        mavlink_msg_statustext_encode(&msg,
            st->data.statustext.severity,
            st->data.statustext.text);
        send_mavlink_msg(&msg);
    }
    s_statustext_count = 0;
}

static void send_param_values(void)
{
    for (int i = 0; i < s_param_value_count; i++) {
        const rpc_telemetry_msg_t *pv = &s_param_value_queue[i];
        mavlink_message_t msg;
        mavlink_msg_param_value_encode(&msg,
            pv->data.param_value.name,
            pv->data.param_value.value,
            pv->data.param_value.type,
            pv->data.param_value.count,
            pv->data.param_value.index);
        send_mavlink_msg(&msg);
    }
    s_param_value_count = 0;
}

static void send_extended_sys_state(void)
{
    if (!rate_check(&s_last_ext_state_ms, 1)) { /* 1 Hz */
        return;
    }

    uint8_t landed = MAV_LANDED_STATE_ON_GROUND;
    if (s_has_status && s_latest_status.data.status.armed) {
        landed = MAV_LANDED_STATE_IN_AIR;
    }

    mavlink_message_t msg;
    mavlink_msg_extended_sys_state_encode(&msg, MAV_VTOL_STATE_UNDEFINED, landed);
    send_mavlink_msg(&msg);
}

static void send_home_position(void)
{
    if (!s_has_home || !rate_check(&s_last_home_ms, 1)) { /* 1 Hz */
        return;
    }

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; /* identity quaternion */
    mavlink_message_t msg;
    mavlink_msg_home_position_encode(&msg,
        s_home_lat, s_home_lon, s_home_alt,
        0.0f, 0.0f, 0.0f, q,
        0.0f, 0.0f, 0.0f,
        (uint64_t)get_time_ms() * 1000ULL);
    send_mavlink_msg(&msg);
}

static void send_mission_current(void)
{
    mavlink_message_t msg;
    mavlink_msg_mission_current_encode(&msg, s_mission_current_seq);
    send_mavlink_msg(&msg);
}

static void send_autopilot_version(void)
{
    uint64_t cap = MAV_PROTOCOL_CAPABILITY_MISSION_INT
                 | MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
                 | MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
                 | MAV_PROTOCOL_CAPABILITY_MAVLINK2;

    uint8_t custom_ver[8] = {0};
    mavlink_message_t msg;
    mavlink_msg_autopilot_version_encode(&msg, cap,
        0x01000000, /* flight_sw v1.0.0 */
        0x01000000, /* middleware v1.0.0 */
        0x05030000, /* os: ESP-IDF v5.3 */
        1,          /* board_version */
        custom_ver,
        0x00000001  /* uid */
    );
    send_mavlink_msg(&msg);
}

static void send_telemetry(void)
{
    send_heartbeat();
    send_attitude();
    send_gps();
    send_battery();
    send_vfr_hud();
    send_servo_output();
    send_extended_sys_state();
    send_home_position();
    send_param_values();
    send_statustext_messages();
}

/* ── Process incoming MAVLink commands from GCS ───────────────── */

static void handle_command_long(const mavlink_message_t *msg)
{
    uint16_t command;
    float param1, param2, param3, param4, param5, param6, param7;
    uint8_t target_system, target_component;

    mavlink_msg_command_long_decode(msg, &command,
        &param1, &param2, &param3, &param4,
        &param5, &param6, &param7,
        &target_system, &target_component);

    /* Ignore commands not addressed to us */
    if (target_system != 0 && target_system != MAV_SYS_ID) {
        return;
    }

    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.timestamp_ms = get_time_ms();

    uint8_t result = MAV_RESULT_ACCEPTED;

    switch (command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (param1 >= 0.5f) {
            rpc_cmd.msg_type = RPC_CMD_ARM;
            rpc_cmd.data.arm_cmd.arm = 1;
        } else {
            rpc_cmd.msg_type = RPC_CMD_DISARM;
            rpc_cmd.data.arm_cmd.arm = 0;
        }
        ESP_LOGI(TAG, "ARM/DISARM command: %s", param1 >= 0.5f ? "ARM" : "DISARM");
        break;

    case MAV_CMD_DO_SET_MODE: {
        /* QGC sends param1=base_mode, param2=custom_mode (PX4 encoding) */
        uint32_t cm = (uint32_t)param2;
        uint8_t fm = px4_custom_to_flight_mode(cm);
        rpc_cmd.msg_type = RPC_CMD_SET_MODE;
        rpc_cmd.data.mode_cmd.mode = fm;
        ESP_LOGI(TAG, "SET_MODE: custom_mode=0x%08lx -> flight_mode=%d",
                 (unsigned long)cm, fm);
        break;
    }

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        rpc_cmd.msg_type = RPC_CMD_SET_MODE;
        rpc_cmd.data.mode_cmd.mode = FLIGHT_MODE_RTH;
        ESP_LOGI(TAG, "NAV_RETURN_TO_LAUNCH -> RTH mode");
        break;

    case MAV_CMD_NAV_LAND:
        rpc_cmd.msg_type = RPC_CMD_SET_MODE;
        rpc_cmd.data.mode_cmd.mode = FLIGHT_MODE_LAND;
        ESP_LOGI(TAG, "NAV_LAND -> LAND mode");
        break;

    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        if (param1 >= 1.0f) {
            rpc_cmd.msg_type = RPC_CMD_REBOOT;
            ESP_LOGW(TAG, "REBOOT command received");
        } else {
            result = MAV_RESULT_DENIED;
        }
        break;

    case MAV_CMD_PREFLIGHT_CALIBRATION:
        ESP_LOGI(TAG, "CALIBRATION command (not implemented yet)");
        result = MAV_RESULT_ACCEPTED;
        rpc_cmd.msg_type = 0;
        break;

    case MAV_CMD_REQUEST_MESSAGE:
        if ((uint32_t)param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
            send_autopilot_version();
            result = MAV_RESULT_ACCEPTED;
        } else if ((uint32_t)param1 == MAVLINK_MSG_ID_HOME_POSITION) {
            /* Request home from Core 0 */
            rpc_cmd.msg_type = RPC_CMD_REQUEST_HOME_POSITION;
            result = MAV_RESULT_ACCEPTED;
        } else {
            result = MAV_RESULT_UNSUPPORTED;
        }
        rpc_cmd.msg_type = 0; /* already handled inline */
        break;

    default:
        ESP_LOGW(TAG, "Unsupported MAV_CMD: %d", command);
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }

    /* Send command ACK back to GCS */
    mavlink_message_t ack_msg;
    mavlink_msg_command_ack_encode(&ack_msg, command, result);
    send_mavlink_msg(&ack_msg);

    /* Forward to Core 0 if accepted and has valid msg_type */
    if (result == MAV_RESULT_ACCEPTED && rpc_cmd.msg_type != 0) {
        if (rpc_send_command(s_rpc_ctx, &rpc_cmd) != 0) {
            ESP_LOGW(TAG, "Failed to send RPC command to Core 0");
        }
    }
}

static void handle_heartbeat_from_gcs(const mavlink_message_t *msg)
{
    /* GCS heartbeat received - confirms link is alive */
    (void)msg;
}

static void handle_param_request_read(const mavlink_message_t *msg)
{
    char param_id[17] = {0};
    int16_t param_index;
    uint8_t target_system, target_component;

    mavlink_msg_param_request_read_decode(msg, param_id, &param_index,
                                          &target_system, &target_component);

    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.msg_type = RPC_CMD_PARAM_REQUEST_READ;
    rpc_cmd.timestamp_ms = get_time_ms();
    memcpy(rpc_cmd.data.param_request.name, param_id, 16);
    rpc_cmd.data.param_request.index = param_index;

    if (rpc_send_command(s_rpc_ctx, &rpc_cmd) != 0) {
        ESP_LOGW(TAG, "Failed to send PARAM_REQUEST_READ to Core 0");
    }
    ESP_LOGI(TAG, "PARAM_REQUEST_READ: name='%s' index=%d", param_id, param_index);
}

static void handle_param_request_list(const mavlink_message_t *msg)
{
    /* PARAM_REQUEST_LIST (ID 21): target_system(1) + target_component(1) = 2 bytes */
    uint8_t target_system = msg->payload[0];
    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.msg_type = RPC_CMD_PARAM_REQUEST_LIST;
    rpc_cmd.timestamp_ms = get_time_ms();
    rpc_cmd.data.param_request.index = -1; /* -1 means "all" */

    if (rpc_send_command(s_rpc_ctx, &rpc_cmd) != 0) {
        ESP_LOGW(TAG, "Failed to send PARAM_REQUEST_LIST to Core 0");
    }
    ESP_LOGI(TAG, "PARAM_REQUEST_LIST received");
}

static void handle_param_set(const mavlink_message_t *msg)
{
    char param_id[17] = {0};
    float param_value;
    uint8_t param_type, target_system, target_component;

    mavlink_msg_param_set_decode(msg, param_id, &param_value, &param_type,
                                 &target_system, &target_component);

    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.msg_type = RPC_CMD_PARAM_SET;
    rpc_cmd.timestamp_ms = get_time_ms();
    memcpy(rpc_cmd.data.param_set.name, param_id, 16);
    rpc_cmd.data.param_set.value = param_value;

    ESP_LOGI(TAG, "PARAM_SET: %s = %.4f", param_id, param_value);

    if (rpc_send_command(s_rpc_ctx, &rpc_cmd) != 0) {
        ESP_LOGW(TAG, "Failed to send PARAM_SET to Core 0");
    }
}

static void handle_rc_channels_override(const mavlink_message_t *msg)
{
    uint16_t channels[8];
    uint8_t target_system, target_component;

    mavlink_msg_rc_channels_override_decode(msg, channels,
                                             &target_system, &target_component);

    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.msg_type = RPC_CMD_RC_OVERRIDE;
    rpc_cmd.timestamp_ms = get_time_ms();

    /* Convert PWM (1000-2000) to normalized (-1000 to +1000) for RPC */
    for (int i = 0; i < 8; i++) {
        if (channels[i] == 0 || channels[i] == UINT16_MAX) {
            /* 0 or 0xFFFF means "release" - don't override this channel */
            rpc_cmd.data.rc_override.channels[i] = 0;
        } else {
            /* Map 1000-2000 -> -1000 to +1000 */
            rpc_cmd.data.rc_override.channels[i] = (int16_t)(channels[i] - 1500);
        }
    }

    if (rpc_send_command(s_rpc_ctx, &rpc_cmd) != 0) {
        ESP_LOGW(TAG, "Failed to send RC_OVERRIDE to Core 0");
    }
}

/* ── Mission Protocol Handlers ────────────────────────────────── */

static void handle_mission_request_list(const mavlink_message_t *msg)
{
    uint8_t target_system, target_component, mission_type;
    mavlink_msg_mission_request_list_decode(msg, &target_system, &target_component, &mission_type);
    if (target_system != 0 && target_system != MAV_SYS_ID) return;
    if (mission_type != MAV_MISSION_TYPE_MISSION) return;

    s_mission_gcs_sysid = msg->sysid;
    s_mission_gcs_compid = msg->compid;

    /* Send mission count to GCS */
    ESP_LOGI(TAG, "MISSION_REQUEST_LIST: count=%u", s_mission_count);
    mavlink_message_t reply;
    mavlink_msg_mission_count_encode(&reply, msg->sysid, msg->compid,
                                     s_mission_count, MAV_MISSION_TYPE_MISSION);
    send_mavlink_msg(&reply);
    s_mission_state = MISSION_STATE_DOWNLOADING;
}

static void handle_mission_count(const mavlink_message_t *msg)
{
    uint16_t count;
    uint8_t target_system, target_component, mission_type;
    mavlink_msg_mission_count_decode(msg, &count, &target_system, &target_component, &mission_type);
    if (target_system != 0 && target_system != MAV_SYS_ID) return;
    if (mission_type != MAV_MISSION_TYPE_MISSION) return;

    s_mission_gcs_sysid = msg->sysid;
    s_mission_gcs_compid = msg->compid;

    if (count > MISSION_MAX_ITEMS) {
        ESP_LOGW(TAG, "MISSION_COUNT: %u exceeds max %d", count, MISSION_MAX_ITEMS);
        mavlink_message_t ack;
        mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                       MAV_MISSION_NO_SPACE, MAV_MISSION_TYPE_MISSION);
        send_mavlink_msg(&ack);
        return;
    }

    ESP_LOGI(TAG, "MISSION_COUNT: expecting %u items", count);
    s_mission_expected = count;
    s_mission_received = 0;
    s_mission_count = 0;
    s_mission_state = MISSION_STATE_UPLOADING;

    /* Request first item */
    if (count > 0) {
        mavlink_message_t req;
        mavlink_msg_mission_request_int_encode(&req, msg->sysid, msg->compid,
                                                0, MAV_MISSION_TYPE_MISSION);
        send_mavlink_msg(&req);
    } else {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                       MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        send_mavlink_msg(&ack);
        s_mission_state = MISSION_STATE_IDLE;
    }
}

static void handle_mission_item_int(const mavlink_message_t *msg)
{
    mavlink_mission_item_int_t item;
    uint8_t target_system, target_component;
    mavlink_msg_mission_item_int_decode(msg, &item, &target_system, &target_component);
    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    if (s_mission_state != MISSION_STATE_UPLOADING) {
        ESP_LOGW(TAG, "MISSION_ITEM_INT: not in upload state");
        return;
    }

    if (item.seq >= MISSION_MAX_ITEMS) {
        ESP_LOGW(TAG, "MISSION_ITEM_INT: seq %u out of range", item.seq);
        return;
    }

    /* Store locally */
    s_mission_items[item.seq] = item;
    s_mission_received++;

    ESP_LOGI(TAG, "MISSION_ITEM_INT: seq=%u cmd=%u (%.7f, %.7f, %.1f) [%u/%u]",
             item.seq, item.command,
             (double)item.x / 1e7, (double)item.y / 1e7, item.z,
             s_mission_received, s_mission_expected);

    /* Forward to Core 0 via RPC */
    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.msg_type = RPC_CMD_MISSION_ITEM;
    rpc_cmd.timestamp_ms = get_time_ms();
    rpc_cmd.data.mission_item_cmd.seq = item.seq;
    rpc_cmd.data.mission_item_cmd.frame = item.frame;
    rpc_cmd.data.mission_item_cmd.command = item.command;
    rpc_cmd.data.mission_item_cmd.autocontinue = item.autocontinue;
    rpc_cmd.data.mission_item_cmd.param1 = item.param1;
    rpc_cmd.data.mission_item_cmd.param2 = item.param2;
    rpc_cmd.data.mission_item_cmd.param3 = item.param3;
    rpc_cmd.data.mission_item_cmd.param4 = item.param4;
    rpc_cmd.data.mission_item_cmd.x = item.x;
    rpc_cmd.data.mission_item_cmd.y = item.y;
    rpc_cmd.data.mission_item_cmd.z = item.z;
    rpc_send_command(s_rpc_ctx, &rpc_cmd);

    if (s_mission_received >= s_mission_expected) {
        /* Upload complete */
        s_mission_count = s_mission_expected;
        s_mission_current_seq = 0;
        s_mission_state = MISSION_STATE_IDLE;

        /* Tell Core 0 the total count */
        rpc_command_msg_t cnt_cmd;
        memset(&cnt_cmd, 0, sizeof(cnt_cmd));
        cnt_cmd.msg_type = RPC_CMD_MISSION_COUNT;
        cnt_cmd.timestamp_ms = get_time_ms();
        cnt_cmd.data.mission_count_cmd.count = s_mission_count;
        rpc_send_command(s_rpc_ctx, &cnt_cmd);

        /* ACK to GCS */
        mavlink_message_t ack;
        mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                       MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        send_mavlink_msg(&ack);
        ESP_LOGI(TAG, "Mission upload complete: %u items", s_mission_count);
    } else {
        /* Request next item */
        mavlink_message_t req;
        mavlink_msg_mission_request_int_encode(&req, msg->sysid, msg->compid,
                                                s_mission_received, MAV_MISSION_TYPE_MISSION);
        send_mavlink_msg(&req);
    }
}

static void handle_mission_request_int(const mavlink_message_t *msg)
{
    uint16_t seq;
    uint8_t target_system, target_component, mission_type;
    mavlink_msg_mission_request_int_decode(msg, &seq, &target_system, &target_component, &mission_type);
    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    if (seq >= s_mission_count) {
        ESP_LOGW(TAG, "MISSION_REQUEST_INT: seq %u >= count %u", seq, s_mission_count);
        return;
    }

    mavlink_message_t reply;
    mavlink_msg_mission_item_int_encode(&reply, msg->sysid, msg->compid,
                                        &s_mission_items[seq]);
    send_mavlink_msg(&reply);
}

static void handle_mission_ack(const mavlink_message_t *msg)
{
    /* GCS acknowledges download complete */
    s_mission_state = MISSION_STATE_IDLE;
    ESP_LOGI(TAG, "MISSION_ACK received from GCS");
}

static void handle_mission_clear_all(const mavlink_message_t *msg)
{
    uint8_t target_system, target_component, mission_type;
    mavlink_msg_mission_clear_all_decode(msg, &target_system, &target_component, &mission_type);
    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    ESP_LOGI(TAG, "MISSION_CLEAR_ALL");
    s_mission_count = 0;
    s_mission_current_seq = 0;
    s_mission_state = MISSION_STATE_IDLE;

    /* Forward to Core 0 */
    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.msg_type = RPC_CMD_MISSION_CLEAR_ALL;
    rpc_cmd.timestamp_ms = get_time_ms();
    rpc_send_command(s_rpc_ctx, &rpc_cmd);

    mavlink_message_t ack;
    mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                   MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    send_mavlink_msg(&ack);
}

static void handle_mission_set_current(const mavlink_message_t *msg)
{
    uint16_t seq;
    uint8_t target_system, target_component;
    mavlink_msg_mission_set_current_decode(msg, &seq, &target_system, &target_component);
    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    ESP_LOGI(TAG, "MISSION_SET_CURRENT: seq=%u", seq);
    s_mission_current_seq = seq;
    send_mission_current();

    /* Forward to Core 0 */
    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.msg_type = RPC_CMD_MISSION_SET_CURRENT;
    rpc_cmd.timestamp_ms = get_time_ms();
    rpc_cmd.data.mission_set_current_cmd.seq = seq;
    rpc_send_command(s_rpc_ctx, &rpc_cmd);
}

static void process_mavlink_message(const mavlink_message_t *msg)
{
    /* Ignore messages from ourselves */
    if (msg->sysid == MAV_SYS_ID && msg->compid == MAV_COMP_ID_AUTOPILOT) {
        return;
    }

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_heartbeat_from_gcs(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_command_long(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_param_request_read(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        handle_param_request_list(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        handle_param_set(msg);
        break;
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        handle_rc_channels_override(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        handle_mission_request_list(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_COUNT:
        handle_mission_count(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        handle_mission_item_int(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        handle_mission_request_int(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ACK:
        handle_mission_ack(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        handle_mission_clear_all(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        handle_mission_set_current(msg);
        break;
    default:
        ESP_LOGD(TAG, "Unhandled MAVLink msg ID: %lu from sys=%d comp=%d",
                 (unsigned long)msg->msgid, msg->sysid, msg->compid);
        break;
    }
}

static void process_incoming_mavlink(void)
{
    uint8_t rxbuf[GCS_BRIDGE_MAX_PKT_LEN];
    int len = gcs_bridge_recv(rxbuf, sizeof(rxbuf), 5); /* 5ms timeout */
    if (len <= 0) {
        return;
    }

    mavlink_message_t msg;
    size_t consumed = 0;
    size_t offset = 0;

    while (offset < (size_t)len) {
        if (mavlink_parse(&s_parser, &rxbuf[offset], (size_t)len - offset,
                          &msg, &consumed)) {
            process_mavlink_message(&msg);
        }
        offset += consumed;
        if (consumed == 0) {
            break; /* prevent infinite loop */
        }
    }
}

/* ── Public API ───────────────────────────────────────────────── */

void mavlink_handler_init(rpc_context_t *ctx, const mavlink_handler_config_t *config)
{
    s_rpc_ctx = ctx;

    if (config) {
        s_config = *config;
    } else {
        /* Default rates */
        s_config.heartbeat_hz  = 1;
        s_config.attitude_hz   = 10;
        s_config.gps_hz        = 5;
        s_config.battery_hz    = 2;
        s_config.vfr_hud_hz   = 2;
    }

    mavlink_parser_init(&s_parser);

    s_has_attitude = false;
    s_has_gps = false;
    s_has_altitude = false;
    s_has_battery = false;
    s_has_status = false;
    s_has_rc = false;

    s_has_home = false;
    s_mission_count = 0;
    s_mission_state = MISSION_STATE_IDLE;

    uint32_t now = get_time_ms();
    s_last_heartbeat_ms  = now;
    s_last_attitude_ms   = now;
    s_last_gps_ms        = now;
    s_last_battery_ms    = now;
    s_last_vfr_hud_ms    = now;
    s_last_ext_state_ms  = now;
    s_last_home_ms       = now;

    ESP_LOGI(TAG, "PX4-compat MAVLink handler initialized (HB=%dHz ATT=%dHz GPS=%dHz BAT=%dHz)",
             s_config.heartbeat_hz, s_config.attitude_hz,
             s_config.gps_hz, s_config.battery_hz);
}

void mavlink_handler_task(void *param)
{
    (void)param;

    ESP_LOGI(TAG, "MAVLink handler task started on core %d", xPortGetCoreID());

    while (1) {
        /* 1. Drain all pending RPC telemetry into our cache */
        drain_rpc_telemetry();

        /* 2. Send MAVLink telemetry at configured rates */
        send_telemetry();

        /* 3. Receive and process incoming MAVLink from GCS */
        process_incoming_mavlink();

        /* 4. Yield briefly to avoid starving other tasks.
         *    The main loop rate is bounded by the recv timeout (5ms)
         *    plus processing time, giving roughly 100-200 Hz loop rate. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
