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
#include "rpc/rpc_messages.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

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

static bool s_has_attitude  = false;
static bool s_has_gps       = false;
static bool s_has_altitude  = false;
static bool s_has_battery   = false;
static bool s_has_status    = false;
static bool s_has_rc        = false;

/* Timing for rate-limited sends */
static uint32_t s_last_heartbeat_ms = 0;
static uint32_t s_last_attitude_ms  = 0;
static uint32_t s_last_gps_ms       = 0;
static uint32_t s_last_battery_ms   = 0;
static uint32_t s_last_sys_status_ms = 0;
static uint32_t s_last_vfr_hud_ms   = 0;

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
        default:
            ESP_LOGW(TAG, "Unknown RPC telem type: 0x%02x", telem.msg_type);
            break;
        }
    }
}

/* ── Send MAVLink telemetry at configured rates ───────────────── */

static void send_heartbeat(void)
{
    if (!rate_check(&s_last_heartbeat_ms, s_config.heartbeat_hz)) {
        return;
    }

    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_STANDBY;
    uint32_t custom_mode = 0;

    if (s_has_status) {
        if (s_latest_status.data.status.armed) {
            base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
            system_status = MAV_STATE_ACTIVE;
        }
        custom_mode = s_latest_status.data.status.flight_mode;

        if (s_latest_status.data.status.failsafe != 0) {
            system_status = MAV_STATE_CRITICAL;
        }
    }

    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode(&msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC,
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

static void send_telemetry(void)
{
    send_heartbeat();
    send_attitude();
    send_gps();
    send_battery();
    send_vfr_hud();
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

    case MAV_CMD_DO_SET_MODE:
        rpc_cmd.msg_type = RPC_CMD_SET_MODE;
        rpc_cmd.data.mode_cmd.mode = (uint8_t)param2; /* custom_mode in param2 */
        ESP_LOGI(TAG, "SET_MODE command: mode=%d", rpc_cmd.data.mode_cmd.mode);
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

    /* Forward to Core 0 if accepted */
    if (result == MAV_RESULT_ACCEPTED) {
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
        s_config.sys_status_hz = 1;
        s_config.vfr_hud_hz    = 2;
    }

    mavlink_parser_init(&s_parser);

    s_has_attitude = false;
    s_has_gps = false;
    s_has_altitude = false;
    s_has_battery = false;
    s_has_status = false;
    s_has_rc = false;

    uint32_t now = get_time_ms();
    s_last_heartbeat_ms  = now;
    s_last_attitude_ms   = now;
    s_last_gps_ms        = now;
    s_last_battery_ms    = now;
    s_last_sys_status_ms = now;
    s_last_vfr_hud_ms    = now;

    ESP_LOGI(TAG, "MAVLink handler initialized (HB=%dHz ATT=%dHz GPS=%dHz BAT=%dHz)",
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
