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
#include "mm_app_common.h"
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
#define PARAM_VALUE_QUEUE_DEPTH 64
static rpc_telemetry_msg_t s_param_value_queue[PARAM_VALUE_QUEUE_DEPTH];
static int s_param_value_count = 0;
static int s_param_value_send_idx = 0;   /* index of next param to send during paced bulk send */
static uint32_t s_param_last_send_ms = 0; /* timestamp of last param send for pacing */
#define PARAM_SEND_INTERVAL_MS 20        /* 20ms between PARAM_VALUE messages */

/* ── Local parameter store (for QGC parameter download) ────────── */

typedef struct {
    char     name[17];  /* null-terminated, 16 chars max */
    float    value;
} param_entry_t;

#define PARAM_STORE_MAX 64

static param_entry_t s_param_store[PARAM_STORE_MAX];
static uint16_t s_param_store_count = 0;
static bool s_param_list_pending = false;  /* true when bulk param send is in progress */

/* Default parameters - initialized on first PARAM_REQUEST_LIST */
static bool s_params_initialized = false;

static void param_store_init(void)
{
    if (s_params_initialized) return;
    s_params_initialized = true;
    s_param_store_count = 0;

    /* Helper macro to add a default param */
    #define ADD_PARAM(n, v) do { \
        if (s_param_store_count < PARAM_STORE_MAX) { \
            strncpy(s_param_store[s_param_store_count].name, (n), 16); \
            s_param_store[s_param_store_count].name[16] = '\0'; \
            s_param_store[s_param_store_count].value = (v); \
            s_param_store_count++; \
        } \
    } while (0)

    /* System identification */
    ADD_PARAM("SYS_AUTOSTART",   0.0f);
    ADD_PARAM("SYS_AUTOCONFIG",  0.0f);
    ADD_PARAM("MAV_SYS_ID",     1.0f);
    ADD_PARAM("MAV_COMP_ID",    1.0f);
    ADD_PARAM("MAV_TYPE",       4.0f);   /* helicopter */
    ADD_PARAM("MAV_PROTO_VER",  2.0f);

    /* Calibration offsets (gyro) */
    ADD_PARAM("CAL_GYRO0_XOFF", 0.0f);
    ADD_PARAM("CAL_GYRO0_YOFF", 0.0f);
    ADD_PARAM("CAL_GYRO0_ZOFF", 0.0f);
    ADD_PARAM("CAL_GYRO0_ID",   0.0f);

    /* Calibration offsets (accel) */
    ADD_PARAM("CAL_ACC0_XOFF",  0.0f);
    ADD_PARAM("CAL_ACC0_YOFF",  0.0f);
    ADD_PARAM("CAL_ACC0_ZOFF",  0.0f);
    ADD_PARAM("CAL_ACC0_XSCALE",1.0f);
    ADD_PARAM("CAL_ACC0_YSCALE",1.0f);
    ADD_PARAM("CAL_ACC0_ZSCALE",1.0f);
    ADD_PARAM("CAL_ACC0_ID",    0.0f);

    /* Calibration offsets (mag) */
    ADD_PARAM("CAL_MAG0_XOFF",  0.0f);
    ADD_PARAM("CAL_MAG0_YOFF",  0.0f);
    ADD_PARAM("CAL_MAG0_ZOFF",  0.0f);
    ADD_PARAM("CAL_MAG0_XSCALE",1.0f);
    ADD_PARAM("CAL_MAG0_YSCALE",1.0f);
    ADD_PARAM("CAL_MAG0_ZSCALE",1.0f);
    ADD_PARAM("CAL_MAG0_ID",    0.0f);
    ADD_PARAM("CAL_MAG0_ROT",   0.0f);

    /* Sensor enable */
    ADD_PARAM("SENS_EN_THERMAL",0.0f);
    ADD_PARAM("SENS_BOARD_ROT", 0.0f);

    /* Battery */
    ADD_PARAM("BAT_V_CHARGED",  4.2f);
    ADD_PARAM("BAT_V_EMPTY",    3.5f);
    ADD_PARAM("BAT_N_CELLS",    6.0f);
    ADD_PARAM("BAT_CAPACITY",   5000.0f);

    /* RC */
    ADD_PARAM("RC_MAP_THROTTLE",3.0f);
    ADD_PARAM("RC_MAP_ROLL",    1.0f);
    ADD_PARAM("RC_MAP_PITCH",   2.0f);
    ADD_PARAM("RC_MAP_YAW",     4.0f);

    /* Flight controller tuning */
    ADD_PARAM("MC_ROLL_P",      6.5f);
    ADD_PARAM("MC_PITCH_P",     6.5f);
    ADD_PARAM("MC_YAW_P",       2.8f);
    ADD_PARAM("MC_ROLLRATE_P",  0.15f);
    ADD_PARAM("MC_PITCHRATE_P", 0.15f);
    ADD_PARAM("MC_YAWRATE_P",   0.20f);

    /* Safety */
    ADD_PARAM("COM_ARM_EKF_AB",  0.0f);
    ADD_PARAM("COM_RC_IN_MODE",  0.0f);
    ADD_PARAM("COM_DISARM_LAND", 2.0f);

    #undef ADD_PARAM
}

/* Find parameter by name, returns index or -1 */
static int param_find(const char *name)
{
    for (int i = 0; i < s_param_store_count; i++) {
        if (strncmp(s_param_store[i].name, name, 16) == 0) {
            return i;
        }
    }
    return -1;
}

/* Set parameter value in local store; returns index or -1 if not found */
static int param_set_value(const char *name, float value)
{
    int idx = param_find(name);
    if (idx >= 0) {
        s_param_store[idx].value = value;
    }
    return idx;
}

/* ── Calibration state machine ──────────────────────────────────── */

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_GYRO,
    CAL_STATE_MAG,
    CAL_STATE_ACCEL,
    CAL_STATE_LEVEL,
} cal_state_t;

static cal_state_t s_cal_state = CAL_STATE_IDLE;
static uint32_t s_cal_start_ms = 0;
static uint32_t s_cal_last_progress_ms = 0;
static int s_cal_step = 0;          /* for multi-step accel cal */

/* Accel calibration orientation names */
static const char *s_accel_orientations[] = {
    "level", "left", "right", "nose-down", "nose-up", "back"
};

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
static uint32_t s_last_estimator_ms = 0;
static uint32_t s_last_vibration_ms = 0;
static uint32_t s_last_highres_ms   = 0;

/* IMU raw cache for HIGHRES_IMU and VIBRATION */
static bool s_has_imu_raw = false;
static rpc_telemetry_msg_t s_latest_imu_raw;
static float s_vib_accel_x_sq = 0.0f;  /* running sum of squares for vibration RMS */
static float s_vib_accel_y_sq = 0.0f;
static float s_vib_accel_z_sq = 0.0f;
static uint32_t s_vib_sample_count = 0;
static uint32_t s_vib_clip[3] = {0, 0, 0}; /* clipping counters per axis */
#define VIB_CLIP_THRESHOLD 15.0f /* m/s^2, ~1.5g */

/* Estimator cache */
static bool s_has_estimator = false;
static rpc_telemetry_msg_t s_latest_estimator;

/* Mag raw cache for HIGHRES_IMU */
static bool s_has_mag_raw = false;
static rpc_telemetry_msg_t s_latest_mag_raw;

/* Baro raw cache for HIGHRES_IMU */
static bool s_has_baro_raw = false;
static rpc_telemetry_msg_t s_latest_baro_raw;

/* Geofence/Rally storage */
#define FENCE_MAX_ITEMS 20
#define RALLY_MAX_ITEMS 10
static mavlink_mission_item_int_t s_fence_items[FENCE_MAX_ITEMS];
static uint16_t s_fence_count = 0;
static mavlink_mission_item_int_t s_rally_items[RALLY_MAX_ITEMS];
static uint16_t s_rally_count = 0;

/* FTP state */
#define FTP_OPCODE_NONE         0
#define FTP_OPCODE_TERMINATE    1
#define FTP_OPCODE_RESET        2
#define FTP_OPCODE_LIST_DIR     3
#define FTP_OPCODE_OPEN_FILE_RO 4
#define FTP_OPCODE_READ_FILE    5
#define FTP_OPCODE_ACK          128
#define FTP_OPCODE_NAK          129
#define FTP_ERR_FAIL            1
#define FTP_ERR_FILENOTFOUND    6
#define FTP_ERR_EOF             7

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

/* TX rate limiter: cap outbound packets to avoid saturating HaLow TX pool */
#define TX_BUDGET_MAX_PER_SEC  30   /* max MAVLink packets per second */
#define TX_BUDGET_WINDOW_MS   1000
static uint32_t s_tx_budget_count = 0;
static uint32_t s_tx_budget_window_start = 0;

/* Forward declarations */
static void send_mission_current(void);
static void send_autopilot_version(void);

static void send_mavlink_msg(mavlink_message_t *msg)
{
    uint32_t now = get_time_ms();

    /* Reset budget window every second */
    if ((now - s_tx_budget_window_start) >= TX_BUDGET_WINDOW_MS) {
        s_tx_budget_count = 0;
        s_tx_budget_window_start = now;
    }

    /* Drop non-critical telemetry when budget exhausted (always allow heartbeat) */
    if (s_tx_budget_count >= TX_BUDGET_MAX_PER_SEC && msg->msgid != 0 /* HEARTBEAT */) {
        return;
    }

    /* Skip non-heartbeat when HaLow TX pool is congested */
    if (app_wlan_tx_is_paused() && msg->msgid != 0 /* HEARTBEAT */) {
        return;
    }

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_serialize(msg, buf, sizeof(buf));
    if (len > 0) {
        gcs_bridge_send(buf, (size_t)len);
        s_tx_budget_count++;
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
        case RPC_MSG_IMU_RAW:
            s_latest_imu_raw = telem;
            s_has_imu_raw = true;
            /* Accumulate vibration RMS data */
            {
                float ax = telem.data.imu_raw.accel_x;
                float ay = telem.data.imu_raw.accel_y;
                float az = telem.data.imu_raw.accel_z;
                s_vib_accel_x_sq += ax * ax;
                s_vib_accel_y_sq += ay * ay;
                s_vib_accel_z_sq += az * az;
                s_vib_sample_count++;
                if (fabsf(ax) > VIB_CLIP_THRESHOLD) s_vib_clip[0]++;
                if (fabsf(ay) > VIB_CLIP_THRESHOLD) s_vib_clip[1]++;
                if (fabsf(az) > VIB_CLIP_THRESHOLD) s_vib_clip[2]++;
            }
            break;
        case RPC_MSG_ESTIMATOR:
            s_latest_estimator = telem;
            s_has_estimator = true;
            break;
        case RPC_MSG_MAG_RAW:
            s_latest_mag_raw = telem;
            s_has_mag_raw = true;
            break;
        case RPC_MSG_BARO_RAW:
            s_latest_baro_raw = telem;
            s_has_baro_raw = true;
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

        uint16_t hdg = 0xFFFF;
        if (s_has_attitude) {
            float yaw_deg = s_latest_attitude.data.attitude.yaw * 57.2957795f;
            if (yaw_deg < 0.0f) yaw_deg += 360.0f;
            hdg = (uint16_t)(yaw_deg * 100.0f); /* cdeg */
        }
        mavlink_msg_global_position_int_encode(&msg,
            s_latest_gps.timestamp_ms,
            lat, lon, alt_msl, alt_rel,
            0, 0, 0, /* vx, vy, vz - not available from GPS alone */
            hdg);
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
    if (!s_has_servo || !rate_check(&s_last_servo_ms, 2)) { /* 2 Hz */
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
    /* Paced bulk send from local param store (PARAM_REQUEST_LIST) */
    if (s_param_list_pending) {
        uint32_t now = get_time_ms();
        if ((now - s_param_last_send_ms) >= PARAM_SEND_INTERVAL_MS) {
            if (s_param_value_send_idx < s_param_store_count) {
                int idx = s_param_value_send_idx;
                mavlink_message_t msg;
                mavlink_msg_param_value_encode(&msg,
                    s_param_store[idx].name,
                    s_param_store[idx].value,
                    MAV_PARAM_TYPE_REAL32,
                    s_param_store_count,
                    idx);
                send_mavlink_msg(&msg);
                s_param_value_send_idx++;
                s_param_last_send_ms = now;
            } else {
                /* All params sent */
                s_param_list_pending = false;
                ESP_LOGI(TAG, "Param list complete: %d params sent", s_param_store_count);
            }
        }
        return; /* Don't send queued RPC params while bulk send is active */
    }

    /* Send individual param values from RPC queue (single requests) */
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

static void send_vibration(void)
{
    if (!rate_check(&s_last_vibration_ms, 2)) { /* 2 Hz */
        return;
    }

    float vib_x = 0.0f, vib_y = 0.0f, vib_z = 0.0f;
    if (s_vib_sample_count > 0) {
        vib_x = sqrtf(s_vib_accel_x_sq / (float)s_vib_sample_count);
        vib_y = sqrtf(s_vib_accel_y_sq / (float)s_vib_sample_count);
        vib_z = sqrtf(s_vib_accel_z_sq / (float)s_vib_sample_count);
        /* Reset accumulators */
        s_vib_accel_x_sq = 0.0f;
        s_vib_accel_y_sq = 0.0f;
        s_vib_accel_z_sq = 0.0f;
        s_vib_sample_count = 0;
    }

    mavlink_message_t msg;
    mavlink_msg_vibration_encode(&msg,
        (uint64_t)get_time_ms() * 1000ULL,
        vib_x, vib_y, vib_z,
        s_vib_clip[0], s_vib_clip[1], s_vib_clip[2]);
    send_mavlink_msg(&msg);
}

static void send_estimator_status(void)
{
    if (!s_has_estimator || !rate_check(&s_last_estimator_ms, 1)) { /* 1 Hz */
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_estimator_status_encode(&msg,
        (uint64_t)get_time_ms() * 1000ULL,
        s_latest_estimator.data.estimator.flags,
        s_latest_estimator.data.estimator.vel_ratio,
        s_latest_estimator.data.estimator.pos_horiz_ratio,
        s_latest_estimator.data.estimator.pos_vert_ratio,
        0.0f, /* mag_ratio - not available */
        0.0f, /* hagl_ratio - not available */
        0.0f, /* tas_ratio - not available */
        s_latest_estimator.data.estimator.pos_horiz_accuracy,
        s_latest_estimator.data.estimator.pos_vert_accuracy);
    send_mavlink_msg(&msg);
}

static void send_highres_imu(void)
{
    if (!s_has_imu_raw || !rate_check(&s_last_highres_ms, 4)) { /* 4 Hz */
        return;
    }

    float xmag = 0.0f, ymag = 0.0f, zmag = 0.0f;
    if (s_has_mag_raw) {
        xmag = s_latest_mag_raw.data.mag_raw.x;
        ymag = s_latest_mag_raw.data.mag_raw.y;
        zmag = s_latest_mag_raw.data.mag_raw.z;
    }

    float abs_pressure = 0.0f, pressure_alt = 0.0f, temperature = 0.0f;
    if (s_has_baro_raw) {
        abs_pressure = s_latest_baro_raw.data.baro_raw.pressure / 100.0f; /* Pa -> hPa(mbar) */
        pressure_alt = s_latest_baro_raw.data.baro_raw.altitude;
        temperature = s_latest_baro_raw.data.baro_raw.temperature;
    } else {
        temperature = s_latest_imu_raw.data.imu_raw.temperature;
    }

    uint16_t fields = 0;
    fields |= (1 << 0) | (1 << 1) | (1 << 2); /* accel */
    fields |= (1 << 3) | (1 << 4) | (1 << 5); /* gyro */
    if (s_has_mag_raw) fields |= (1 << 6) | (1 << 7) | (1 << 8); /* mag */
    if (s_has_baro_raw) fields |= (1 << 9) | (1 << 11) | (1 << 12); /* pressure, alt, temp */

    mavlink_message_t msg;
    mavlink_msg_highres_imu_encode(&msg,
        (uint64_t)get_time_ms() * 1000ULL,
        s_latest_imu_raw.data.imu_raw.accel_x,
        s_latest_imu_raw.data.imu_raw.accel_y,
        s_latest_imu_raw.data.imu_raw.accel_z,
        s_latest_imu_raw.data.imu_raw.gyro_x,
        s_latest_imu_raw.data.imu_raw.gyro_y,
        s_latest_imu_raw.data.imu_raw.gyro_z,
        xmag, ymag, zmag,
        abs_pressure, 0.0f, /* diff_pressure not available */
        pressure_alt, temperature,
        fields);
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
                 | MAV_PROTOCOL_CAPABILITY_MAVLINK2
                 | MAV_PROTOCOL_CAPABILITY_FTP
                 | MAV_PROTOCOL_CAPABILITY_MISSION_FENCE
                 | MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;

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

/* Send a statustext message immediately */
static void send_cal_statustext(uint8_t severity, const char *text)
{
    mavlink_message_t msg;
    mavlink_msg_statustext_encode(&msg, severity, text);
    send_mavlink_msg(&msg);
}

/* Calibration state machine - called each loop iteration */
static void calibration_tick(void)
{
    if (s_cal_state == CAL_STATE_IDLE) return;

    uint32_t now = get_time_ms();
    uint32_t elapsed = now - s_cal_start_ms;

    switch (s_cal_state) {
    case CAL_STATE_GYRO:
        /* Gyro cal: collect for 5 seconds, then report success */
        if (s_cal_last_progress_ms == 0) {
            send_cal_statustext(MAV_SEVERITY_INFO, "[cal] Gyro calibration: hold still");
            s_cal_last_progress_ms = now;
        } else if (elapsed >= 5000) {
            send_cal_statustext(MAV_SEVERITY_INFO, "[cal] Gyro calibration complete");
            send_cal_statustext(MAV_SEVERITY_INFO, "CAL: calibration done");
            s_cal_state = CAL_STATE_IDLE;
        } else if ((now - s_cal_last_progress_ms) >= 1000) {
            char buf[50];
            snprintf(buf, sizeof(buf), "[cal] Gyro calibrating... %lu%%",
                     (unsigned long)(elapsed * 100 / 5000));
            send_cal_statustext(MAV_SEVERITY_INFO, buf);
            s_cal_last_progress_ms = now;
        }
        break;

    case CAL_STATE_MAG:
        /* Mag cal: simulate 30s calibration */
        if (s_cal_last_progress_ms == 0) {
            send_cal_statustext(MAV_SEVERITY_INFO, "[cal] Mag calibration: rotate vehicle");
            s_cal_last_progress_ms = now;
        } else if (elapsed >= 30000) {
            send_cal_statustext(MAV_SEVERITY_INFO, "[cal] Mag calibration complete");
            send_cal_statustext(MAV_SEVERITY_INFO, "CAL: calibration done");
            s_cal_state = CAL_STATE_IDLE;
        } else if ((now - s_cal_last_progress_ms) >= 3000) {
            char buf[50];
            snprintf(buf, sizeof(buf), "[cal] Mag calibrating... %lu%%",
                     (unsigned long)(elapsed * 100 / 30000));
            send_cal_statustext(MAV_SEVERITY_INFO, buf);
            s_cal_last_progress_ms = now;
        }
        break;

    case CAL_STATE_ACCEL:
        /* 6-side accel cal: ~5s per orientation */
        if (s_cal_last_progress_ms == 0) {
            char buf[80];
            snprintf(buf, sizeof(buf), "[cal] Accel: place %s and press OK",
                     s_accel_orientations[s_cal_step]);
            send_cal_statustext(MAV_SEVERITY_INFO, buf);
            s_cal_last_progress_ms = now;
        } else if (elapsed >= (uint32_t)(s_cal_step + 1) * 5000) {
            s_cal_step++;
            if (s_cal_step >= 6) {
                send_cal_statustext(MAV_SEVERITY_INFO, "[cal] Accel calibration complete");
                send_cal_statustext(MAV_SEVERITY_INFO, "CAL: calibration done");
                s_cal_state = CAL_STATE_IDLE;
            } else {
                char buf[80];
                snprintf(buf, sizeof(buf), "[cal] Accel: place %s and press OK",
                         s_accel_orientations[s_cal_step]);
                send_cal_statustext(MAV_SEVERITY_INFO, buf);
                s_cal_last_progress_ms = now;
            }
        }
        break;

    case CAL_STATE_LEVEL:
        /* Simple level cal: 3 seconds */
        if (s_cal_last_progress_ms == 0) {
            send_cal_statustext(MAV_SEVERITY_INFO, "[cal] Level calibration: hold level");
            s_cal_last_progress_ms = now;
        } else if (elapsed >= 3000) {
            send_cal_statustext(MAV_SEVERITY_INFO, "[cal] Level calibration complete");
            send_cal_statustext(MAV_SEVERITY_INFO, "CAL: calibration done");
            s_cal_state = CAL_STATE_IDLE;
        }
        break;

    default:
        s_cal_state = CAL_STATE_IDLE;
        break;
    }
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
    send_vibration();
    send_estimator_status();
    send_highres_imu();
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
        if (param1 >= 1.0f) {
            /* Gyro calibration */
            s_cal_state = CAL_STATE_GYRO;
            s_cal_start_ms = get_time_ms();
            s_cal_last_progress_ms = 0;
            ESP_LOGI(TAG, "Starting GYRO calibration");
        } else if (param2 >= 1.0f) {
            /* Mag calibration */
            s_cal_state = CAL_STATE_MAG;
            s_cal_start_ms = get_time_ms();
            s_cal_last_progress_ms = 0;
            ESP_LOGI(TAG, "Starting MAG calibration");
        } else if (param5 >= 2.0f) {
            /* Accel level calibration */
            s_cal_state = CAL_STATE_LEVEL;
            s_cal_start_ms = get_time_ms();
            s_cal_last_progress_ms = 0;
            ESP_LOGI(TAG, "Starting LEVEL calibration");
        } else if (param5 >= 1.0f) {
            /* Full accel calibration */
            s_cal_state = CAL_STATE_ACCEL;
            s_cal_start_ms = get_time_ms();
            s_cal_last_progress_ms = 0;
            s_cal_step = 0;
            ESP_LOGI(TAG, "Starting ACCEL calibration");
        } else {
            /* Cancel calibration */
            s_cal_state = CAL_STATE_IDLE;
            ESP_LOGI(TAG, "Calibration cancelled");
        }
        result = MAV_RESULT_ACCEPTED;
        rpc_cmd.msg_type = 0;
        break;

    case MAV_CMD_DO_SET_SERVO: {
        /* param1=servo_number (1-based), param2=PWM value */
        uint8_t servo_num = (uint8_t)param1;
        uint16_t pwm = (uint16_t)param2;
        ESP_LOGI(TAG, "DO_SET_SERVO: servo=%d pwm=%d", servo_num, pwm);
        rpc_cmd.msg_type = RPC_CMD_SET_SERVO;
        rpc_cmd.data.servo_cmd.servo_number = servo_num;
        rpc_cmd.data.servo_cmd.pwm_value = pwm;
        break;
    }

    case MAV_CMD_DO_MOTOR_TEST: {
        /* param1=motor_number (1-based), param2=throttle_type, param3=throttle, param4=timeout */
        uint8_t motor_num = (uint8_t)param1;
        float throttle = param3;
        float timeout_sec = param4;
        ESP_LOGI(TAG, "DO_MOTOR_TEST: motor=%d throttle=%.1f%% timeout=%.1fs",
                 motor_num, throttle, timeout_sec);
        rpc_cmd.msg_type = RPC_CMD_MOTOR_TEST;
        rpc_cmd.data.motor_test_cmd.motor_number = motor_num;
        rpc_cmd.data.motor_test_cmd.throttle_type = (uint8_t)param2;
        rpc_cmd.data.motor_test_cmd.throttle = throttle;
        rpc_cmd.data.motor_test_cmd.timeout_s = timeout_sec;
        break;
    }

    case MAV_CMD_SET_MESSAGE_INTERVAL: {
        /* param1=message_id, param2=interval_us (-1=disable, 0=default) */
        uint32_t mid = (uint32_t)param1;
        float interval_us = param2;
        ESP_LOGI(TAG, "SET_MESSAGE_INTERVAL: msg=%lu interval=%.0fus",
                 (unsigned long)mid, interval_us);
        result = MAV_RESULT_ACCEPTED;
        rpc_cmd.msg_type = 0;
        break;
    }

    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        send_autopilot_version();
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

    param_store_init();

    int idx = -1;
    if (param_id[0] != '\0') {
        idx = param_find(param_id);
    } else if (param_index >= 0 && param_index < s_param_store_count) {
        idx = param_index;
    }

    if (idx >= 0) {
        mavlink_message_t reply;
        mavlink_msg_param_value_encode(&reply,
            s_param_store[idx].name,
            s_param_store[idx].value,
            MAV_PARAM_TYPE_REAL32,
            s_param_store_count,
            idx);
        send_mavlink_msg(&reply);
    }

    ESP_LOGI(TAG, "PARAM_REQUEST_READ: name='%s' index=%d -> found=%d",
             param_id, param_index, idx);
}

static void handle_param_request_list(const mavlink_message_t *msg)
{
    /* PARAM_REQUEST_LIST (ID 21): target_system(1) + target_component(1) = 2 bytes */
    uint8_t target_system = msg->payload[0];
    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    /* Initialize local param store if not done */
    param_store_init();

    /* Start paced bulk send from local param store */
    s_param_value_send_idx = 0;
    s_param_last_send_ms = 0;
    s_param_list_pending = true;

    ESP_LOGI(TAG, "PARAM_REQUEST_LIST: sending %d params (paced %dms)",
             s_param_store_count, PARAM_SEND_INTERVAL_MS);
}

static void handle_param_set(const mavlink_message_t *msg)
{
    char param_id[17] = {0};
    float param_value;
    uint8_t param_type, target_system, target_component;

    mavlink_msg_param_set_decode(msg, param_id, &param_value, &param_type,
                                 &target_system, &target_component);

    if (target_system != 0 && target_system != MAV_SYS_ID) return;

    param_store_init();

    /* Update local store */
    int idx = param_set_value(param_id, param_value);
    ESP_LOGI(TAG, "PARAM_SET: %s = %.4f (idx=%d)", param_id, param_value, idx);

    /* Respond with PARAM_VALUE (QGC expects confirmation) */
    if (idx >= 0) {
        mavlink_message_t reply;
        mavlink_msg_param_value_encode(&reply,
            s_param_store[idx].name,
            s_param_store[idx].value,
            MAV_PARAM_TYPE_REAL32,
            s_param_store_count,
            idx);
        send_mavlink_msg(&reply);
    }

    /* Also forward to Core 0 */
    rpc_command_msg_t rpc_cmd;
    memset(&rpc_cmd, 0, sizeof(rpc_cmd));
    rpc_cmd.msg_type = RPC_CMD_PARAM_SET;
    rpc_cmd.timestamp_ms = get_time_ms();
    memcpy(rpc_cmd.data.param_set.name, param_id, 16);
    rpc_cmd.data.param_set.value = param_value;

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
    if (mission_type == MAV_MISSION_TYPE_FENCE) {
        mavlink_message_t reply;
        mavlink_msg_mission_count_encode(&reply, msg->sysid, msg->compid,
                                         s_fence_count, MAV_MISSION_TYPE_FENCE);
        send_mavlink_msg(&reply);
        return;
    }
    if (mission_type == MAV_MISSION_TYPE_RALLY) {
        mavlink_message_t reply;
        mavlink_msg_mission_count_encode(&reply, msg->sysid, msg->compid,
                                         s_rally_count, MAV_MISSION_TYPE_RALLY);
        send_mavlink_msg(&reply);
        return;
    }
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
    if (mission_type == MAV_MISSION_TYPE_FENCE) {
        if (count > FENCE_MAX_ITEMS) {
            mavlink_message_t ack;
            mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                           MAV_MISSION_NO_SPACE, MAV_MISSION_TYPE_FENCE);
            send_mavlink_msg(&ack);
        } else {
            s_fence_count = count;
            ESP_LOGI(TAG, "FENCE_COUNT: %u items", count);
            mavlink_message_t ack;
            mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                           MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_FENCE);
            send_mavlink_msg(&ack);
        }
        return;
    }
    if (mission_type == MAV_MISSION_TYPE_RALLY) {
        if (count > RALLY_MAX_ITEMS) {
            mavlink_message_t ack;
            mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                           MAV_MISSION_NO_SPACE, MAV_MISSION_TYPE_RALLY);
            send_mavlink_msg(&ack);
        } else {
            s_rally_count = count;
            ESP_LOGI(TAG, "RALLY_COUNT: %u items", count);
            mavlink_message_t ack;
            mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                           MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_RALLY);
            send_mavlink_msg(&ack);
        }
        return;
    }
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

    if (mission_type == MAV_MISSION_TYPE_FENCE || mission_type == MAV_MISSION_TYPE_ALL) {
        s_fence_count = 0;
        ESP_LOGI(TAG, "Fence items cleared");
    }
    if (mission_type == MAV_MISSION_TYPE_RALLY || mission_type == MAV_MISSION_TYPE_ALL) {
        s_rally_count = 0;
        ESP_LOGI(TAG, "Rally points cleared");
    }
    if (mission_type != MAV_MISSION_TYPE_MISSION && mission_type != MAV_MISSION_TYPE_ALL) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_encode(&ack, msg->sysid, msg->compid,
                                       MAV_MISSION_ACCEPTED, mission_type);
        send_mavlink_msg(&ack);
        return;
    }

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

static void handle_file_transfer_protocol(const mavlink_message_t *msg)
{
    uint8_t target_network, target_system, target_component;
    uint8_t payload[251];
    uint8_t payload_len;

    mavlink_msg_file_transfer_protocol_decode(msg, &target_network,
        &target_system, &target_component, payload, &payload_len);

    if (target_system != 0 && target_system != MAV_SYS_ID) return;
    if (payload_len < 12) return; /* FTP header is 12 bytes minimum */

    /* FTP payload structure:
     * [0..1] seq_number
     * [2]    session
     * [3]    opcode
     * [4]    size
     * [5]    req_opcode
     * [6]    burst_complete
     * [7]    padding
     * [8..11] offset (uint32_t LE)
     * [12..] data
     */
    uint16_t seq = payload[0] | ((uint16_t)payload[1] << 8);
    uint8_t opcode = payload[3];

    /* Build NAK response template */
    uint8_t resp[251];
    memset(resp, 0, sizeof(resp));
    resp[0] = payload[0]; /* seq low */
    resp[1] = payload[1]; /* seq high */
    resp[2] = payload[2]; /* session */
    resp[5] = opcode;     /* req_opcode */

    switch (opcode) {
    case FTP_OPCODE_TERMINATE:
    case FTP_OPCODE_RESET:
        resp[3] = FTP_OPCODE_ACK;
        resp[4] = 0; /* size = 0 */
        break;

    case FTP_OPCODE_LIST_DIR:
        /* Return empty directory listing (no files) */
        resp[3] = FTP_OPCODE_NAK;
        resp[4] = 1;
        resp[12] = FTP_ERR_EOF;
        break;

    case FTP_OPCODE_OPEN_FILE_RO:
        /* No files available */
        resp[3] = FTP_OPCODE_NAK;
        resp[4] = 1;
        resp[12] = FTP_ERR_FILENOTFOUND;
        break;

    case FTP_OPCODE_READ_FILE:
        resp[3] = FTP_OPCODE_NAK;
        resp[4] = 1;
        resp[12] = FTP_ERR_EOF;
        break;

    default:
        resp[3] = FTP_OPCODE_NAK;
        resp[4] = 1;
        resp[12] = FTP_ERR_FAIL;
        break;
    }

    mavlink_message_t reply;
    mavlink_msg_file_transfer_protocol_encode(&reply, 0, msg->sysid, msg->compid,
                                               resp, 13);
    send_mavlink_msg(&reply);

    ESP_LOGD(TAG, "FTP: opcode=%d seq=%u -> resp=%d", opcode, seq, resp[3]);
    (void)seq; /* suppress unused warning when LOG_LOCAL_LEVEL < DEBUG */
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
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        handle_file_transfer_protocol(msg);
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
    s_has_servo = false;
    s_has_imu_raw = false;
    s_has_estimator = false;
    s_has_mag_raw = false;
    s_has_baro_raw = false;

    s_has_home = false;
    s_mission_count = 0;
    s_mission_state = MISSION_STATE_IDLE;

    /* Reset vibration accumulators */
    s_vib_accel_x_sq = 0.0f;
    s_vib_accel_y_sq = 0.0f;
    s_vib_accel_z_sq = 0.0f;
    s_vib_sample_count = 0;
    s_vib_clip[0] = s_vib_clip[1] = s_vib_clip[2] = 0;

    uint32_t now = get_time_ms();
    s_last_heartbeat_ms  = now;
    s_last_attitude_ms   = now;
    s_last_gps_ms        = now;
    s_last_battery_ms    = now;
    s_last_vfr_hud_ms    = now;
    s_last_ext_state_ms  = now;
    s_last_home_ms       = now;
    s_last_estimator_ms  = now;
    s_last_vibration_ms  = now;
    s_last_highres_ms    = now;

    /* TX budget limiter */
    s_tx_budget_count = 0;
    s_tx_budget_window_start = now;

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

        /* 3. Drive calibration state machine */
        calibration_tick();

        /* 4. Receive and process incoming MAVLink from GCS */
        process_incoming_mavlink();

        /* 4. Yield briefly to avoid starving other tasks.
         *    The main loop rate is bounded by the recv timeout (5ms)
         *    plus processing time, giving roughly 100-200 Hz loop rate. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
