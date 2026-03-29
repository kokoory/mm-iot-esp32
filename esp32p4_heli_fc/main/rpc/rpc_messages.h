/*
 * RPC Message Definitions - Shared between Core 0 (FC) and Core 1 (HaLow)
 *
 * Core 0 sends telemetry messages to Core 1 for MAVLink transmission.
 * Core 1 sends command messages to Core 0 for flight controller execution.
 */
#pragma once
#include <stdint.h>

/* Message types: Core 0 -> Core 1 (telemetry) */
enum {
    RPC_MSG_HEARTBEAT   = 0x01,
    RPC_MSG_ATTITUDE    = 0x02,
    RPC_MSG_GPS         = 0x03,
    RPC_MSG_ALTITUDE    = 0x04,
    RPC_MSG_BATTERY     = 0x05,
    RPC_MSG_STATUS      = 0x06,
    RPC_MSG_RC_CHANNELS = 0x07,
    RPC_MSG_PARAM_VALUE   = 0x08,
    RPC_MSG_SERVO_OUTPUT  = 0x09,
    RPC_MSG_STATUSTEXT    = 0x0A,
    RPC_MSG_HOME_POSITION = 0x0B,
    RPC_MSG_MISSION_COUNT = 0x0C,
    RPC_MSG_MISSION_ITEM  = 0x0D,
    RPC_MSG_MISSION_ACK   = 0x0E,
    RPC_MSG_MISSION_CURRENT = 0x0F,
    RPC_MSG_IMU_RAW         = 0x10,
    RPC_MSG_ESTIMATOR       = 0x11,
    RPC_MSG_MAG_RAW         = 0x12,
    RPC_MSG_BARO_RAW        = 0x13,
};

/* Message types: Core 1 -> Core 0 (commands) */
enum {
    RPC_CMD_ARM         = 0x80,
    RPC_CMD_DISARM      = 0x81,
    RPC_CMD_SET_MODE    = 0x82,
    RPC_CMD_RC_OVERRIDE = 0x83,
    RPC_CMD_PARAM_SET         = 0x84,
    RPC_CMD_REBOOT            = 0x85,
    RPC_CMD_PARAM_REQUEST_READ = 0x86,
    RPC_CMD_PARAM_REQUEST_LIST = 0x87,
    RPC_CMD_PARAM_SAVE        = 0x88,
    RPC_CMD_MISSION_COUNT     = 0x89,
    RPC_CMD_MISSION_ITEM      = 0x8A,
    RPC_CMD_MISSION_REQUEST_LIST = 0x8B,
    RPC_CMD_MISSION_CLEAR_ALL = 0x8C,
    RPC_CMD_MISSION_SET_CURRENT = 0x8D,
    RPC_CMD_REQUEST_HOME_POSITION = 0x8E,
};

/* Telemetry message (Core 0 -> Core 1) */
typedef struct {
    uint8_t  msg_type;
    uint32_t timestamp_ms;
    union {
        struct {
            float roll, pitch, yaw;
            float rollspeed, pitchspeed, yawspeed;
        } attitude;

        struct {
            double lat, lon;
            float  alt;
            uint8_t fix_type;
            uint8_t satellites;
            float  hdop;
            float  ground_speed;
            float  course;
        } gps;

        struct {
            float alt_msl, alt_rel;
            float climb_rate;
        } altitude;

        struct {
            float   voltage, current;
            uint8_t remaining_pct;
        } battery;

        struct {
            uint8_t armed;
            uint8_t flight_mode;
            uint8_t failsafe;
            uint8_t sensor_health;  /* bitmask: bit0=imu, bit1=baro, bit2=mag, bit3=gps */
        } status;

        struct {
            int16_t channels[8];
            uint8_t count;
        } rc;

        struct {
            char     name[17];      /* param name (null-terminated, 16 chars max) */
            float    value;
            uint8_t  type;          /* MAV_PARAM_TYPE (6 = REAL32) */
            uint16_t count;         /* total param count */
            uint16_t index;         /* param index */
        } param_value;

        struct {
            uint16_t servo_us[5];   /* servo1, servo2, servo3, tail_esc, main_esc */
        } servo_output;

        struct {
            int32_t lat;            /* degE7 */
            int32_t lon;            /* degE7 */
            int32_t alt;            /* mm MSL */
        } home_position;

        struct {
            uint16_t count;         /* total mission items */
        } mission_count;

        struct {
            uint16_t seq;
            uint8_t  frame;
            uint16_t command;
            uint8_t  current;
            uint8_t  autocontinue;
            float    param1, param2, param3, param4;
            int32_t  x;            /* lat*1e7 */
            int32_t  y;            /* lon*1e7 */
            float    z;            /* alt */
        } mission_item;

        struct {
            uint8_t result;         /* MAV_MISSION_RESULT */
        } mission_ack;

        struct {
            uint16_t seq;           /* current mission sequence number */
        } mission_current;

        struct {
            uint8_t severity;       /* MAV_SEVERITY */
            char    text[50];
        } statustext;

        struct {
            float accel_x, accel_y, accel_z;    /* m/s^2 */
            float gyro_x, gyro_y, gyro_z;       /* rad/s */
            float temperature;                   /* degC */
        } imu_raw;

        struct {
            uint16_t flags;                 /* ESTIMATOR_STATUS_FLAGS */
            float    vel_ratio;
            float    pos_horiz_ratio;
            float    pos_vert_ratio;
            float    pos_horiz_accuracy;    /* meters */
            float    pos_vert_accuracy;     /* meters */
        } estimator;

        struct {
            float x, y, z;                 /* Gauss */
        } mag_raw;

        struct {
            float pressure;                /* Pa */
            float temperature;             /* degC */
            float altitude;                /* meters */
        } baro_raw;
    } data;
} rpc_telemetry_msg_t;

/* Command message (Core 1 -> Core 0) */
typedef struct {
    uint8_t  msg_type;
    uint32_t timestamp_ms;
    union {
        struct { uint8_t arm; } arm_cmd;
        struct { uint8_t mode; } mode_cmd;
        struct { int16_t channels[8]; } rc_override;
        struct { char name[17]; float value; } param_set;
        struct { char name[17]; int16_t index; } param_request;
        struct {
            uint16_t count;
        } mission_count_cmd;
        struct {
            uint16_t seq;
            uint8_t  frame;
            uint16_t command;
            uint8_t  current;
            uint8_t  autocontinue;
            float    param1, param2, param3, param4;
            int32_t  x;
            int32_t  y;
            float    z;
        } mission_item_cmd;
        struct {
            uint16_t seq;
        } mission_set_current_cmd;
    } data;
} rpc_command_msg_t;
