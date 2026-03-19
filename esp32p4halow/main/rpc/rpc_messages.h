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
};

/* Message types: Core 1 -> Core 0 (commands) */
enum {
    RPC_CMD_ARM         = 0x80,
    RPC_CMD_DISARM      = 0x81,
    RPC_CMD_SET_MODE    = 0x82,
    RPC_CMD_RC_OVERRIDE = 0x83,
    RPC_CMD_PARAM_SET   = 0x84,
    RPC_CMD_REBOOT      = 0x85,
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
        struct { uint16_t param_id; float value; } param_set;
    } data;
} rpc_command_msg_t;
