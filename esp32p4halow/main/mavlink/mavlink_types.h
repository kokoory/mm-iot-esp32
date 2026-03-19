/*
 * Minimal MAVLink v2 Type Definitions
 *
 * Provides core MAVLink v2 types and constants without requiring
 * the full MAVLink generator toolchain.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#define MAVLINK_STX_V2          0xFD
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_NUM_HEADER_BYTES 10
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_NUM_NON_PAYLOAD_BYTES (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)
#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES)

/* System / Component IDs */
#define MAV_SYS_ID             1
#define MAV_COMP_ID_AUTOPILOT  1
#define MAV_COMP_ID_GCS        255

/* Message IDs */
#define MAVLINK_MSG_ID_HEARTBEAT            0
#define MAVLINK_MSG_ID_SYS_STATUS           1
#define MAVLINK_MSG_ID_GPS_RAW_INT          24
#define MAVLINK_MSG_ID_ATTITUDE             30
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33
#define MAVLINK_MSG_ID_RC_CHANNELS          65
#define MAVLINK_MSG_ID_VFR_HUD              74
#define MAVLINK_MSG_ID_COMMAND_LONG         76
#define MAVLINK_MSG_ID_COMMAND_ACK          77
#define MAVLINK_MSG_ID_BATTERY_STATUS       147
#define MAVLINK_MSG_ID_STATUSTEXT           253

/* MAV_TYPE */
#define MAV_TYPE_HELICOPTER    4

/* MAV_AUTOPILOT */
#define MAV_AUTOPILOT_GENERIC  0

/* MAV_MODE_FLAG */
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  1
#define MAV_MODE_FLAG_SAFETY_ARMED         128

/* MAV_STATE */
#define MAV_STATE_UNINIT    0
#define MAV_STATE_BOOT      1
#define MAV_STATE_CALIBRATING 2
#define MAV_STATE_STANDBY   3
#define MAV_STATE_ACTIVE    4
#define MAV_STATE_CRITICAL  5
#define MAV_STATE_EMERGENCY 6

/* MAV_CMD */
#define MAV_CMD_COMPONENT_ARM_DISARM  400
#define MAV_CMD_DO_SET_MODE           176

/* MAV_RESULT */
#define MAV_RESULT_ACCEPTED   0
#define MAV_RESULT_DENIED     1
#define MAV_RESULT_UNSUPPORTED 3
#define MAV_RESULT_FAILED     4

/**
 * MAVLink v2 message structure.
 * The wire format is:
 *   [STX][LEN][INCOMPAT][COMPAT][SEQ][SYSID][COMPID][MSGID(3)][PAYLOAD(LEN)][CRC(2)]
 */
typedef struct {
    uint8_t  magic;             /* STX marker (0xFD for v2) */
    uint8_t  len;               /* payload length */
    uint8_t  incompat_flags;
    uint8_t  compat_flags;
    uint8_t  seq;               /* packet sequence number */
    uint8_t  sysid;             /* system ID */
    uint8_t  compid;            /* component ID */
    uint32_t msgid;             /* 24-bit message ID (stored in 32-bit for convenience) */
    uint8_t  payload[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t checksum;          /* CRC-16/MCRF4XX */
} mavlink_message_t;

/**
 * Parser state machine states.
 */
typedef enum {
    MAVLINK_PARSE_STATE_IDLE = 0,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_INCOMPAT,
    MAVLINK_PARSE_STATE_GOT_COMPAT,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID1,
    MAVLINK_PARSE_STATE_GOT_MSGID2,
    MAVLINK_PARSE_STATE_GOT_MSGID3,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1,
} mavlink_parse_state_t;

/**
 * Parser status structure - maintains state between calls.
 */
typedef struct {
    mavlink_parse_state_t state;
    mavlink_message_t     rxmsg;
    uint8_t               payload_idx;
    uint16_t              crc;
} mavlink_parser_t;

/**
 * Get the CRC extra byte for a given message ID.
 * This is used in the MAVLink CRC calculation for message validation.
 */
uint8_t mavlink_get_crc_extra(uint32_t msgid);
