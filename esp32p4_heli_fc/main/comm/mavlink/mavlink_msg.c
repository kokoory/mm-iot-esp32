/*
 * MAVLink v2 Message Encoding/Decoding - Implementation
 *
 * Full CRC-16/MCRF4XX (X.25) checksum, message encoding for all
 * supported message types, state-machine parser for incoming bytes.
 */
#include "mavlink_msg.h"
#include <string.h>

/* ── Sequence counter (global, incremented per finalized message) ── */
static uint8_t s_mavlink_seq = 0;

/* ── CRC Extra Bytes ──────────────────────────────────────────────
 * These are computed from the MAVLink XML message definitions.
 * Each message ID has a unique CRC extra byte used in checksum
 * computation to detect message definition mismatches.
 */
uint8_t mavlink_get_crc_extra(uint32_t msgid)
{
    switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:           return 50;
    case MAVLINK_MSG_ID_SYS_STATUS:          return 124;
    case MAVLINK_MSG_ID_GPS_RAW_INT:         return 24;
    case MAVLINK_MSG_ID_ATTITUDE:            return 39;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return 104;
    case MAVLINK_MSG_ID_RC_CHANNELS:         return 118;
    case MAVLINK_MSG_ID_VFR_HUD:             return 20;
    case MAVLINK_MSG_ID_COMMAND_LONG:        return 152;
    case MAVLINK_MSG_ID_COMMAND_ACK:         return 143;
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:    return 222;
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: return 124;
    case MAVLINK_MSG_ID_BATTERY_STATUS:      return 154;
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:   return 170;
    case MAVLINK_MSG_ID_STATUSTEXT:          return 83;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:  return 214;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:  return 159;
    case MAVLINK_MSG_ID_PARAM_VALUE:         return 220;
    case MAVLINK_MSG_ID_PARAM_SET:           return 168;
    default:                                 return 0;
    }
}

/* ── CRC-16/MCRF4XX (X.25) ───────────────────────────────────── */

void mavlink_crc_init(uint16_t *crc)
{
    *crc = 0xFFFF;
}

void mavlink_crc_accumulate(uint16_t *crc, uint8_t data)
{
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (tmp << 4);
    *crc = (*crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

static void mavlink_crc_accumulate_buf(uint16_t *crc, const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        mavlink_crc_accumulate(crc, buf[i]);
    }
}

/* ── Helper: put little-endian values into payload ────────────── */

static void put_u8(uint8_t *buf, size_t offset, uint8_t val)
{
    buf[offset] = val;
}

static void put_i8(uint8_t *buf, size_t offset, int8_t val)
{
    buf[offset] = (uint8_t)val;
}

static void put_u16(uint8_t *buf, size_t offset, uint16_t val)
{
    buf[offset + 0] = (uint8_t)(val & 0xFF);
    buf[offset + 1] = (uint8_t)((val >> 8) & 0xFF);
}

static void put_i16(uint8_t *buf, size_t offset, int16_t val)
{
    put_u16(buf, offset, (uint16_t)val);
}

static void put_u32(uint8_t *buf, size_t offset, uint32_t val)
{
    buf[offset + 0] = (uint8_t)(val & 0xFF);
    buf[offset + 1] = (uint8_t)((val >> 8) & 0xFF);
    buf[offset + 2] = (uint8_t)((val >> 16) & 0xFF);
    buf[offset + 3] = (uint8_t)((val >> 24) & 0xFF);
}

static void put_i32(uint8_t *buf, size_t offset, int32_t val)
{
    put_u32(buf, offset, (uint32_t)val);
}

static void put_u64(uint8_t *buf, size_t offset, uint64_t val)
{
    for (int i = 0; i < 8; i++) {
        buf[offset + i] = (uint8_t)((val >> (i * 8)) & 0xFF);
    }
}

static void put_float(uint8_t *buf, size_t offset, float val)
{
    uint32_t tmp;
    memcpy(&tmp, &val, sizeof(tmp));
    put_u32(buf, offset, tmp);
}

/* ── Helper: get little-endian values from payload ────────────── */

static uint16_t get_u16(const uint8_t *buf, size_t offset)
{
    return (uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8);
}

static float get_float(const uint8_t *buf, size_t offset)
{
    uint32_t tmp = (uint32_t)buf[offset]
                 | ((uint32_t)buf[offset + 1] << 8)
                 | ((uint32_t)buf[offset + 2] << 16)
                 | ((uint32_t)buf[offset + 3] << 24);
    float val;
    memcpy(&val, &tmp, sizeof(val));
    return val;
}

static int16_t get_i16(const uint8_t *buf, size_t offset)
{
    return (int16_t)get_u16(buf, offset);
}

static uint8_t get_u8(const uint8_t *buf, size_t offset)
{
    return buf[offset];
}

/* ── Finalize & Serialize ─────────────────────────────────────── */

void mavlink_finalize(mavlink_message_t *msg)
{
    msg->magic = MAVLINK_STX_V2;
    msg->incompat_flags = 0;
    msg->compat_flags = 0;
    msg->seq = s_mavlink_seq++;
    /* sysid, compid, msgid, len, payload must already be set by caller */

    /* Compute CRC over: len, incompat, compat, seq, sysid, compid, msgid(3), payload, crc_extra */
    uint16_t crc;
    mavlink_crc_init(&crc);
    mavlink_crc_accumulate(&crc, msg->len);
    mavlink_crc_accumulate(&crc, msg->incompat_flags);
    mavlink_crc_accumulate(&crc, msg->compat_flags);
    mavlink_crc_accumulate(&crc, msg->seq);
    mavlink_crc_accumulate(&crc, msg->sysid);
    mavlink_crc_accumulate(&crc, msg->compid);
    mavlink_crc_accumulate(&crc, (uint8_t)(msg->msgid & 0xFF));
    mavlink_crc_accumulate(&crc, (uint8_t)((msg->msgid >> 8) & 0xFF));
    mavlink_crc_accumulate(&crc, (uint8_t)((msg->msgid >> 16) & 0xFF));
    mavlink_crc_accumulate_buf(&crc, msg->payload, msg->len);
    mavlink_crc_accumulate(&crc, mavlink_get_crc_extra(msg->msgid));
    msg->checksum = crc;
}

int mavlink_serialize(const mavlink_message_t *msg, uint8_t *buf, size_t buf_len)
{
    size_t total = (size_t)msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    if (buf_len < total) {
        return -1;
    }

    size_t idx = 0;
    buf[idx++] = msg->magic;
    buf[idx++] = msg->len;
    buf[idx++] = msg->incompat_flags;
    buf[idx++] = msg->compat_flags;
    buf[idx++] = msg->seq;
    buf[idx++] = msg->sysid;
    buf[idx++] = msg->compid;
    buf[idx++] = (uint8_t)(msg->msgid & 0xFF);
    buf[idx++] = (uint8_t)((msg->msgid >> 8) & 0xFF);
    buf[idx++] = (uint8_t)((msg->msgid >> 16) & 0xFF);
    memcpy(&buf[idx], msg->payload, msg->len);
    idx += msg->len;
    buf[idx++] = (uint8_t)(msg->checksum & 0xFF);
    buf[idx++] = (uint8_t)((msg->checksum >> 8) & 0xFF);

    return (int)idx;
}

/* ── State Machine Parser ─────────────────────────────────────── */

void mavlink_parser_init(mavlink_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
    parser->state = MAVLINK_PARSE_STATE_IDLE;
}

int mavlink_parse_byte(mavlink_parser_t *parser, uint8_t byte, mavlink_message_t *out_msg)
{
    switch (parser->state) {
    case MAVLINK_PARSE_STATE_IDLE:
        if (byte == MAVLINK_STX_V2) {
            memset(&parser->rxmsg, 0, sizeof(parser->rxmsg));
            parser->rxmsg.magic = byte;
            parser->payload_idx = 0;
            mavlink_crc_init(&parser->crc);
            parser->state = MAVLINK_PARSE_STATE_GOT_STX;
        }
        break;

    case MAVLINK_PARSE_STATE_GOT_STX:
        parser->rxmsg.len = byte;
        mavlink_crc_accumulate(&parser->crc, byte);
        parser->state = MAVLINK_PARSE_STATE_GOT_LENGTH;
        break;

    case MAVLINK_PARSE_STATE_GOT_LENGTH:
        parser->rxmsg.incompat_flags = byte;
        mavlink_crc_accumulate(&parser->crc, byte);
        parser->state = MAVLINK_PARSE_STATE_GOT_INCOMPAT;
        break;

    case MAVLINK_PARSE_STATE_GOT_INCOMPAT:
        parser->rxmsg.compat_flags = byte;
        mavlink_crc_accumulate(&parser->crc, byte);
        parser->state = MAVLINK_PARSE_STATE_GOT_COMPAT;
        break;

    case MAVLINK_PARSE_STATE_GOT_COMPAT:
        parser->rxmsg.seq = byte;
        mavlink_crc_accumulate(&parser->crc, byte);
        parser->state = MAVLINK_PARSE_STATE_GOT_SEQ;
        break;

    case MAVLINK_PARSE_STATE_GOT_SEQ:
        parser->rxmsg.sysid = byte;
        mavlink_crc_accumulate(&parser->crc, byte);
        parser->state = MAVLINK_PARSE_STATE_GOT_SYSID;
        break;

    case MAVLINK_PARSE_STATE_GOT_SYSID:
        parser->rxmsg.compid = byte;
        mavlink_crc_accumulate(&parser->crc, byte);
        parser->state = MAVLINK_PARSE_STATE_GOT_COMPID;
        break;

    case MAVLINK_PARSE_STATE_GOT_COMPID:
        parser->rxmsg.msgid = byte;
        mavlink_crc_accumulate(&parser->crc, byte);
        parser->state = MAVLINK_PARSE_STATE_GOT_MSGID1;
        break;

    case MAVLINK_PARSE_STATE_GOT_MSGID1:
        parser->rxmsg.msgid |= ((uint32_t)byte << 8);
        mavlink_crc_accumulate(&parser->crc, byte);
        parser->state = MAVLINK_PARSE_STATE_GOT_MSGID2;
        break;

    case MAVLINK_PARSE_STATE_GOT_MSGID2:
        parser->rxmsg.msgid |= ((uint32_t)byte << 16);
        mavlink_crc_accumulate(&parser->crc, byte);
        if (parser->rxmsg.len == 0) {
            parser->state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            /* Fall through to CRC extra and checksum */
            mavlink_crc_accumulate(&parser->crc,
                                   mavlink_get_crc_extra(parser->rxmsg.msgid));
            parser->state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        } else {
            parser->state = MAVLINK_PARSE_STATE_GOT_MSGID3;
            parser->payload_idx = 0;
        }
        break;

    case MAVLINK_PARSE_STATE_GOT_MSGID3:
        parser->rxmsg.payload[parser->payload_idx++] = byte;
        mavlink_crc_accumulate(&parser->crc, byte);
        if (parser->payload_idx >= parser->rxmsg.len) {
            /* Accumulate CRC extra byte */
            mavlink_crc_accumulate(&parser->crc,
                                   mavlink_get_crc_extra(parser->rxmsg.msgid));
            parser->state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        }
        break;

    case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
        /* First CRC byte (low) */
        parser->rxmsg.checksum = byte;
        parser->state = MAVLINK_PARSE_STATE_GOT_CRC1;
        break;

    case MAVLINK_PARSE_STATE_GOT_CRC1: {
        /* Second CRC byte (high) */
        parser->rxmsg.checksum |= ((uint16_t)byte << 8);
        parser->state = MAVLINK_PARSE_STATE_IDLE;

        /* Validate CRC */
        if (parser->rxmsg.checksum == parser->crc) {
            memcpy(out_msg, &parser->rxmsg, sizeof(mavlink_message_t));
            return 1;
        }
        /* CRC mismatch - discard */
        break;
    }

    default:
        parser->state = MAVLINK_PARSE_STATE_IDLE;
        break;
    }

    return 0;
}

int mavlink_parse(mavlink_parser_t *parser, const uint8_t *buf, size_t len,
                  mavlink_message_t *out_msg, size_t *consumed)
{
    for (size_t i = 0; i < len; i++) {
        if (mavlink_parse_byte(parser, buf[i], out_msg)) {
            if (consumed) {
                *consumed = i + 1;
            }
            return 1;
        }
    }
    if (consumed) {
        *consumed = len;
    }
    return 0;
}

/* ── Message Encoding: helper to prepare msg header ───────────── */

static void msg_init(mavlink_message_t *msg, uint32_t msgid)
{
    memset(msg, 0, sizeof(*msg));
    msg->sysid = MAV_SYS_ID;
    msg->compid = MAV_COMP_ID_AUTOPILOT;
    msg->msgid = msgid;
}

/* ── Heartbeat (ID 0) ────────────────────────────────────────── *
 * Payload layout (9 bytes):
 *   0-3: custom_mode (uint32)
 *   4:   type (uint8)
 *   5:   autopilot (uint8)
 *   6:   base_mode (uint8)
 *   7:   system_status (uint8)
 *   8:   mavlink_version (uint8) = 3
 */
void mavlink_msg_heartbeat_encode(mavlink_message_t *msg,
                                  uint8_t type, uint8_t autopilot,
                                  uint8_t base_mode, uint32_t custom_mode,
                                  uint8_t system_status)
{
    msg_init(msg, MAVLINK_MSG_ID_HEARTBEAT);
    msg->len = 9;
    put_u32(msg->payload, 0, custom_mode);
    put_u8(msg->payload, 4, type);
    put_u8(msg->payload, 5, autopilot);
    put_u8(msg->payload, 6, base_mode);
    put_u8(msg->payload, 7, system_status);
    put_u8(msg->payload, 8, 3); /* MAVLink version */
    mavlink_finalize(msg);
}

/* ── Attitude (ID 30) ────────────────────────────────────────── *
 * Payload layout (28 bytes):
 *   0-3:   time_boot_ms (uint32)
 *   4-7:   roll (float)
 *   8-11:  pitch (float)
 *   12-15: yaw (float)
 *   16-19: rollspeed (float)
 *   20-23: pitchspeed (float)
 *   24-27: yawspeed (float)
 */
void mavlink_msg_attitude_encode(mavlink_message_t *msg,
                                 uint32_t time_boot_ms,
                                 float roll, float pitch, float yaw,
                                 float rollspeed, float pitchspeed, float yawspeed)
{
    msg_init(msg, MAVLINK_MSG_ID_ATTITUDE);
    msg->len = 28;
    put_u32(msg->payload, 0, time_boot_ms);
    put_float(msg->payload, 4, roll);
    put_float(msg->payload, 8, pitch);
    put_float(msg->payload, 12, yaw);
    put_float(msg->payload, 16, rollspeed);
    put_float(msg->payload, 20, pitchspeed);
    put_float(msg->payload, 24, yawspeed);
    mavlink_finalize(msg);
}

/* ── GPS Raw Int (ID 24) ─────────────────────────────────────── *
 * Payload layout (30 bytes):
 *   0-7:   time_usec (uint64)
 *   8-11:  lat (int32, degE7)
 *   12-15: lon (int32, degE7)
 *   16-19: alt (int32, mm)
 *   20-21: eph (uint16, cm)
 *   22-23: epv (uint16, cm)
 *   24-25: vel (uint16, cm/s)
 *   26-27: cog (uint16, cdeg)
 *   28:    fix_type (uint8)
 *   29:    satellites_visible (uint8)
 */
void mavlink_msg_gps_raw_int_encode(mavlink_message_t *msg,
                                    uint64_t time_usec,
                                    uint8_t fix_type,
                                    int32_t lat, int32_t lon, int32_t alt,
                                    uint16_t eph, uint16_t epv,
                                    uint16_t vel, uint16_t cog,
                                    uint8_t satellites_visible)
{
    msg_init(msg, MAVLINK_MSG_ID_GPS_RAW_INT);
    msg->len = 30;
    put_u64(msg->payload, 0, time_usec);
    put_i32(msg->payload, 8, lat);
    put_i32(msg->payload, 12, lon);
    put_i32(msg->payload, 16, alt);
    put_u16(msg->payload, 20, eph);
    put_u16(msg->payload, 22, epv);
    put_u16(msg->payload, 24, vel);
    put_u16(msg->payload, 26, cog);
    put_u8(msg->payload, 28, fix_type);
    put_u8(msg->payload, 29, satellites_visible);
    mavlink_finalize(msg);
}

/* ── Global Position Int (ID 33) ─────────────────────────────── *
 * Payload layout (28 bytes):
 *   0-3:   time_boot_ms (uint32)
 *   4-7:   lat (int32, degE7)
 *   8-11:  lon (int32, degE7)
 *   12-15: alt (int32, mm MSL)
 *   16-19: relative_alt (int32, mm above home)
 *   20-21: vx (int16, cm/s)
 *   22-23: vy (int16, cm/s)
 *   24-25: vz (int16, cm/s)
 *   26-27: hdg (uint16, cdeg, 0-35999)
 */
void mavlink_msg_global_position_int_encode(mavlink_message_t *msg,
                                            uint32_t time_boot_ms,
                                            int32_t lat, int32_t lon,
                                            int32_t alt, int32_t relative_alt,
                                            int16_t vx, int16_t vy, int16_t vz,
                                            uint16_t hdg)
{
    msg_init(msg, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    msg->len = 28;
    put_u32(msg->payload, 0, time_boot_ms);
    put_i32(msg->payload, 4, lat);
    put_i32(msg->payload, 8, lon);
    put_i32(msg->payload, 12, alt);
    put_i32(msg->payload, 16, relative_alt);
    put_i16(msg->payload, 20, vx);
    put_i16(msg->payload, 22, vy);
    put_i16(msg->payload, 24, vz);
    put_u16(msg->payload, 26, hdg);
    mavlink_finalize(msg);
}

/* ── Sys Status (ID 1) ───────────────────────────────────────── *
 * Payload layout (31 bytes):
 *   0-3:   sensors_present (uint32)
 *   4-7:   sensors_enabled (uint32)
 *   8-11:  sensors_health (uint32)
 *   12-13: load (uint16, permille)
 *   14-15: voltage_battery (uint16, mV)
 *   16-17: current_battery (int16, cA)
 *   18:    battery_remaining (int8, %)
 *   19-20: drop_rate_comm (uint16)
 *   21-22: errors_comm (uint16)
 *   23-24: errors_count1 (uint16)
 *   25-26: errors_count2 (uint16)
 *   27-28: errors_count3 (uint16)
 *   29-30: errors_count4 (uint16)
 */
void mavlink_msg_sys_status_encode(mavlink_message_t *msg,
                                   uint32_t onboard_control_sensors_present,
                                   uint32_t onboard_control_sensors_enabled,
                                   uint32_t onboard_control_sensors_health,
                                   uint16_t load,
                                   uint16_t voltage_battery,
                                   int16_t  current_battery,
                                   int8_t   battery_remaining)
{
    msg_init(msg, MAVLINK_MSG_ID_SYS_STATUS);
    msg->len = 31;
    put_u32(msg->payload, 0, onboard_control_sensors_present);
    put_u32(msg->payload, 4, onboard_control_sensors_enabled);
    put_u32(msg->payload, 8, onboard_control_sensors_health);
    put_u16(msg->payload, 12, load);
    put_u16(msg->payload, 14, voltage_battery);
    put_i16(msg->payload, 16, current_battery);
    put_i8(msg->payload, 18, battery_remaining);
    put_u16(msg->payload, 19, 0); /* drop_rate_comm */
    put_u16(msg->payload, 21, 0); /* errors_comm */
    put_u16(msg->payload, 23, 0); /* errors_count1 */
    put_u16(msg->payload, 25, 0); /* errors_count2 */
    put_u16(msg->payload, 27, 0); /* errors_count3 */
    put_u16(msg->payload, 29, 0); /* errors_count4 */
    mavlink_finalize(msg);
}

/* ── VFR HUD (ID 74) ─────────────────────────────────────────── *
 * Payload layout (20 bytes):
 *   0-3:   airspeed (float, m/s)
 *   4-7:   groundspeed (float, m/s)
 *   8-11:  alt (float, m MSL)
 *   12-15: climb (float, m/s)
 *   16-17: heading (int16, deg 0-360)
 *   18-19: throttle (uint16, %)
 */
void mavlink_msg_vfr_hud_encode(mavlink_message_t *msg,
                                float airspeed, float groundspeed,
                                int16_t heading, uint16_t throttle,
                                float alt, float climb)
{
    msg_init(msg, MAVLINK_MSG_ID_VFR_HUD);
    msg->len = 20;
    put_float(msg->payload, 0, airspeed);
    put_float(msg->payload, 4, groundspeed);
    put_float(msg->payload, 8, alt);
    put_float(msg->payload, 12, climb);
    put_i16(msg->payload, 16, heading);
    put_u16(msg->payload, 18, throttle);
    mavlink_finalize(msg);
}

/* ── Battery Status (ID 147) ─────────────────────────────────── *
 * Payload layout (36 bytes):
 *   0-3:   current_consumed (int32, mAh)
 *   4-7:   energy_consumed (int32, hJ)
 *   8-9:   temperature (int16, cdegC)
 *   10-29: voltages[10] (uint16 each, mV; unused = UINT16_MAX)
 *   30-31: current_battery (int16, cA)
 *   32:    id (uint8)
 *   33:    battery_function (uint8)
 *   34:    type (uint8)
 *   35:    battery_remaining (int8, %)
 */
void mavlink_msg_battery_status_encode(mavlink_message_t *msg,
                                       uint8_t id, uint8_t function,
                                       uint8_t type, int16_t temperature,
                                       uint16_t *voltages,
                                       int16_t current_battery,
                                       int32_t current_consumed,
                                       int32_t energy_consumed,
                                       int8_t battery_remaining)
{
    msg_init(msg, MAVLINK_MSG_ID_BATTERY_STATUS);
    msg->len = 36;
    put_i32(msg->payload, 0, current_consumed);
    put_i32(msg->payload, 4, energy_consumed);
    put_i16(msg->payload, 8, temperature);
    /* 10 voltage cells at offsets 10-29 */
    for (int i = 0; i < 10; i++) {
        if (voltages && i < 10) {
            put_u16(msg->payload, 10 + i * 2, voltages[i]);
        } else {
            put_u16(msg->payload, 10 + i * 2, 0xFFFF); /* unused cell */
        }
    }
    put_i16(msg->payload, 30, current_battery);
    put_u8(msg->payload, 32, id);
    put_u8(msg->payload, 33, function);
    put_u8(msg->payload, 34, type);
    put_i8(msg->payload, 35, battery_remaining);
    mavlink_finalize(msg);
}

/* ── RC Channels (ID 65) ─────────────────────────────────────── *
 * Payload layout (42 bytes):
 *   0-3:   time_boot_ms (uint32)
 *   4-5:   chan1_raw ... (uint16 each, up to 18 channels)
 *   40:    chancount (uint8)
 *   41:    rssi (uint8)
 */
void mavlink_msg_rc_channels_encode(mavlink_message_t *msg,
                                    uint32_t time_boot_ms,
                                    uint8_t chancount,
                                    uint16_t *channels, uint8_t num_channels,
                                    uint8_t rssi)
{
    msg_init(msg, MAVLINK_MSG_ID_RC_CHANNELS);
    msg->len = 42;
    memset(msg->payload, 0, 42);
    put_u32(msg->payload, 0, time_boot_ms);
    /* Channels at offsets 4, 6, 8, ... up to 18 channels */
    for (int i = 0; i < 18 && i < num_channels; i++) {
        put_u16(msg->payload, 4 + i * 2, channels[i]);
    }
    /* Fill unused channels with UINT16_MAX */
    for (int i = num_channels; i < 18; i++) {
        put_u16(msg->payload, 4 + i * 2, 0xFFFF);
    }
    put_u8(msg->payload, 40, chancount);
    put_u8(msg->payload, 41, rssi);
    mavlink_finalize(msg);
}

/* ── Command Long (ID 76) - decode ───────────────────────────── *
 * Payload layout (33 bytes):
 *   0-3:   param1 (float)
 *   4-7:   param2 (float)
 *   8-11:  param3 (float)
 *   12-15: param4 (float)
 *   16-19: param5 (float)
 *   20-23: param6 (float)
 *   24-27: param7 (float)
 *   28-29: command (uint16)
 *   30:    target_system (uint8)
 *   31:    target_component (uint8)
 *   32:    confirmation (uint8)
 */
void mavlink_msg_command_long_decode(const mavlink_message_t *msg,
                                     uint16_t *command,
                                     float *param1, float *param2,
                                     float *param3, float *param4,
                                     float *param5, float *param6,
                                     float *param7,
                                     uint8_t *target_system,
                                     uint8_t *target_component)
{
    if (param1) *param1 = get_float(msg->payload, 0);
    if (param2) *param2 = get_float(msg->payload, 4);
    if (param3) *param3 = get_float(msg->payload, 8);
    if (param4) *param4 = get_float(msg->payload, 12);
    if (param5) *param5 = get_float(msg->payload, 16);
    if (param6) *param6 = get_float(msg->payload, 20);
    if (param7) *param7 = get_float(msg->payload, 24);
    if (command) *command = get_u16(msg->payload, 28);
    if (target_system) *target_system = get_u8(msg->payload, 30);
    if (target_component) *target_component = get_u8(msg->payload, 31);
}

/* ── Command Ack (ID 77) ─────────────────────────────────────── *
 * Payload layout (3 bytes):
 *   0-1: command (uint16)
 *   2:   result (uint8)
 */
void mavlink_msg_command_ack_encode(mavlink_message_t *msg,
                                    uint16_t command, uint8_t result)
{
    msg_init(msg, MAVLINK_MSG_ID_COMMAND_ACK);
    msg->len = 3;
    put_u16(msg->payload, 0, command);
    put_u8(msg->payload, 2, result);
    mavlink_finalize(msg);
}

/* ── Param Value (ID 22) ─────────────────────────────────────── *
 * Payload layout (25 bytes):
 *   0-3:   param_value (float)
 *   4-5:   param_count (uint16)
 *   6-7:   param_index (uint16)
 *   8-23:  param_id (char[16])
 *   24:    param_type (uint8) - MAV_PARAM_TYPE_REAL32 = 9
 */
void mavlink_msg_param_value_encode(mavlink_message_t *msg,
                                    const char *param_id,
                                    float param_value,
                                    uint8_t param_type,
                                    uint16_t param_count,
                                    uint16_t param_index)
{
    msg_init(msg, MAVLINK_MSG_ID_PARAM_VALUE);
    msg->len = 25;
    memset(msg->payload, 0, 25);
    put_float(msg->payload, 0, param_value);
    put_u16(msg->payload, 4, param_count);
    put_u16(msg->payload, 6, param_index);
    if (param_id) {
        size_t len = strlen(param_id);
        if (len > 16) len = 16;
        memcpy(&msg->payload[8], param_id, len);
    }
    put_u8(msg->payload, 24, param_type);
    mavlink_finalize(msg);
}

/* ── Param Set (ID 23) - decode ──────────────────────────────── *
 * Payload layout (23 bytes):
 *   0-3:   param_value (float)
 *   4:     target_system (uint8)
 *   5:     target_component (uint8)
 *   6-21:  param_id (char[16])
 *   22:    param_type (uint8)
 */
void mavlink_msg_param_set_decode(const mavlink_message_t *msg,
                                  char *param_id,
                                  float *param_value,
                                  uint8_t *param_type,
                                  uint8_t *target_system,
                                  uint8_t *target_component)
{
    if (param_value) *param_value = get_float(msg->payload, 0);
    if (target_system) *target_system = get_u8(msg->payload, 4);
    if (target_component) *target_component = get_u8(msg->payload, 5);
    if (param_id) {
        memcpy(param_id, &msg->payload[6], 16);
        param_id[16] = '\0';
    }
    if (param_type) *param_type = get_u8(msg->payload, 22);
}

/* ── Statustext (ID 253) ─────────────────────────────────────── *
 * Payload layout (51 bytes):
 *   0:    severity (uint8) - MAV_SEVERITY enum
 *   1-50: text (char[50])
 */
void mavlink_msg_statustext_encode(mavlink_message_t *msg,
                                   uint8_t severity,
                                   const char *text)
{
    msg_init(msg, MAVLINK_MSG_ID_STATUSTEXT);
    msg->len = 51;
    memset(msg->payload, 0, 51);
    put_u8(msg->payload, 0, severity);
    if (text) {
        size_t len = strlen(text);
        if (len > 50) len = 50;
        memcpy(&msg->payload[1], text, len);
    }
    mavlink_finalize(msg);
}

/* ── RC Channels Override (ID 70) - decode ───────────────────── *
 * Payload layout (18 bytes):
 *   0-1:   chan1_raw (uint16)
 *   2-3:   chan2_raw (uint16)
 *   ...
 *   14-15: chan8_raw (uint16)
 *   16:    target_system (uint8)
 *   17:    target_component (uint8)
 */
void mavlink_msg_rc_channels_override_decode(const mavlink_message_t *msg,
                                              uint16_t chan_out[8],
                                              uint8_t *target_system,
                                              uint8_t *target_component)
{
    for (int i = 0; i < 8; i++) {
        chan_out[i] = get_u16(msg->payload, i * 2);
    }
    if (target_system) *target_system = get_u8(msg->payload, 16);
    if (target_component) *target_component = get_u8(msg->payload, 17);
}

/* ── Servo Output Raw (ID 36) ────────────────────────────────── *
 * Payload layout (21 bytes):
 *   0-3:   time_usec (uint32)
 *   4-5:   servo1_raw (uint16)
 *   6-7:   servo2_raw (uint16)
 *   ...
 *   18-19: servo8_raw (uint16)
 *   20:    port (uint8)
 */
void mavlink_msg_servo_output_raw_encode(mavlink_message_t *msg,
                                         uint32_t time_usec,
                                         uint8_t port,
                                         uint16_t servo[8])
{
    msg_init(msg, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW);
    msg->len = 21;
    memset(msg->payload, 0, 21);
    put_u32(msg->payload, 0, time_usec);
    for (int i = 0; i < 8; i++) {
        put_u16(msg->payload, 4 + i * 2, servo[i]);
    }
    put_u8(msg->payload, 20, port);
    mavlink_finalize(msg);
}

/* ── Named Value Float (ID 251) ──────────────────────────────── *
 * Payload layout (18 bytes):
 *   0-3:   time_boot_ms (uint32)
 *   4-7:   value (float)
 *   8-17:  name (char[10])
 */
void mavlink_msg_named_value_float_encode(mavlink_message_t *msg,
                                          uint32_t time_boot_ms,
                                          const char *name,
                                          float value)
{
    msg_init(msg, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
    msg->len = 18;
    memset(msg->payload, 0, 18);
    put_u32(msg->payload, 0, time_boot_ms);
    put_float(msg->payload, 4, value);
    if (name) {
        size_t len = strlen(name);
        if (len > 10) len = 10;
        memcpy(&msg->payload[8], name, len);
    }
    mavlink_finalize(msg);
}

/* ── Param Request Read (ID 20) - decode ─────────────────────── *
 * Payload layout (20 bytes):
 *   0-1:   param_index (int16)
 *   2:     target_system (uint8)
 *   3:     target_component (uint8)
 *   4-19:  param_id (char[16])
 */
void mavlink_msg_param_request_read_decode(const mavlink_message_t *msg,
                                           char *param_id,
                                           int16_t *param_index,
                                           uint8_t *target_system,
                                           uint8_t *target_component)
{
    if (param_index) *param_index = get_i16(msg->payload, 0);
    if (target_system) *target_system = get_u8(msg->payload, 2);
    if (target_component) *target_component = get_u8(msg->payload, 3);
    if (param_id) {
        memcpy(param_id, &msg->payload[4], 16);
        param_id[16] = '\0';
    }
}
