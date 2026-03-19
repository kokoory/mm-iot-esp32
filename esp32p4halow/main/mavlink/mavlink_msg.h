/*
 * MAVLink v2 Message Encoding/Decoding
 *
 * Provides functions to encode and decode MAVLink v2 messages,
 * including full CRC-16/MCRF4XX checksum computation.
 */
#pragma once
#include "mavlink_types.h"

/**
 * Initialize a MAVLink parser state machine.
 */
void mavlink_parser_init(mavlink_parser_t *parser);

/* ── CRC ─────────────────────────────────────────────────────── */

/** Accumulate one byte into CRC-16/MCRF4XX (X.25). */
void mavlink_crc_accumulate(uint16_t *crc, uint8_t data);

/** Initialize CRC to 0xFFFF. */
void mavlink_crc_init(uint16_t *crc);

/* ── Message Encoding ────────────────────────────────────────── */

/**
 * Finalize a MAVLink message: set header fields, compute CRC.
 * The caller must have already set msgid, sysid, compid, and filled payload.
 * len must be set to the payload length before calling.
 */
void mavlink_finalize(mavlink_message_t *msg);

/**
 * Serialize a finalized MAVLink message to a byte buffer.
 * Returns the number of bytes written, or -1 if buf is too small.
 * buf must be at least (msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES) bytes.
 */
int mavlink_serialize(const mavlink_message_t *msg, uint8_t *buf, size_t buf_len);

/**
 * Parse incoming bytes through a state machine.
 * Call once per byte. Returns 1 when a complete valid message is available
 * in *out_msg, 0 otherwise.
 */
int mavlink_parse_byte(mavlink_parser_t *parser, uint8_t byte, mavlink_message_t *out_msg);

/**
 * Parse a buffer of bytes. Returns 1 if a complete message was found,
 * and sets *consumed to the number of bytes consumed from buf.
 * May need to be called repeatedly to parse all messages in a buffer.
 */
int mavlink_parse(mavlink_parser_t *parser, const uint8_t *buf, size_t len,
                  mavlink_message_t *out_msg, size_t *consumed);

/* ── Heartbeat (ID 0) ────────────────────────────────────────── */

void mavlink_msg_heartbeat_encode(mavlink_message_t *msg,
                                  uint8_t type, uint8_t autopilot,
                                  uint8_t base_mode, uint32_t custom_mode,
                                  uint8_t system_status);

/* ── Attitude (ID 30) ────────────────────────────────────────── */

void mavlink_msg_attitude_encode(mavlink_message_t *msg,
                                 uint32_t time_boot_ms,
                                 float roll, float pitch, float yaw,
                                 float rollspeed, float pitchspeed, float yawspeed);

/* ── GPS Raw Int (ID 24) ─────────────────────────────────────── */

void mavlink_msg_gps_raw_int_encode(mavlink_message_t *msg,
                                    uint64_t time_usec,
                                    uint8_t fix_type,
                                    int32_t lat, int32_t lon, int32_t alt,
                                    uint16_t eph, uint16_t epv,
                                    uint16_t vel, uint16_t cog,
                                    uint8_t satellites_visible);

/* ── Global Position Int (ID 33) ─────────────────────────────── */

void mavlink_msg_global_position_int_encode(mavlink_message_t *msg,
                                            uint32_t time_boot_ms,
                                            int32_t lat, int32_t lon,
                                            int32_t alt, int32_t relative_alt,
                                            int16_t vx, int16_t vy, int16_t vz,
                                            uint16_t hdg);

/* ── Sys Status (ID 1) ───────────────────────────────────────── */

void mavlink_msg_sys_status_encode(mavlink_message_t *msg,
                                   uint32_t onboard_control_sensors_present,
                                   uint32_t onboard_control_sensors_enabled,
                                   uint32_t onboard_control_sensors_health,
                                   uint16_t load,
                                   uint16_t voltage_battery,
                                   int16_t  current_battery,
                                   int8_t   battery_remaining);

/* ── VFR HUD (ID 74) ─────────────────────────────────────────── */

void mavlink_msg_vfr_hud_encode(mavlink_message_t *msg,
                                float airspeed, float groundspeed,
                                int16_t heading, uint16_t throttle,
                                float alt, float climb);

/* ── Battery Status (ID 147) ─────────────────────────────────── */

void mavlink_msg_battery_status_encode(mavlink_message_t *msg,
                                       uint8_t id, uint8_t function,
                                       uint8_t type, int16_t temperature,
                                       uint16_t *voltages,
                                       int16_t current_battery,
                                       int32_t current_consumed,
                                       int32_t energy_consumed,
                                       int8_t battery_remaining);

/* ── RC Channels (ID 65) ─────────────────────────────────────── */

void mavlink_msg_rc_channels_encode(mavlink_message_t *msg,
                                    uint32_t time_boot_ms,
                                    uint8_t chancount,
                                    uint16_t *channels, uint8_t num_channels,
                                    uint8_t rssi);

/* ── Command Long (ID 76) - decode ───────────────────────────── */

void mavlink_msg_command_long_decode(const mavlink_message_t *msg,
                                     uint16_t *command,
                                     float *param1, float *param2,
                                     float *param3, float *param4,
                                     float *param5, float *param6,
                                     float *param7,
                                     uint8_t *target_system,
                                     uint8_t *target_component);

/* ── Command Ack (ID 77) ─────────────────────────────────────── */

void mavlink_msg_command_ack_encode(mavlink_message_t *msg,
                                    uint16_t command, uint8_t result);
