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

/* ── Param Value (ID 22) ─────────────────────────────────────── */

void mavlink_msg_param_value_encode(mavlink_message_t *msg,
                                    const char *param_id,
                                    float param_value,
                                    uint8_t param_type,
                                    uint16_t param_count,
                                    uint16_t param_index);

/* ── Param Set (ID 23) - decode ──────────────────────────────── */

void mavlink_msg_param_set_decode(const mavlink_message_t *msg,
                                  char *param_id,
                                  float *param_value,
                                  uint8_t *param_type,
                                  uint8_t *target_system,
                                  uint8_t *target_component);

/* ── Param Request Read (ID 20) - decode ─────────────────────── */

void mavlink_msg_param_request_read_decode(const mavlink_message_t *msg,
                                           char *param_id,
                                           int16_t *param_index,
                                           uint8_t *target_system,
                                           uint8_t *target_component);

/* ── Statustext (ID 253) ─────────────────────────────────────── */

void mavlink_msg_statustext_encode(mavlink_message_t *msg,
                                   uint8_t severity,
                                   const char *text);

/* ── RC Channels Override (ID 70) - decode ───────────────────── */

void mavlink_msg_rc_channels_override_decode(const mavlink_message_t *msg,
                                              uint16_t chan_out[8],
                                              uint8_t *target_system,
                                              uint8_t *target_component);

/* ── Servo Output Raw (ID 36) ────────────────────────────────── */

void mavlink_msg_servo_output_raw_encode(mavlink_message_t *msg,
                                         uint32_t time_usec,
                                         uint8_t port,
                                         uint16_t servo[8]);

/* ── Named Value Float (ID 251) ──────────────────────────────── */

void mavlink_msg_named_value_float_encode(mavlink_message_t *msg,
                                          uint32_t time_boot_ms,
                                          const char *name,
                                          float value);

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

/* ── Home Position (ID 242) ──────────────────────────────────── */

void mavlink_msg_home_position_encode(mavlink_message_t *msg,
                                      int32_t lat, int32_t lon, int32_t alt,
                                      float x, float y, float z,
                                      const float q[4],
                                      float approach_x, float approach_y, float approach_z,
                                      uint64_t time_usec);

/* ── Extended Sys State (ID 245) ─────────────────────────────── */

void mavlink_msg_extended_sys_state_encode(mavlink_message_t *msg,
                                           uint8_t vtol_state,
                                           uint8_t landed_state);

/* ── Autopilot Version (ID 148) ──────────────────────────────── */

void mavlink_msg_autopilot_version_encode(mavlink_message_t *msg,
                                          uint64_t capabilities,
                                          uint32_t flight_sw_version,
                                          uint32_t middleware_sw_version,
                                          uint32_t os_sw_version,
                                          uint32_t board_version,
                                          const uint8_t flight_custom_version[8],
                                          uint64_t uid);

/* ── Mission Count (ID 44) ───────────────────────────────────── */

void mavlink_msg_mission_count_encode(mavlink_message_t *msg,
                                      uint8_t target_system, uint8_t target_component,
                                      uint16_t count, uint8_t mission_type);

void mavlink_msg_mission_count_decode(const mavlink_message_t *msg,
                                      uint16_t *count,
                                      uint8_t *target_system, uint8_t *target_component,
                                      uint8_t *mission_type);

/* ── Mission Request Int (ID 51) ─────────────────────────────── */

void mavlink_msg_mission_request_int_encode(mavlink_message_t *msg,
                                            uint8_t target_system, uint8_t target_component,
                                            uint16_t seq, uint8_t mission_type);

void mavlink_msg_mission_request_int_decode(const mavlink_message_t *msg,
                                            uint16_t *seq,
                                            uint8_t *target_system, uint8_t *target_component,
                                            uint8_t *mission_type);

/* ── Mission Item Int (ID 73) ────────────────────────────────── */

typedef struct {
    uint16_t seq;
    uint8_t  frame;
    uint16_t command;
    uint8_t  current;
    uint8_t  autocontinue;
    float    param1, param2, param3, param4;
    int32_t  x;  /* lat * 1e7 */
    int32_t  y;  /* lon * 1e7 */
    float    z;  /* alt */
    uint8_t  mission_type;
} mavlink_mission_item_int_t;

void mavlink_msg_mission_item_int_encode(mavlink_message_t *msg,
                                         uint8_t target_system, uint8_t target_component,
                                         const mavlink_mission_item_int_t *item);

void mavlink_msg_mission_item_int_decode(const mavlink_message_t *msg,
                                         mavlink_mission_item_int_t *item,
                                         uint8_t *target_system, uint8_t *target_component);

/* ── Mission Ack (ID 47) ─────────────────────────────────────── */

void mavlink_msg_mission_ack_encode(mavlink_message_t *msg,
                                    uint8_t target_system, uint8_t target_component,
                                    uint8_t type, uint8_t mission_type);

/* ── Mission Request List (ID 43) - decode ───────────────────── */

void mavlink_msg_mission_request_list_decode(const mavlink_message_t *msg,
                                             uint8_t *target_system, uint8_t *target_component,
                                             uint8_t *mission_type);

/* ── Mission Clear All (ID 45) - decode ──────────────────────── */

void mavlink_msg_mission_clear_all_decode(const mavlink_message_t *msg,
                                          uint8_t *target_system, uint8_t *target_component,
                                          uint8_t *mission_type);

/* ── Mission Set Current (ID 41) - decode ────────────────────── */

void mavlink_msg_mission_set_current_decode(const mavlink_message_t *msg,
                                            uint16_t *seq,
                                            uint8_t *target_system, uint8_t *target_component);

/* ── Mission Current (ID 42) ─────────────────────────────────── */

void mavlink_msg_mission_current_encode(mavlink_message_t *msg, uint16_t seq);

/* ── Vibration (ID 241) ──────────────────────────────────────── */

void mavlink_msg_vibration_encode(mavlink_message_t *msg,
                                  uint64_t time_usec,
                                  float vibration_x, float vibration_y, float vibration_z,
                                  uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2);

/* ── Estimator Status (ID 230) ───────────────────────────────── */

void mavlink_msg_estimator_status_encode(mavlink_message_t *msg,
                                         uint64_t time_usec,
                                         uint16_t flags,
                                         float vel_ratio, float pos_horiz_ratio,
                                         float pos_vert_ratio, float mag_ratio,
                                         float hagl_ratio, float tas_ratio,
                                         float pos_horiz_accuracy, float pos_vert_accuracy);

/* ── HIGHRES_IMU (ID 105) ────────────────────────────────────── */

void mavlink_msg_highres_imu_encode(mavlink_message_t *msg,
                                    uint64_t time_usec,
                                    float xacc, float yacc, float zacc,
                                    float xgyro, float ygyro, float zgyro,
                                    float xmag, float ymag, float zmag,
                                    float abs_pressure, float diff_pressure,
                                    float pressure_alt, float temperature,
                                    uint16_t fields_updated);

/* ── Local Position NED (ID 32) ──────────────────────────────── */

void mavlink_msg_local_position_ned_encode(mavlink_message_t *msg,
                                           uint32_t time_boot_ms,
                                           float x, float y, float z,
                                           float vx, float vy, float vz);

/* ── File Transfer Protocol (ID 110) ─────────────────────────── */

void mavlink_msg_file_transfer_protocol_encode(mavlink_message_t *msg,
                                               uint8_t target_network,
                                               uint8_t target_system,
                                               uint8_t target_component,
                                               const uint8_t *payload, uint8_t payload_len);

void mavlink_msg_file_transfer_protocol_decode(const mavlink_message_t *msg,
                                               uint8_t *target_network,
                                               uint8_t *target_system,
                                               uint8_t *target_component,
                                               uint8_t *payload, uint8_t *payload_len);
