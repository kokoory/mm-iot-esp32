/*
 * MAVLink UART Telemetry for Pixhawk
 *
 * Bidirectional MAVLink bridge:
 *   Pixhawk <-> UART <-> ESP32-P4 <-> Wi-Fi HaLow <-> Ground Station
 *
 * Supports MAVLink v2 protocol with transparent serial-to-UDP forwarding.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MAVLink UART configuration */
#define MAVLINK_UART_NUM        1           /* UART1 */
#define MAVLINK_UART_TX_PIN     6           /* GPIO6 on P4 40-pin header */
#define MAVLINK_UART_RX_PIN     7           /* GPIO7 on P4 40-pin header */
#define MAVLINK_UART_BAUD       921600      /* Pixhawk TELEM baud rate */

/* Ground station UDP configuration */
#define MAVLINK_GCS_PORT        14550       /* Standard MAVLink GCS port */
#define MAVLINK_LOCAL_PORT      14555       /* Local UDP listen port */
#define MAVLINK_MAX_PACKET_LEN  280         /* MAVLink v2 max packet size */

/* MAVLink system/component IDs for this bridge */
#define MAVLINK_SYS_ID          1           /* System ID (match Pixhawk) */
#define MAVLINK_COMP_ID         191         /* Component ID: onboard computer */

/**
 * MAVLink telemetry statistics
 */
typedef struct {
    uint32_t uart_rx_bytes;         /* Total bytes received from Pixhawk */
    uint32_t uart_tx_bytes;         /* Total bytes sent to Pixhawk */
    uint32_t udp_rx_packets;        /* Packets received from GCS */
    uint32_t udp_tx_packets;        /* Packets sent to GCS */
    uint32_t parse_errors;          /* MAVLink parse errors */
    uint32_t heartbeat_count;       /* Heartbeats received from Pixhawk */
    float uart_rx_rate;             /* UART receive rate (bytes/sec) */
} mavlink_stats_t;

/**
 * Initialize the MAVLink UART telemetry bridge.
 *
 * Sets up:
 * - UART connection to Pixhawk (TX/RX pins, baud rate)
 * - UDP socket for ground station communication
 * - RX/TX processing tasks
 *
 * @return ESP_OK on success.
 */
esp_err_t mavlink_uart_init(void);

/**
 * Set the ground station IP address for UDP forwarding.
 * If not set, the bridge broadcasts to 255.255.255.255.
 *
 * @param ip_str  IP address string (e.g., "192.168.1.100")
 */
void mavlink_set_gcs_ip(const char *ip_str);

/**
 * Get current telemetry statistics.
 *
 * @param stats  Pointer to stats structure to fill.
 */
void mavlink_get_stats(mavlink_stats_t *stats);

/**
 * Check if Pixhawk heartbeat is being received.
 *
 * @return true if heartbeat received within last 3 seconds.
 */
bool mavlink_is_pixhawk_connected(void);

#ifdef __cplusplus
}
#endif
