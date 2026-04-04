/*
 * GCS Bridge - UDP network communication with Ground Control Station
 *
 * Manages a UDP socket for MAVLink communication with the GCS.
 * Auto-detects GCS IP address from the first received packet.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define GCS_BRIDGE_UDP_PORT     14550
#define GCS_BRIDGE_MAX_PKT_LEN 300

typedef enum {
    GCS_BRIDGE_DISCONNECTED = 0,
    GCS_BRIDGE_WAITING_FOR_GCS,
    GCS_BRIDGE_CONNECTED,
} gcs_bridge_status_t;

/**
 * Initialize the GCS bridge. Creates a UDP socket bound to the
 * MAVLink port. Must be called after network is available.
 * Returns 0 on success, -1 on failure.
 */
int gcs_bridge_init(void);

/**
 * Send a buffer of bytes to the GCS via UDP.
 * If no GCS has been detected yet, sends as broadcast.
 * Returns number of bytes sent, or -1 on error.
 */
int gcs_bridge_send(const uint8_t *buf, size_t len);

/**
 * Receive bytes from GCS via UDP.
 * Blocks up to timeout_ms milliseconds.
 * On first received packet, stores the GCS IP for future sends.
 * Returns number of bytes received, 0 on timeout, -1 on error.
 */
int gcs_bridge_recv(uint8_t *buf, size_t max_len, uint32_t timeout_ms);

/**
 * Get current GCS bridge connection status.
 */
gcs_bridge_status_t gcs_bridge_get_status(void);

/**
 * Get the detected GCS IP address.
 * Returns the IP address in network byte order, or 0 if not known.
 */
uint32_t gcs_bridge_get_ip(void);

/**
 * Shutdown and cleanup the GCS bridge.
 */
void gcs_bridge_deinit(void);
