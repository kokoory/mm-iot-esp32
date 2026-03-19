/*
 * MAVLink UART Telemetry Bridge for Pixhawk
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Transparent bidirectional MAVLink bridge:
 *   Pixhawk UART <-> ESP32-P4 <-> Wi-Fi HaLow UDP <-> Ground Station
 *
 * This implements a lightweight MAVLink v2 frame parser for transparent
 * forwarding. It does NOT require the full MAVLink C library - it only
 * needs to detect frame boundaries and forward complete frames.
 *
 * MAVLink v2 frame format:
 *   [0xFD] [len] [incompat] [compat] [seq] [sysid] [compid]
 *   [msgid_low] [msgid_mid] [msgid_high] [payload...] [crc_low] [crc_high]
 *   Optional: [signature 13 bytes]
 *
 * Total overhead: 12 bytes header + 2 bytes CRC = 14 bytes minimum
 * With signature: +13 bytes = 27 bytes overhead
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "mavlink_uart.h"

static const char *TAG = "mavlink";

/* MAVLink v2 constants */
#define MAVLINK_V2_MAGIC        0xFD
#define MAVLINK_V2_HEADER_LEN   10      /* Header bytes before payload */
#define MAVLINK_V2_CRC_LEN      2
#define MAVLINK_V2_SIG_LEN      13
#define MAVLINK_V2_INCOMPAT_FLAG_SIGNED  0x01

/* MAVLink message IDs we care about */
#define MAVLINK_MSG_ID_HEARTBEAT    0

/* UART buffer sizes */
#define UART_RX_BUF_SIZE        2048
#define UART_TX_BUF_SIZE        1024

/* Module state */
static struct {
    bool initialized;

    /* UART */
    int uart_num;

    /* UDP socket */
    int udp_sock;
    struct sockaddr_in gcs_addr;
    bool gcs_addr_set;

    /* Pixhawk connection tracking */
    int64_t last_heartbeat_time;

    /* Statistics */
    mavlink_stats_t stats;
    int64_t stats_start_time;
    uint32_t stats_rx_bytes;

    /* Tasks */
    TaskHandle_t uart_rx_task;
    TaskHandle_t udp_rx_task;
} s_mav = {0};

/* ========== MAVLink v2 Frame Parser ========== */

typedef enum {
    MAV_PARSE_IDLE,
    MAV_PARSE_GOT_MAGIC,
    MAV_PARSE_GOT_LEN,
    MAV_PARSE_IN_FRAME,
} mav_parse_state_t;

typedef struct {
    mav_parse_state_t state;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t idx;
    uint8_t payload_len;
    uint8_t incompat_flags;
    uint16_t expected_len;
} mav_parser_t;

static void mav_parser_init(mav_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = MAV_PARSE_IDLE;
}

/**
 * Feed one byte to the MAVLink v2 parser.
 *
 * @return Frame length if complete frame detected, 0 otherwise.
 *         Complete frame data is in parser->buf[0..return_value-1]
 */
static uint16_t mav_parser_feed(mav_parser_t *p, uint8_t byte)
{
    switch (p->state) {
    case MAV_PARSE_IDLE:
        if (byte == MAVLINK_V2_MAGIC) {
            p->buf[0] = byte;
            p->idx = 1;
            p->state = MAV_PARSE_GOT_MAGIC;
        }
        break;

    case MAV_PARSE_GOT_MAGIC:
        p->buf[p->idx++] = byte;
        p->payload_len = byte;
        p->state = MAV_PARSE_GOT_LEN;
        break;

    case MAV_PARSE_GOT_LEN:
        p->buf[p->idx++] = byte;
        p->incompat_flags = byte;
        /* Calculate expected total frame length */
        p->expected_len = MAVLINK_V2_HEADER_LEN + p->payload_len + MAVLINK_V2_CRC_LEN;
        if (p->incompat_flags & MAVLINK_V2_INCOMPAT_FLAG_SIGNED) {
            p->expected_len += MAVLINK_V2_SIG_LEN;
        }
        if (p->expected_len > MAVLINK_MAX_PACKET_LEN) {
            /* Invalid frame, reset */
            p->state = MAV_PARSE_IDLE;
            s_mav.stats.parse_errors++;
            break;
        }
        p->state = MAV_PARSE_IN_FRAME;
        break;

    case MAV_PARSE_IN_FRAME:
        p->buf[p->idx++] = byte;
        if (p->idx >= p->expected_len) {
            /* Complete frame received */
            uint16_t frame_len = p->idx;
            p->state = MAV_PARSE_IDLE;
            p->idx = 0;
            return frame_len;
        }
        break;
    }

    return 0;
}

/**
 * Check if a complete MAVLink frame is a heartbeat message.
 */
static bool mav_is_heartbeat(const uint8_t *frame, uint16_t len)
{
    if (len < MAVLINK_V2_HEADER_LEN + MAVLINK_V2_CRC_LEN) return false;
    if (frame[0] != MAVLINK_V2_MAGIC) return false;

    /* Message ID is bytes 7-9 (little-endian 24-bit) */
    uint32_t msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16);
    return (msgid == MAVLINK_MSG_ID_HEARTBEAT);
}

/* ========== UDP Socket Setup ========== */

static esp_err_t udp_socket_init(void)
{
    s_mav.udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_mav.udp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return ESP_FAIL;
    }

    /* Bind to local port */
    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(MAVLINK_LOCAL_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(s_mav.udp_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed: errno %d", errno);
        close(s_mav.udp_sock);
        s_mav.udp_sock = -1;
        return ESP_FAIL;
    }

    /* Enable broadcast */
    int broadcast = 1;
    setsockopt(s_mav.udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    /* Set default GCS address (broadcast) */
    if (!s_mav.gcs_addr_set) {
        s_mav.gcs_addr.sin_family = AF_INET;
        s_mav.gcs_addr.sin_port = htons(MAVLINK_GCS_PORT);
        s_mav.gcs_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    }

    ESP_LOGI(TAG, "UDP socket ready (local:%d, GCS:%d)",
             MAVLINK_LOCAL_PORT, MAVLINK_GCS_PORT);
    return ESP_OK;
}

/* ========== UART -> UDP Task (Pixhawk -> Ground) ========== */

static void uart_rx_task(void *arg)
{
    mav_parser_t parser;
    mav_parser_init(&parser);

    uint8_t rx_byte;

    ESP_LOGI(TAG, "UART RX task started (UART%d, %d baud)",
             MAVLINK_UART_NUM, MAVLINK_UART_BAUD);

    while (1) {
        /* Read one byte at a time for frame parsing */
        int len = uart_read_bytes(s_mav.uart_num, &rx_byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        s_mav.stats.uart_rx_bytes++;
        s_mav.stats_rx_bytes++;

        /* Feed to MAVLink parser */
        uint16_t frame_len = mav_parser_feed(&parser, rx_byte);
        if (frame_len > 0) {
            /* Complete MAVLink frame - forward to GCS via UDP */
            int sent = sendto(s_mav.udp_sock, parser.buf, frame_len, 0,
                            (struct sockaddr *)&s_mav.gcs_addr,
                            sizeof(s_mav.gcs_addr));
            if (sent > 0) {
                s_mav.stats.udp_tx_packets++;
            }

            /* Track heartbeats */
            if (mav_is_heartbeat(parser.buf, frame_len)) {
                s_mav.last_heartbeat_time = esp_timer_get_time();
                s_mav.stats.heartbeat_count++;
            }
        }

        /* Update RX rate every second */
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - s_mav.stats_start_time;
        if (elapsed > 1000000) {
            s_mav.stats.uart_rx_rate = (float)s_mav.stats_rx_bytes * 1000000.0f / (float)elapsed;
            s_mav.stats_rx_bytes = 0;
            s_mav.stats_start_time = now;
        }
    }
}

/* ========== UDP -> UART Task (Ground -> Pixhawk) ========== */

static void udp_rx_task(void *arg)
{
    uint8_t rx_buf[MAVLINK_MAX_PACKET_LEN];
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);

    ESP_LOGI(TAG, "UDP RX task started (port %d)", MAVLINK_LOCAL_PORT);

    while (1) {
        int len = recvfrom(s_mav.udp_sock, rx_buf, sizeof(rx_buf), 0,
                          (struct sockaddr *)&src_addr, &addr_len);
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        s_mav.stats.udp_rx_packets++;

        /* Auto-detect GCS address from first received packet */
        if (!s_mav.gcs_addr_set) {
            s_mav.gcs_addr.sin_addr = src_addr.sin_addr;
            s_mav.gcs_addr.sin_port = src_addr.sin_port;
            s_mav.gcs_addr_set = true;
            char addr_str[16];
            inet_ntoa_r(src_addr.sin_addr, addr_str, sizeof(addr_str));
            ESP_LOGI(TAG, "GCS auto-detected: %s:%d", addr_str,
                     ntohs(src_addr.sin_port));
        }

        /* Forward to Pixhawk via UART */
        int written = uart_write_bytes(s_mav.uart_num, rx_buf, len);
        if (written > 0) {
            s_mav.stats.uart_tx_bytes += written;
        }
    }
}

/* ========== Public API ========== */

esp_err_t mavlink_uart_init(void)
{
    if (s_mav.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing MAVLink UART bridge");
    ESP_LOGI(TAG, "  UART%d: TX=GPIO%d, RX=GPIO%d, Baud=%d",
             MAVLINK_UART_NUM, MAVLINK_UART_TX_PIN, MAVLINK_UART_RX_PIN,
             MAVLINK_UART_BAUD);
    ESP_LOGI(TAG, "  UDP: local=%d, GCS=%d",
             MAVLINK_LOCAL_PORT, MAVLINK_GCS_PORT);

    /* Configure UART */
    uart_config_t uart_config = {
        .baud_rate = MAVLINK_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    s_mav.uart_num = MAVLINK_UART_NUM;

    esp_err_t ret = uart_driver_install(s_mav.uart_num, UART_RX_BUF_SIZE,
                                         UART_TX_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(s_mav.uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(s_mav.uart_num, MAVLINK_UART_TX_PIN, MAVLINK_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART initialized");

    /* Initialize UDP socket */
    ret = udp_socket_init();
    if (ret != ESP_OK) {
        return ret;
    }

    s_mav.stats_start_time = esp_timer_get_time();

    /* Start RX/TX tasks */
    xTaskCreatePinnedToCore(uart_rx_task, "mav_uart_rx", 4096, NULL, 6,
                            &s_mav.uart_rx_task, 0);
    xTaskCreatePinnedToCore(udp_rx_task, "mav_udp_rx", 4096, NULL, 6,
                            &s_mav.udp_rx_task, 0);

    s_mav.initialized = true;
    ESP_LOGI(TAG, "MAVLink UART bridge initialized");

    return ESP_OK;
}

void mavlink_set_gcs_ip(const char *ip_str)
{
    s_mav.gcs_addr.sin_family = AF_INET;
    s_mav.gcs_addr.sin_port = htons(MAVLINK_GCS_PORT);
    inet_aton(ip_str, &s_mav.gcs_addr.sin_addr);
    s_mav.gcs_addr_set = true;
    ESP_LOGI(TAG, "GCS address set to %s:%d", ip_str, MAVLINK_GCS_PORT);
}

void mavlink_get_stats(mavlink_stats_t *stats)
{
    if (stats) {
        memcpy(stats, &s_mav.stats, sizeof(mavlink_stats_t));
    }
}

bool mavlink_is_pixhawk_connected(void)
{
    if (s_mav.last_heartbeat_time == 0) return false;
    int64_t elapsed = esp_timer_get_time() - s_mav.last_heartbeat_time;
    return (elapsed < 3000000);  /* 3 second timeout */
}
