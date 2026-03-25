/*
 * ESP32-P4 + Wi-Fi HaLow Drone Telemetry & Camera System
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Integrates three subsystems:
 *   1. Wi-Fi HaLow (Wio-WM6180) - Long-range sub-GHz wireless link
 *   2. MIPI-CSI Camera + H.264  - Hardware-encoded video streaming
 *   3. MAVLink UART (Pixhawk)   - Bidirectional telemetry bridge
 *
 * Hardware:
 *   - Waveshare ESP32-P4-WIFI6 board
 *   - Wio-WM6180 Wi-Fi HaLow module (SPI)
 *   - MIPI-CSI camera (OV5647 / SC2336)
 *   - Pixhawk flight controller (UART TELEM port)
 *
 * Network Topology:
 *   Pixhawk ←UART→ ESP32-P4 ←SPI→ WM6180 ~~~HaLow~~~ AP ←→ Ground Station
 *                      ↑
 *                   MIPI-CSI
 *                   Camera
 *
 * Endpoints:
 *   http://<ip>/        - MJPEG camera stream (HW JPEG encoded)
 *   http://<ip>/h264    - Raw H.264 stream (for GCS decoding)
 *   http://<ip>/status  - JSON system status
 *   UDP 14550           - MAVLink telemetry (GCS port)
 *   UDP 14555           - MAVLink telemetry (local listen)
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_event.h"

#include "mmosal.h"
#include "mmwlan.h"
#include "mmipal.h"

#include "mm_app_common.h"
#include "camera_h264.h"
#include "mavlink_uart.h"

static const char *TAG = "p4_halow_drone";

/**
 * Print system status summary to console.
 */
static void print_status(void)
{
    mavlink_stats_t mav_stats;
    mavlink_get_stats(&mav_stats);

    printf("\n--- System Status ---\n");
    printf("Pixhawk: %s (heartbeats: %lu)\n",
           mavlink_is_pixhawk_connected() ? "CONNECTED" : "DISCONNECTED",
           (unsigned long)mav_stats.heartbeat_count);
    printf("MAVLink: UART RX=%lu TX=%lu bytes, UDP RX=%lu TX=%lu pkts\n",
           (unsigned long)mav_stats.uart_rx_bytes,
           (unsigned long)mav_stats.uart_tx_bytes,
           (unsigned long)mav_stats.udp_rx_packets,
           (unsigned long)mav_stats.udp_tx_packets);
    printf("UART RX rate: %.0f bytes/sec\n", mav_stats.uart_rx_rate);
    printf("Camera FPS: %.1f\n", camera_get_fps());
    printf("Free heap: %lu bytes (PSRAM: %lu bytes)\n",
           (unsigned long)esp_get_free_heap_size(),
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    printf("Uptime: %lld sec\n", esp_timer_get_time() / 1000000LL);
    printf("---------------------\n\n");
}

/**
 * Main entry point.
 *
 * Initialization order:
 *   1. Wi-Fi HaLow (SPI) - must be first, provides network connectivity
 *   2. MAVLink UART       - needs network for UDP forwarding
 *   3. Camera + H.264     - needs network for HTTP streaming
 */
void app_main(void)
{
    esp_err_t err;

    printf("\n\n");
    printf("==============================================\n");
    printf("  ESP32-P4 HaLow Drone System\n");
    printf("  Camera + MAVLink + Wi-Fi HaLow\n");
    printf("  Built " __DATE__ " " __TIME__ "\n");
    printf("==============================================\n\n");

    /* === Phase 1: Wi-Fi HaLow === */
    ESP_LOGI(TAG, "Phase 1: Initializing Wi-Fi HaLow...");
    app_wlan_init();
    app_wlan_start();
    ESP_LOGI(TAG, "Wi-Fi HaLow connected");

    esp_event_loop_create_default();

    /* === Phase 2: MAVLink UART Bridge === */
    ESP_LOGI(TAG, "Phase 2: Initializing MAVLink UART bridge...");
    err = mavlink_uart_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAVLink init failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Continuing without MAVLink - check UART wiring to Pixhawk");
    } else {
        ESP_LOGI(TAG, "MAVLink bridge active (UART%d → UDP:%d)",
                 MAVLINK_UART_NUM, MAVLINK_GCS_PORT);
    }

    /* === Phase 3: Camera + JPEG === */
    ESP_LOGI(TAG, "Phase 3: Initializing MIPI-CSI camera + JPEG...");
    err = camera_h264_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Continuing without camera");
    } else {
        httpd_handle_t server = camera_stream_server_start();
        if (server) {
            ESP_LOGI(TAG, "Camera streaming active");
        }
    }

    /* === Main Loop === */
    ESP_LOGI(TAG, "All subsystems initialized. Entering main loop.");
    printf("\n");

    uint32_t loop_count = 0;

    while (1) {
        /* Send ARP keepalive every 5 seconds */
        app_wlan_arp_send();

        /* Print status every 30 seconds */
        if (loop_count % 6 == 0) {
            print_status();
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
