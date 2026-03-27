/*
 * ESP32-P4 HaLow Communication Module (Core 1)
 *
 * Integrates three subsystems on Core 1:
 *   1. Wi-Fi HaLow (MMECH06) - Long-range sub-GHz wireless link
 *   2. MIPI-CSI Camera + H.264  - Hardware-encoded video streaming
 *   3. MAVLink GCS Bridge        - Telemetry/command via RPC from Core 0
 *
 * Hardware:
 *   - Waveshare ESP32-P4-Module-DEV-KIT board
 *   - MMECH06 Wi-Fi HaLow module (SPI, BCF=bcf_mf08651_us.mbin)
 *   - MIPI-CSI camera (OV5647 / SC2336)
 *   - Flight controller running on Core 0 (same binary)
 *
 * Network Topology:
 *   FC (Core 0) <-RPC-> HaLow Module (Core 1) <-SPI-> MMECH06 ~~~HaLow~~~ AP <-> GCS
 *                           |
 *                        MIPI-CSI
 *                        Camera
 *
 * Endpoints:
 *   http://<ip>/        - MJPEG camera stream (fallback)
 *   http://<ip>/h264    - H.264 camera stream
 *   http://<ip>/status  - JSON system status
 *   UDP 14550           - MAVLink telemetry (GCS port)
 *
 * The RPC context is shared with the flight controller running on Core 0.
 * It is passed in via halow_comm_start() -- NOT created locally.
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_event.h"

#include "mm_app_common.h"
#include "mm_app_loadconfig.h"
#include "camera_h264.h"
#include "../rpc/rpc_core.h"
#include "gcs_bridge.h"
#include "mavlink_handler.h"
#include "halow_comm.h"

static const char *TAG = "halow_main";

#define MAVLINK_TASK_STACK_SIZE  8192
#define MAVLINK_TASK_PRIORITY    5
#define MAVLINK_TASK_CORE        1

#define HALOW_INIT_TASK_STACK    8192
#define HALOW_INIT_TASK_PRIORITY 4

/**
 * Print system status summary to console.
 */
static void print_status(void)
{
    printf("\n--- HaLow System Status ---\n");
    printf("WiFi HaLow: %s\n", app_wlan_is_connected() ? "CONNECTED" : "DISCONNECTED");
    printf("Camera FPS: %.1f\n", camera_get_fps());
    printf("Free heap: %lu bytes (PSRAM: %lu bytes)\n",
           (unsigned long)esp_get_free_heap_size(),
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    printf("Uptime: %lld sec\n", esp_timer_get_time() / 1000000LL);
    printf("---------------------------\n\n");
}

/* ── HaLow init task (runs on Core 1) ────────────────────────── */

static void halow_init_task(void *param)
{
    rpc_context_t *rpc = (rpc_context_t *)param;
    esp_err_t err;

    ESP_LOGI(TAG, "ESP32-P4 HaLow Communication Module starting on Core %d...",
             xPortGetCoreID());

    printf("\n");
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

    /* === Phase 2: GCS Bridge + MAVLink Handler === */
    ESP_LOGI(TAG, "Phase 2: Initializing GCS bridge + MAVLink handler...");

    if (gcs_bridge_init() != 0) {
        ESP_LOGE(TAG, "GCS bridge initialization failed");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "GCS bridge ready on UDP port %d", GCS_BRIDGE_UDP_PORT);

    mavlink_handler_config_t mav_config = {
        .heartbeat_hz  = 1,
        .attitude_hz   = 10,
        .gps_hz        = 5,
        .battery_hz    = 2,
        .vfr_hud_hz    = 2,
    };
    mavlink_handler_init(rpc, &mav_config);

    /* Create MAVLink handler task pinned to Core 1 */
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        mavlink_handler_task,
        "mavlink_handler",
        MAVLINK_TASK_STACK_SIZE,
        rpc,
        MAVLINK_TASK_PRIORITY,
        NULL,
        MAVLINK_TASK_CORE
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MAVLink handler task");
    } else {
        ESP_LOGI(TAG, "MAVLink handler task started on Core %d", MAVLINK_TASK_CORE);
    }

    /* === Phase 3: Camera + H.264 === */
    ESP_LOGI(TAG, "Phase 3: Initializing MIPI-CSI camera + H.264...");
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

    /* === Main Loop: keepalive + status === */
    ESP_LOGI(TAG, "All subsystems initialized. Entering main loop.");

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

/* ── Public API ───────────────────────────────────────────────── */

void halow_comm_start(rpc_context_t *rpc)
{
    ESP_LOGI(TAG, "Launching HaLow communication on Core %d", MAVLINK_TASK_CORE);

    xTaskCreatePinnedToCore(
        halow_init_task,
        "halow_init",
        HALOW_INIT_TASK_STACK,
        (void *)rpc,
        HALOW_INIT_TASK_PRIORITY,
        NULL,
        MAVLINK_TASK_CORE
    );
}
