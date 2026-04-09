/*
 * ESP32-P4 HaLow Communication Module
 *
 * Wi-Fi HaLow init (morselib) MUST run from app_main (Core 0) - same as
 * the reference esp32p4halow example. Camera + MAVLink run on Core 1.
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
#include "thermal_camera.h"
#include "../rpc/rpc_core.h"
#include "gcs_bridge.h"
#include "mavlink_handler.h"
#include "halow_comm.h"

static const char *TAG = "halow_main";

#define MAVLINK_TASK_STACK_SIZE  8192
#define MAVLINK_TASK_PRIORITY    5
#define MAVLINK_TASK_CORE        1

#define HALOW_COMM_TASK_STACK   8192
#define HALOW_COMM_TASK_PRIORITY 4

/**
 * Print system status summary to console.
 */
static void print_status(void)
{
    printf("\n--- HaLow System Status ---\n");
    printf("WiFi HaLow: %s\n", app_wlan_is_connected() ? "CONNECTED" : "DISCONNECTED");
    if (app_wlan_is_connected()) {
        app_wlan_print_link_stats();
    }
    printf("TX flow: %s (pause_count=%lu)\n",
           app_wlan_tx_is_paused() ? "PAUSED" : "ready",
           (unsigned long)app_wlan_tx_pause_count());
    printf("GCS: %s\n", gcs_bridge_is_active() ? "active" : "inactive");
    printf("Camera FPS: %.1f\n", camera_get_fps());
    printf("Free heap: %lu bytes (PSRAM: %lu bytes)\n",
           (unsigned long)esp_get_free_heap_size(),
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    printf("Uptime: %lld sec\n", esp_timer_get_time() / 1000000LL);
    printf("---------------------------\n\n");
}

/* ── Communication task (runs on Core 1 AFTER HaLow is connected) ── */

static void halow_comm_task(void *param)
{
    rpc_context_t *rpc = (rpc_context_t *)param;
    esp_err_t err;

    ESP_LOGI(TAG, "Communication task running on Core %d", xPortGetCoreID());

    /* === GCS Bridge + MAVLink Handler === */
    ESP_LOGI(TAG, "Initializing GCS bridge + MAVLink handler...");

    if (gcs_bridge_init() != 0) {
        ESP_LOGE(TAG, "GCS bridge initialization failed");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "GCS bridge ready on UDP port %d", GCS_BRIDGE_UDP_PORT);

    mavlink_handler_config_t mav_config = {
        .heartbeat_hz  = 1,
        .attitude_hz   = 4,
        .gps_hz        = 2,
        .battery_hz    = 1,
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

    /* === USB Webcam === */
    ESP_LOGI(TAG, "Initializing USB webcam...");
    err = camera_h264_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Continuing without camera");
    }

    /* Always start HTTP server (thermal viewer + status need it even without camera) */
    httpd_handle_t server = camera_stream_server_start();
    if (server) {
        ESP_LOGI(TAG, "HTTP server active");
    }

    /* === Thermal Camera (PureThermal Lepton via USB) === */
    ESP_LOGI(TAG, "Initializing thermal camera (USB UVC)...");
    err = thermal_camera_init(NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Thermal camera not available (connect PureThermal to USB OTG)");
    } else {
        thermal_camera_start();
        ESP_LOGI(TAG, "Thermal camera streaming (160x120 @ 9fps)");
    }

    /* === Main Loop: keepalive + status === */
    ESP_LOGI(TAG, "All subsystems initialized. Entering main loop.");

    uint32_t loop_count = 0;
    while (1) {
        app_wlan_arp_send();

        if (loop_count % 6 == 0) {
            print_status();
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ── Public API ───────────────────────────────────────────────── */

void halow_comm_init_wlan(void)
{
    ESP_LOGI(TAG, "Phase 1: Initializing Wi-Fi HaLow (from Core 0)...");

    printf("\n");
    printf("==============================================\n");
    printf("  ESP32-P4 HaLow Drone System\n");
    printf("  Camera + MAVLink + Wi-Fi HaLow\n");
    printf("  Built " __DATE__ " " __TIME__ "\n");
    printf("==============================================\n\n");

    app_wlan_init();
    app_wlan_start();
    ESP_LOGI(TAG, "Wi-Fi HaLow connected");

    /* Create default event loop AFTER HaLow is connected (matches reference) */
    esp_event_loop_create_default();
}

void halow_comm_start(rpc_context_t *rpc)
{
    ESP_LOGI(TAG, "Launching comm tasks on Core %d", MAVLINK_TASK_CORE);

    xTaskCreatePinnedToCore(
        halow_comm_task,
        "halow_comm",
        HALOW_COMM_TASK_STACK,
        (void *)rpc,
        HALOW_COMM_TASK_PRIORITY,
        NULL,
        MAVLINK_TASK_CORE
    );
}
