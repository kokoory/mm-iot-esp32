/*
 * ESP32-P4 Helicopter Flight Controller - Main Entry Point
 *
 * CPU0 (HP Core 0): Flight controller tasks
 *   - Sensor Agent (1kHz IMU, 100Hz Baro/Mag, GPS)
 *   - Flight Controller Agent (500Hz control loop)
 *   - Actuator Agent (500Hz PWM output)
 *   - System Monitor Agent (10Hz health/battery/failsafe)
 *
 * CPU1 (HP Core 1): Communication tasks (esp32p4halow project)
 *   - HaLow WiFi + MAVLink + GCS bridge
 *   - Connected via RPC queues
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "common/board_config.h"
#include "common/flight_modes.h"
#include "uorb/uorb.h"
#include "rpc/rpc_core.h"
#include "agents/sensor_agent.h"
#include "agents/flight_ctrl_agent.h"
#include "agents/actuator_agent.h"
#include "agents/sysmon_agent.h"

static const char *TAG = "HELI_FC";

/* Global RPC context shared between FC and communication cores */
static rpc_context_t g_rpc_ctx;

/* Getter for other modules that need RPC access */
rpc_context_t *main_get_rpc_context(void)
{
    return &g_rpc_ctx;
}

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void init_status_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_STATUS_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_STATUS_LED, 0);
}

static void print_banner(void)
{
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, " ESP32-P4 Helicopter Flight Controller");
    ESP_LOGI(TAG, " PX4-inspired | 120 deg CCPM | Tail ESC");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "Core 0: Flight Controller Tasks");
    ESP_LOGI(TAG, "Core 1: HaLow Communication (separate project)");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
}

void app_main(void)
{
    /* Step 1: Initialize NVS */
    init_nvs();

    /* Step 2: Print startup banner */
    print_banner();

    /* Step 3: Initialize status LED */
    init_status_led();

    /* Step 4: Initialize uORB pub/sub message bus */
    ESP_LOGI(TAG, "Initializing uORB message bus...");
    orb_init();

    /* Step 5: Initialize inter-core RPC */
    ESP_LOGI(TAG, "Initializing RPC inter-core communication...");
    rpc_init(&g_rpc_ctx);

    /* Step 6: Start flight controller agents on Core 0 */
    ESP_LOGI(TAG, "Starting Sensor Agent on Core %d (priority %d)...",
             FC_CORE, SENSOR_TASK_PRIORITY);
    sensor_agent_start();

    /* Small delay to let sensors initialize before starting control loop */
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Starting Flight Controller Agent on Core %d (priority %d)...",
             FC_CORE, FLIGHT_CTRL_PRIORITY);
    flight_ctrl_agent_start();

    ESP_LOGI(TAG, "Starting Actuator Agent on Core %d (priority %d)...",
             FC_CORE, ACTUATOR_TASK_PRIORITY);
    actuator_agent_start();

    ESP_LOGI(TAG, "Starting System Monitor Agent on Core %d (priority %d)...",
             FC_CORE, SYSMON_TASK_PRIORITY);
    sysmon_agent_start(&g_rpc_ctx);

    /* Step 7: Report startup complete */
    ESP_LOGI(TAG, "All flight controller agents started successfully.");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "System ready. Waiting for arm command from GCS...");

    /* app_main returns, FreeRTOS scheduler continues running tasks */
}
