/*
 * ESP32-P4 Helicopter Flight Controller - Main Entry Point
 *
 * Initialization order (matching reference esp32p4halow):
 *   1. NVS, params, uORB, RPC
 *   2. Wi-Fi HaLow init (MUST run from app_main / Core 0)
 *   3. FC agents on Core 0
 *   4. Comm tasks (camera, MAVLink) on Core 1
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "common/board_config.h"
#include "common/flight_modes.h"
#include "common/param.h"
#include "uorb/uorb.h"
#include "rpc/rpc_core.h"
#include "agents/sensor_agent.h"
#include "agents/flight_ctrl_agent.h"
#include "agents/actuator_agent.h"
#include "agents/sysmon_agent.h"
#include "comm/halow_comm.h"
#include "common/i2c_sync.h"

EventGroupHandle_t g_i2c_sync_event;

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

void app_main(void)
{
    /* Pre-initialize newlib's __env_lock before any code that might log from
     * ISR/critical-section context. The lock is lazily created on first use;
     * calling setenv()+tzset() here forces creation in safe task context.
     * See: https://github.com/espressif/esp-idf/issues/11674 */
    setenv("TZ", "UTC0", 1);
    tzset();

    /* Step 1: Initialize NVS + params */
    init_nvs();
    ESP_LOGI(TAG, "Initializing parameter system...");
    param_init();

    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, " ESP32-P4 Helicopter Flight Controller");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Internal RAM free: %lu bytes",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    init_status_led();

    /* Step 2: Initialize uORB + RPC */
    orb_init();
    rpc_init(&g_rpc_ctx);

    /* Step 3: I2C bus synchronization init */
    i2c_sync_init();

    /* Step 4: Wi-Fi HaLow init — MUST run from app_main (Core 0)
     * This matches the reference esp32p4halow example exactly.
     * morselib creates internal tasks that expect Core 0 context. */
    halow_comm_init_wlan();

    ESP_LOGI(TAG, "Internal RAM free after HaLow: %lu bytes",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    /* Step 5: Start comm tasks (camera, MAVLink) on Core 1 */
    halow_comm_start(&g_rpc_ctx);

    /* Step 6: Start FC agents on Core 0 */
    sensor_agent_start();     /* IMU, baro, GPS, airspeed */
    vTaskDelay(pdMS_TO_TICKS(100));
    sysmon_agent_start(&g_rpc_ctx);  /* System monitor + RPC telemetry forwarding */
    flight_ctrl_agent_start();
    actuator_agent_start();

    ESP_LOGI(TAG, "Init complete. Free heap: %lu bytes",
             (unsigned long)esp_get_free_heap_size());

    /* app_main returns, FreeRTOS scheduler continues running tasks */
}
