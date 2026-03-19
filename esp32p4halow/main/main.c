/*
 * ESP32-P4 HaLow Communication Module - Main Entry Point
 *
 * This is the main application for the HaLow (802.11ah) communication
 * subsystem. It runs on Core 1 of the ESP32-P4 and is responsible for:
 *
 * 1. Initializing the Morse Micro HaLow radio (STA mode)
 * 2. Establishing network connectivity (IP via DHCP)
 * 3. Running the GCS bridge (UDP socket for MAVLink)
 * 4. Running the MAVLink handler (RPC <-> MAVLink translation)
 *
 * The RPC context is shared with the flight controller running on Core 0
 * via a global variable initialized before this task starts.
 */
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mmosal.h"
#include "mmhal_os.h"
#include "mmwlan.h"
#include "mmipal.h"
#include "mmregdb.h"

#include "rpc/rpc_core.h"
#include "gcs_bridge.h"
#include "mavlink_handler.h"

static const char *TAG = "halow_main";

/* ── Configuration ────────────────────────────────────────────── */

#ifndef HALOW_COUNTRY_CODE
#define HALOW_COUNTRY_CODE "US"
#endif

#ifndef HALOW_SSID
#define HALOW_SSID "HeliGCS"
#endif

#ifndef HALOW_PASSPHRASE
#define HALOW_PASSPHRASE "helicopter"
#endif

#define MAVLINK_TASK_STACK_SIZE  8192
#define MAVLINK_TASK_PRIORITY    5
#define MAVLINK_TASK_CORE        1

/* ── Global shared RPC context ────────────────────────────────── *
 * This is allocated and initialized by the FC (Core 0) before
 * starting the HaLow task. The HaLow module accesses it via
 * this extern declaration.
 */
rpc_context_t g_rpc_context;

/* ── Network state ────────────────────────────────────────────── */

static struct mmosal_semb *s_link_up_sem = NULL;
static volatile bool s_network_ready = false;

/* ── Morse Micro Callbacks ────────────────────────────────────── */

static void sta_status_callback(enum mmwlan_sta_state sta_state)
{
    const char *state_names[] = { "DISABLED", "CONNECTING", "CONNECTED" };
    ESP_LOGI(TAG, "WLAN STA state: %s (%u)", state_names[sta_state], sta_state);
}

static void link_status_callback(const struct mmipal_link_status *link_status)
{
    if (link_status->link_state == MMIPAL_LINK_UP) {
        ESP_LOGI(TAG, "Link UP - IP: %s  GW: %s  Mask: %s",
                 link_status->ip_addr, link_status->gateway, link_status->netmask);
        s_network_ready = true;
        mmosal_semb_give(s_link_up_sem);
    } else {
        ESP_LOGW(TAG, "Link DOWN");
        s_network_ready = false;
    }
}

/* ── HaLow Initialization ────────────────────────────────────── */

static int halow_init_radio(void)
{
    enum mmwlan_status status;

    ESP_LOGI(TAG, "Initializing Morse Micro HaLow radio...");

    /* Create link-up semaphore */
    s_link_up_sem = mmosal_semb_create("halow_link_up");
    if (s_link_up_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create link-up semaphore");
        return -1;
    }

    /* Initialize HAL and WLAN subsystems (must be in this order) */
    mmhal_init();
    mmwlan_init();

    /* Set regulatory domain / channel list */
    const struct mmwlan_s1g_channel_list *channel_list =
        mmwlan_lookup_regulatory_domain(get_regulatory_db(), HALOW_COUNTRY_CODE);
    if (channel_list == NULL) {
        ESP_LOGE(TAG, "No regulatory domain for country code: %s", HALOW_COUNTRY_CODE);
        return -1;
    }
    status = mmwlan_set_channel_list(channel_list);
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set channel list: %d", status);
        return -1;
    }

    /* Boot the WLAN firmware */
    struct mmwlan_boot_args boot_args = MMWLAN_BOOT_ARGS_INIT;
    status = mmwlan_boot(&boot_args);
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGE(TAG, "Failed to boot WLAN: %d", status);
        return -1;
    }

    /* Print version info */
    struct mmwlan_version version = {0};
    status = mmwlan_get_version(&version);
    if (status == MMWLAN_SUCCESS) {
        ESP_LOGI(TAG, "Morse FW: %s  Morselib: %s  Chip: 0x%04lx (%s)",
                 version.morse_fw_version, version.morselib_version,
                 version.morse_chip_id, version.morse_chip_id_string);
    }

    /* Initialize IP stack (LWIP via MMIPAL) */
    struct mmipal_init_args mmipal_args = MMIPAL_INIT_ARGS_DEFAULT;
    if (mmipal_init(&mmipal_args) != MMIPAL_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize IP stack");
        return -1;
    }
    mmipal_set_link_status_callback(link_status_callback);

    ESP_LOGI(TAG, "HaLow radio initialized");
    return 0;
}

static int halow_connect(void)
{
    enum mmwlan_status status;

    ESP_LOGI(TAG, "Connecting to AP: SSID=%s", HALOW_SSID);

    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;
    sta_args.ssid_len = strlen(HALOW_SSID);
    memcpy(sta_args.ssid, HALOW_SSID, sta_args.ssid_len);

#ifdef HALOW_PASSPHRASE
    sta_args.passphrase_len = strlen(HALOW_PASSPHRASE);
    memcpy(sta_args.passphrase, HALOW_PASSPHRASE, sta_args.passphrase_len);
    sta_args.security_type = MMWLAN_SAE;
#else
    sta_args.security_type = MMWLAN_OWE;
#endif

    status = mmwlan_sta_enable(&sta_args, sta_status_callback);
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGE(TAG, "Failed to enable STA mode: %d", status);
        return -1;
    }

    /* Wait for link to come up (DHCP to complete) */
    ESP_LOGI(TAG, "Waiting for network link...");
    bool ok = mmosal_semb_wait(s_link_up_sem, 30000); /* 30 second timeout */
    if (!ok) {
        ESP_LOGE(TAG, "Timed out waiting for network link");
        return -1;
    }

    ESP_LOGI(TAG, "Network connected and ready");
    return 0;
}

/* ── Main Application Entry ───────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 HaLow Communication Module starting...");

    /* 1. Initialize NVS (required for Wi-Fi config storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased due to format change");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* 2. Initialize RPC context (shared with Core 0 FC) */
    rpc_init(&g_rpc_context);
    ESP_LOGI(TAG, "RPC context initialized");

    /* 3. Initialize HaLow radio */
    if (halow_init_radio() != 0) {
        ESP_LOGE(TAG, "HaLow radio initialization failed");
        return;
    }

    /* 4. Connect to HaLow AP */
    if (halow_connect() != 0) {
        ESP_LOGE(TAG, "HaLow connection failed, will retry...");
        /* In production, implement retry logic here */
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (halow_connect() != 0) {
            ESP_LOGE(TAG, "HaLow connection failed after retry");
            return;
        }
    }

    /* 5. Initialize GCS bridge (UDP socket) */
    if (gcs_bridge_init() != 0) {
        ESP_LOGE(TAG, "GCS bridge initialization failed");
        return;
    }
    ESP_LOGI(TAG, "GCS bridge ready on UDP port %d", GCS_BRIDGE_UDP_PORT);

    /* 6. Initialize and start MAVLink handler */
    mavlink_handler_config_t mav_config = {
        .heartbeat_hz  = 1,
        .attitude_hz   = 10,
        .gps_hz        = 5,
        .battery_hz    = 2,
        .sys_status_hz = 1,
        .vfr_hud_hz    = 2,
    };
    mavlink_handler_init(&g_rpc_context, &mav_config);

    /* Create MAVLink handler task pinned to Core 1 */
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        mavlink_handler_task,
        "mavlink_handler",
        MAVLINK_TASK_STACK_SIZE,
        &g_rpc_context,
        MAVLINK_TASK_PRIORITY,
        NULL,
        MAVLINK_TASK_CORE
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MAVLink handler task");
        return;
    }

    ESP_LOGI(TAG, "MAVLink handler task started on Core %d", MAVLINK_TASK_CORE);
    ESP_LOGI(TAG, "HaLow communication module fully operational");
}
