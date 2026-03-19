/*
 * Copyright 2025 Morse Micro
 *
 * This file is licensed under terms that can be found in the LICENSE.md file in the root
 * directory of the Morse Micro IoT SDK software package.
 */

/**
 * @file
 * @brief AP/STA Auto-Switch Example Application.
 *
 * This example demonstrates automatic switching between STA and AP modes:
 * - Starts in STA mode and attempts to connect to a configured AP.
 * - If the STA link does not come up within a timeout, switches to AP mode.
 * - While in STA mode, if the link goes down and stays down past a grace period,
 *   switches to AP mode.
 * - While in AP mode, if no stations connect within a timeout, switches back to STA mode.
 * - The cycle repeats indefinitely.
 *
 * Architecture notes:
 * - The transceiver is booted once via mmwlan_boot() and kept alive across mode switches.
 *   This means mmwlan_sta_disable() and mmwlan_ap_disable() will not shut down the chip
 *   (as documented in the mmwlan API).
 * - mmipal (IP stack) is NOT used in this example because it does not support deinit/reinit
 *   for switching between DHCP (STA) and Static IP (AP). Instead, raw mmwlan link-level
 *   callbacks are used to detect connectivity.
 * - For a production application that needs IP connectivity in both modes, consider using
 *   ESP-NETIF directly or implementing a full restart between modes.
 */

#include <string.h>
#include "mmhal.h"
#include "mmosal.h"
#include "mmutils.h"
#include "mmregdb.h"

/* Country code for regulatory compliance. */
#ifndef COUNTRY_CODE
#define COUNTRY_CODE "US"
#endif

/*
 * ---------------------------
 * STA Mode Configuration
 * ---------------------------
 */

#ifndef STA_SSID
/** SSID of the AP to connect to in STA mode. */
#define STA_SSID "node2"
#endif

#ifndef STA_PASSPHRASE
/** Passphrase of the AP to connect to in STA mode. */
#define STA_PASSPHRASE "12345678"
#endif

/*
 * ---------------------------
 * AP Mode Configuration
 * ---------------------------
 */

#ifndef AP_SSID
/** SSID to broadcast in AP mode. (Do not quote; it will be stringified.) */
#define AP_SSID MorseMicroFallback
#endif

#ifndef AP_PASSPHRASE
/** Passphrase for AP mode. (Do not quote; it will be stringified.) */
#define AP_PASSPHRASE 12345678
#endif

/*
 * ---------------------------
 * AP Channel Configuration
 * ---------------------------
 */

#ifndef OP_CLASS
#define OP_CLASS (25)
#endif

#ifndef S1G_CHANNEL
#define S1G_CHANNEL (43)
#endif

#ifndef PRIMARY_BW_MHZ
#define PRIMARY_BW_MHZ (0)
#endif

#ifndef PRIMARY_1MHZ_CHANNEL_INDEX
#define PRIMARY_1MHZ_CHANNEL_INDEX (0)
#endif

#ifndef MAX_STAS
#define MAX_STAS MMWLAN_DEFAULT_AP_MAX_STAS
#endif

/*
 * ---------------------------
 * Timing Configuration (ms)
 * ---------------------------
 */

/** How long to wait for STA link-up before switching to AP mode. */
#ifndef STA_CONNECT_TIMEOUT_MS
#define STA_CONNECT_TIMEOUT_MS (30000)
#endif

/** How long to stay in AP mode with no connected clients before switching back to STA. */
#ifndef AP_IDLE_TIMEOUT_MS
#define AP_IDLE_TIMEOUT_MS (60000)
#endif

/** Grace period after STA link-down before switching to AP. */
#ifndef STA_LINK_DOWN_GRACE_MS
#define STA_LINK_DOWN_GRACE_MS (10000)
#endif

/*
 * ---------------------------
 * Stringify helpers
 * ---------------------------
 */
#define _STRINGIFY(x) #x
#define STRINGIFY(x) _STRINGIFY(x)

/*
 * ---------------------------
 * Application state
 * ---------------------------
 */

enum app_mode
{
    APP_MODE_IDLE,
    APP_MODE_STA,
    APP_MODE_AP,
};

static volatile enum app_mode current_mode = APP_MODE_IDLE;
static volatile bool sta_link_is_up = false;
static volatile uint32_t ap_client_count = 0;

/** Semaphore used to wake the main loop on state changes. */
static struct mmosal_semb *event_semaphore = NULL;

/*
 * ---------------------------
 * Callbacks
 * ---------------------------
 */

/** STA status callback — logs connection state changes. */
static void sta_status_handler(enum mmwlan_sta_state sta_state)
{
    const char *desc[] = { "DISABLED", "CONNECTING", "CONNECTED" };
    printf("[STA] State: %s\n", desc[sta_state]);
}

/** STA link state callback — signals the main loop on link changes. */
static void sta_link_state_cb(enum mmwlan_link_state link_state, void *arg)
{
    (void)arg;
    if (link_state == MMWLAN_LINK_UP)
    {
        printf("[STA] Link UP\n");
        sta_link_is_up = true;
    }
    else
    {
        printf("[STA] Link DOWN\n");
        sta_link_is_up = false;
    }
    mmosal_semb_give(event_semaphore);
}

/** AP client status callback — tracks connected/disconnected clients. */
static void ap_sta_status_cb(const struct mmwlan_ap_sta_status *sta_status, void *arg)
{
    (void)arg;
    if (sta_status->state == MMWLAN_AP_STA_AUTHORIZED)
    {
        ap_client_count++;
        printf("[AP] Client connected (%02x:%02x:%02x:%02x:%02x:%02x), clients: %lu\n",
               sta_status->mac_addr[0], sta_status->mac_addr[1],
               sta_status->mac_addr[2], sta_status->mac_addr[3],
               sta_status->mac_addr[4], sta_status->mac_addr[5],
               (unsigned long)ap_client_count);
    }
    else if (sta_status->state == MMWLAN_AP_STA_UNKNOWN)
    {
        if (ap_client_count > 0)
        {
            ap_client_count--;
        }
        printf("[AP] Client disconnected (%02x:%02x:%02x:%02x:%02x:%02x), clients: %lu\n",
               sta_status->mac_addr[0], sta_status->mac_addr[1],
               sta_status->mac_addr[2], sta_status->mac_addr[3],
               sta_status->mac_addr[4], sta_status->mac_addr[5],
               (unsigned long)ap_client_count);
    }
    mmosal_semb_give(event_semaphore);
}

/** Receive callback — required by mmwlan but not used in this example. */
static void rx_handler(uint8_t *header, unsigned header_len,
                       uint8_t *payload, unsigned payload_len, void *arg)
{
    (void)header;
    (void)header_len;
    (void)payload;
    (void)payload_len;
    (void)arg;
}

/*
 * ---------------------------
 * Mode start/stop helpers
 * ---------------------------
 */

static enum mmwlan_status start_sta_mode(void)
{
    enum mmwlan_status status;
    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;

    printf("\n=== Switching to STA mode ===\n");
    printf("[STA] Connecting to SSID: %s\n", STA_SSID);

    sta_link_is_up = false;

    sta_args.ssid_len = sizeof(STA_SSID) - 1;
    memcpy(sta_args.ssid, STA_SSID, sta_args.ssid_len);
    sta_args.passphrase_len = sizeof(STA_PASSPHRASE) - 1;
    memcpy(sta_args.passphrase, STA_PASSPHRASE, sta_args.passphrase_len);
    sta_args.security_type = MMWLAN_SAE;

    status = mmwlan_sta_enable(&sta_args, sta_status_handler);
    if (status != MMWLAN_SUCCESS)
    {
        printf("[STA] Failed to enable STA mode (status %d)\n", status);
        return status;
    }

    current_mode = APP_MODE_STA;
    return MMWLAN_SUCCESS;
}

static void stop_sta_mode(void)
{
    printf("[STA] Disabling...\n");
    mmwlan_sta_disable();
    sta_link_is_up = false;
    current_mode = APP_MODE_IDLE;
    printf("[STA] Disabled\n");
}

static enum mmwlan_status start_ap_mode(void)
{
    enum mmwlan_status status;
    struct mmwlan_ap_args ap_args = MMWLAN_AP_ARGS_INIT;

    printf("\n=== Switching to AP mode ===\n");
    printf("[AP] SSID: %s\n", STRINGIFY(AP_SSID));

    ap_client_count = 0;

    (void)mmosal_safer_strcpy((char *)ap_args.ssid, STRINGIFY(AP_SSID), sizeof(ap_args.ssid));
    ap_args.ssid_len = strlen((char *)ap_args.ssid);
    (void)mmosal_safer_strcpy(ap_args.passphrase, STRINGIFY(AP_PASSPHRASE),
                              sizeof(ap_args.passphrase));
    ap_args.passphrase_len = strlen(ap_args.passphrase);
    ap_args.security_type = MMWLAN_SAE;
    ap_args.pmf_mode = MMWLAN_PMF_REQUIRED;
    ap_args.op_class = OP_CLASS;
    ap_args.s1g_chan_num = S1G_CHANNEL;
    ap_args.pri_bw_mhz = PRIMARY_BW_MHZ;
    ap_args.pri_1mhz_chan_idx = PRIMARY_1MHZ_CHANNEL_INDEX;
    ap_args.max_stas = MAX_STAS;
    ap_args.sta_status_cb = ap_sta_status_cb;
    ap_args.sta_status_cb_arg = NULL;

    status = mmwlan_ap_enable(&ap_args);
    if (status != MMWLAN_SUCCESS)
    {
        printf("[AP] Failed to enable AP mode (status %d)\n", status);
        return status;
    }

    current_mode = APP_MODE_AP;
    printf("[AP] Started successfully\n");
    return MMWLAN_SUCCESS;
}

static void stop_ap_mode(void)
{
    printf("[AP] Disabling...\n");
    mmwlan_ap_disable();
    ap_client_count = 0;
    current_mode = APP_MODE_IDLE;
    printf("[AP] Disabled\n");
}

/*
 * ---------------------------
 * Version info
 * ---------------------------
 */

static void app_print_version_info(void)
{
    struct mmwlan_version version = { 0 };
    struct mmwlan_bcf_metadata bcf_metadata = { 0 };

    printf("-----------------------------------\n");
    printf("  HW Version:              %s\n", CONFIG_IDF_TARGET);

    if (mmwlan_get_bcf_metadata(&bcf_metadata) == MMWLAN_SUCCESS)
    {
        printf("  BCF API version:         %u.%u.%u\n",
               bcf_metadata.version.major, bcf_metadata.version.minor,
               bcf_metadata.version.patch);
        if (bcf_metadata.board_desc[0] != '\0')
        {
            printf("  BCF board description:   %s\n", bcf_metadata.board_desc);
        }
    }

    if (mmwlan_get_version(&version) == MMWLAN_SUCCESS)
    {
        printf("  Morselib version:        %s\n", version.morselib_version);
        printf("  Morse firmware version:  %s\n", version.morse_fw_version);
        printf("  Morse chip ID:           0x%04lx\n", version.morse_chip_id);
    }
    printf("-----------------------------------\n");
}

/*
 * ---------------------------
 * Main state machine
 * ---------------------------
 *
 *  +-----------+      timeout/link-down      +-----------+
 *  | STA Mode  | --------------------------> |  AP Mode  |
 *  | (client)  |                             | (fallback)|
 *  +-----------+ <-------------------------- +-----------+
 *                    no clients for timeout
 */
void app_main(void)
{
    enum mmwlan_status status;
    const struct mmwlan_s1g_channel_list *channel_list;
    struct mmwlan_boot_args boot_args = MMWLAN_BOOT_ARGS_INIT;

    printf("\n\nAP/STA Auto-Switch Example (Built " __DATE__ " " __TIME__ ")\n\n");

    event_semaphore = mmosal_semb_create("evt");
    MMOSAL_ASSERT(event_semaphore != NULL);

    /* Initialize HAL and WLAN. */
    mmhal_init();
    mmwlan_init();

    /* Disable power save — BUSY pin not wired on XIAO HaLow boards. */
    mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED);

    /* Set regulatory domain. */
    channel_list = mmwlan_lookup_regulatory_domain(get_regulatory_db(), COUNTRY_CODE);
    MMOSAL_ASSERT(channel_list != NULL);
    status = mmwlan_set_channel_list(channel_list);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);

    /* Register link state callback (persists across mode switches). */
    status = mmwlan_register_link_state_cb(sta_link_state_cb, NULL);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);

    /* Register a dummy RX callback (required by the WLAN driver). */
    status = mmwlan_register_rx_cb(rx_handler, NULL);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);

    /* Boot transceiver once. Keeping it booted prevents sta_disable/ap_disable
     * from shutting down the chip, enabling fast mode switches. */
    status = mmwlan_boot(&boot_args);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);
    app_print_version_info();

    printf("[MAIN] Timeouts: STA connect=%lums, STA grace=%lums, AP idle=%lums\n",
           (unsigned long)STA_CONNECT_TIMEOUT_MS,
           (unsigned long)STA_LINK_DOWN_GRACE_MS,
           (unsigned long)AP_IDLE_TIMEOUT_MS);

    /* ---- Main loop: STA <-> AP cycle ---- */
    while (1)
    {
        /* ============ STA Phase ============ */
        status = start_sta_mode();
        if (status != MMWLAN_SUCCESS)
        {
            printf("[MAIN] STA start failed, retrying in 5s...\n");
            mmosal_task_sleep(5000);
            continue;
        }

        /* Wait for connection or timeout. */
        printf("[MAIN] Waiting %lu ms for STA connection...\n",
               (unsigned long)STA_CONNECT_TIMEOUT_MS);

        uint32_t sta_start = mmosal_get_time_ms();
        while (!sta_link_is_up)
        {
            uint32_t elapsed = mmosal_get_time_ms() - sta_start;
            if (elapsed >= STA_CONNECT_TIMEOUT_MS)
            {
                break;
            }
            uint32_t remaining = STA_CONNECT_TIMEOUT_MS - elapsed;
            mmosal_semb_wait(event_semaphore, remaining);
        }

        if (sta_link_is_up)
        {
            printf("[MAIN] STA connected! Monitoring link...\n");

            /* Stay in STA as long as the link is up. */
            while (sta_link_is_up)
            {
                mmosal_semb_wait(event_semaphore, 5000);
            }

            /* Link went down — wait grace period for reconnection. */
            printf("[MAIN] Link lost. Grace period: %lu ms\n",
                   (unsigned long)STA_LINK_DOWN_GRACE_MS);

            uint32_t grace_start = mmosal_get_time_ms();
            while (!sta_link_is_up)
            {
                uint32_t elapsed = mmosal_get_time_ms() - grace_start;
                if (elapsed >= STA_LINK_DOWN_GRACE_MS)
                {
                    break;
                }
                uint32_t remaining = STA_LINK_DOWN_GRACE_MS - elapsed;
                mmosal_semb_wait(event_semaphore, remaining);
            }

            if (sta_link_is_up)
            {
                printf("[MAIN] Link recovered during grace period\n");
                continue;  /* Stay in STA mode */
            }
        }
        else
        {
            printf("[MAIN] STA connection timed out\n");
        }

        stop_sta_mode();

        /* ============ AP Phase ============ */
        status = start_ap_mode();
        if (status != MMWLAN_SUCCESS)
        {
            printf("[MAIN] AP start failed, retrying STA in 5s...\n");
            mmosal_task_sleep(5000);
            continue;
        }

        printf("[MAIN] AP active. Will return to STA if idle for %lu ms\n",
               (unsigned long)AP_IDLE_TIMEOUT_MS);

        uint32_t idle_start = mmosal_get_time_ms();
        while (1)
        {
            mmosal_semb_wait(event_semaphore, 5000);

            if (ap_client_count > 0)
            {
                /* Clients connected — reset idle timer. */
                idle_start = mmosal_get_time_ms();
            }
            else
            {
                uint32_t elapsed = mmosal_get_time_ms() - idle_start;
                if (elapsed >= AP_IDLE_TIMEOUT_MS)
                {
                    printf("[MAIN] AP idle timeout (%lu ms), switching to STA\n",
                           (unsigned long)elapsed);
                    break;
                }
            }
        }

        stop_ap_mode();
    }
}
