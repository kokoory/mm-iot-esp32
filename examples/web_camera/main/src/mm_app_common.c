/*
 * Copyright 2023 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "mmosal.h"
#include "mmhal.h"
#include "mmwlan.h"
#include "mmipal.h"
#include "mm_app_common.h"
#include "mm_app_loadconfig.h"

/** Maximum number of DNS servers to attempt to retrieve from config store. */
#ifndef DNS_MAX_SERVERS
#define DNS_MAX_SERVERS                 2
#endif

/** Binary semaphore used to start user_main() once the link comes up. */
static struct mmosal_semb *link_established = NULL;

static bool link_up = false;
static uint32_t ip_addr_u32 = 0;
static uint32_t gw_addr_u32 = 0;
uint8_t mac_addr[MMWLAN_MAC_ADDR_LEN];

/**
 * WLAN station status callback, invoked when WLAN STA state changes.
 *
 * @param sta_state  The new STA state.
 */
static void sta_status_callback(enum mmwlan_sta_state sta_state)
{
    switch (sta_state)
    {
    case MMWLAN_STA_DISABLED:
        printf("WLAN STA disabled\n");
        break;

    case MMWLAN_STA_CONNECTING:
        printf("WLAN STA connecting\n");
        break;

    case MMWLAN_STA_CONNECTED:
        printf("WLAN STA connected\n");
        break;
    }
}

/**
 * Link status callback
 *
 * @param link_status   Current link status info.
 */
static void link_status_callback(const struct mmipal_link_status *link_status)
{
    uint32_t time_ms = mmosal_get_time_ms();
    if (link_status->link_state == MMIPAL_LINK_UP)
    {
        printf("Link is up. Time: %lu ms, ", time_ms);
        printf("IP: %s, ", link_status->ip_addr);
        printf("Netmask: %s, ", link_status->netmask);
        printf("Gateway: %s\n", link_status->gateway);

        mmosal_semb_give(link_established);

        ip_addr_u32 = ipaddr_addr(link_status->ip_addr);
        printf("IP hex: 0X%X\n", ip_addr_u32);
        gw_addr_u32 = ipaddr_addr(link_status->gateway);
        printf("GW hex: 0X%X\n", gw_addr_u32);

        link_up = true;
        app_wlan_arp_send();
    }
    else
    {
        printf("Link is down. Time: %lu ms\n", time_ms);
        link_up = false;
    }
}

void app_wlan_init(void)
{
    enum mmwlan_status status;
    struct mmwlan_version version;

    /* Ensure we don't call twice */
    MMOSAL_ASSERT(link_established == NULL);
    link_established = mmosal_semb_create("link_established");

    /* Initialize Morse subsystems, note that they must be called in this order. */
    mmhal_init();
    mmwlan_init();

    /* Seeed XIAO HaLow boards do not wire the BUSY pin.
     * Disable power save as a workaround. */
    mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED);

    mmwlan_set_channel_list(load_channel_list());

    /* Load IP stack settings from config store, or use defaults if no entry found in
     * config store. */
    struct mmipal_init_args mmipal_init_args = MMIPAL_INIT_ARGS_DEFAULT;
    load_mmipal_init_args(&mmipal_init_args);

    /* Initialize IP stack. */
    if (mmipal_init(&mmipal_init_args) != MMIPAL_SUCCESS)
    {
        printf("Error initializing network interface.\n");
        MMOSAL_ASSERT(false);
    }

    mmipal_set_link_status_callback(link_status_callback);

    status = mmwlan_get_version(&version);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);
    printf("Morse firmware version %s, morselib version %s, Morse chip ID 0x%lx\n\n",
           version.morse_fw_version, version.morselib_version, version.morse_chip_id);

    /* Read and display MAC address. */
    status = mmwlan_get_mac_addr(mac_addr);
    if (status != MMWLAN_SUCCESS)
    {
        printf("Failed to get MAC address\n");
        MMOSAL_ASSERT(false);
    }
}

void app_wlan_start(void)
{
    enum mmwlan_status status;

    /* Load Wi-Fi settings from config store */
    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;
    load_mmwlan_sta_args(&sta_args);
    load_mmwlan_settings();

    printf("Attempting to connect to %s ", sta_args.ssid);
    if (sta_args.security_type == MMWLAN_SAE)
    {
        printf("with passphrase %s", sta_args.passphrase);
    }
    printf("\n");
    printf("This may take some time (~30 seconds)\n");

    status = mmwlan_sta_enable(&sta_args, sta_status_callback);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);

    /* Wait for link status callback.
    * Use a binary semaphore to block us until Link is up.
    */
    mmosal_semb_wait(link_established, UINT32_MAX);

    /* Wi-Fi link is now established, return to caller */
}

void app_wlan_stop(void)
{
    /* Shutdown wlan interface */
    mmwlan_shutdown();
}

void app_wlan_arp_send(void)
{
    if (link_up)
    {
        enum mmwlan_status status;
        /* Send a gratuitous ARP to let the router know we are online. */
        uint8_t arp_packet[] = {
            /* 802.3 header: dest_addr, source_addr, ethertype */
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
            0x08, 0x06,

            /* ARP payload */
            0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
            ((uint8_t *)&ip_addr_u32)[0], ((uint8_t *)&ip_addr_u32)[1],
            ((uint8_t *)&ip_addr_u32)[2], ((uint8_t *)&ip_addr_u32)[3],
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ((uint8_t *)&gw_addr_u32)[0], ((uint8_t *)&gw_addr_u32)[1],
            ((uint8_t *)&gw_addr_u32)[2], ((uint8_t *)&gw_addr_u32)[3],
        };
        status = mmwlan_tx(arp_packet, sizeof(arp_packet));
        if (status != MMWLAN_SUCCESS)
        {
            printf("TX failed with status %d\n", status);
            MMOSAL_ASSERT(false);
        }
    }
}
