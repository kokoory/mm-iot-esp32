/*
 * Copyright 2023 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdbool.h>
#include "lwip/inet.h"
#include "mmosal.h"
#include "mmhal.h"
#include "mmwlan.h"
#include "mmipal.h"
#include "mm_app_common.h"
#include "mm_app_loadconfig.h"

#ifndef DNS_MAX_SERVERS
#define DNS_MAX_SERVERS                 2
#endif

static struct mmosal_semb *link_established = NULL;

static bool link_up = false;
static uint32_t ip_addr_u32 = 0;
static uint32_t gw_addr_u32 = 0;
uint8_t mac_addr[MMWLAN_MAC_ADDR_LEN];

/* TX flow control monitoring */
static volatile bool s_tx_paused = false;
static volatile uint32_t s_tx_pause_count = 0;

static void tx_flow_control_cb(enum mmwlan_tx_flow_control_state state, void *arg)
{
    (void)arg;
    if (state == MMWLAN_TX_PAUSED) {
        s_tx_paused = true;
        s_tx_pause_count++;
        if ((s_tx_pause_count % 20) == 1) {
            printf("HaLow TX PAUSED (count=%lu)\n", (unsigned long)s_tx_pause_count);
        }
    } else {
        s_tx_paused = false;
    }
}

bool app_wlan_tx_is_paused(void)
{
    return s_tx_paused;
}

uint32_t app_wlan_tx_pause_count(void)
{
    return s_tx_pause_count;
}

int32_t app_wlan_get_rssi(void)
{
    return mmwlan_get_rssi();
}

void app_wlan_print_link_stats(void)
{
    int32_t rssi = mmwlan_get_rssi();
    printf("  RSSI: %ld dBm\n", (long)rssi);

    struct mmwlan_rc_stats *rc = mmwlan_get_rc_stats();
    if (rc && rc->n_entries > 0) {
        /* Find the most-used rate (highest total_sent) */
        uint32_t best_idx = 0;
        uint32_t best_sent = 0;
        for (uint32_t i = 0; i < rc->n_entries; i++) {
            if (rc->total_sent[i] > best_sent) {
                best_sent = rc->total_sent[i];
                best_idx = i;
            }
        }
        if (best_sent > 0) {
            uint32_t info = rc->rate_info[best_idx];
            uint32_t bw = info & 0x0F;
            uint32_t mcs = (info >> 4) & 0x0F;
            uint32_t gi = (info >> 8) & 0x01;
            uint32_t success = rc->total_success[best_idx];
            printf("  Active MCS: MCS%lu %s %sMHz (sent=%lu ok=%lu loss=%.1f%%)\n",
                   (unsigned long)mcs,
                   gi ? "SGI" : "LGI",
                   bw == 0 ? "1" : bw == 1 ? "2" : "4",
                   (unsigned long)best_sent,
                   (unsigned long)success,
                   best_sent > 0 ? (1.0f - (float)success / (float)best_sent) * 100.0f : 0.0f);
        }
    }
}

/* PHY data rate table: MCS → kbps for 2MHz BW, LGI
 * Source: IEEE 802.11ah Table S36 */
static const uint32_t s_phy_rate_2mhz_lgi_kbps[] = {
    /* MCS0 */  650,
    /* MCS1 */ 1300,
    /* MCS2 */ 1950,
    /* MCS3 */ 2600,
    /* MCS4 */ 3900,
    /* MCS5 */ 5200,
    /* MCS6 */ 5850,
    /* MCS7 */ 6500,
    /* MCS8 */ 7800,
    /* MCS9 */ 8667,
};

void app_wlan_get_link_quality(app_wlan_link_quality_t *out)
{
    memset(out, 0, sizeof(*out));
    out->mcs = 0xFF;
    out->bw_mhz = 2;
    out->throughput_bps = 95000; /* safe default: ~760 kbps (MCS2 2MHz 30% loss) */

    struct mmwlan_rc_stats *rc = mmwlan_get_rc_stats();
    if (!rc || rc->n_entries == 0) {
        if (rc) mmwlan_free_rc_stats(rc);
        return;
    }

    /* Find the most-used rate (highest total_sent) */
    uint32_t best_idx = 0;
    uint32_t best_sent = 0;
    for (uint32_t i = 0; i < rc->n_entries; i++) {
        if (rc->total_sent[i] > best_sent) {
            best_sent = rc->total_sent[i];
            best_idx = i;
        }
    }

    if (best_sent > 0) {
        uint32_t info = rc->rate_info[best_idx];
        uint32_t bw  = info & 0x0F;
        uint32_t mcs = (info >> 4) & 0x0F;
        uint32_t gi  = (info >> 8) & 0x01;
        uint32_t success = rc->total_success[best_idx];

        out->mcs = (uint8_t)mcs;
        out->bw_mhz = (bw == 0) ? 1 : (bw == 1) ? 2 : 4;
        out->sgi = (uint8_t)gi;
        out->loss_pct = (uint8_t)((1.0f - (float)success / (float)best_sent) * 100.0f);

        /* Compute usable throughput:
         * PHY rate (for 2MHz LGI) × BW scaling × SGI bonus × (1 - loss) × MAC overhead (~60%) */
        uint32_t phy_kbps = (mcs <= 9) ? s_phy_rate_2mhz_lgi_kbps[mcs] : 1950;
        /* BW scaling: 1MHz=0.5x, 2MHz=1x, 4MHz=2x */
        if (out->bw_mhz == 1) phy_kbps /= 2;
        else if (out->bw_mhz == 4) phy_kbps *= 2;
        /* SGI: ~11% faster */
        if (gi) phy_kbps = phy_kbps * 111 / 100;
        /* MAC overhead (~70% of PHY rate is usable data for UDP streaming) */
        uint32_t mac_kbps = phy_kbps * 70 / 100;
        /* Packet loss */
        uint32_t usable_kbps = mac_kbps * (100 - out->loss_pct) / 100;
        /* Convert to bytes/sec */
        out->throughput_bps = usable_kbps * 1000 / 8;
    }

    mmwlan_free_rc_stats(rc);
}

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
        gw_addr_u32 = ipaddr_addr(link_status->gateway);

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

    MMOSAL_ASSERT(link_established == NULL);
    link_established = mmosal_semb_create("link_established");

    mmhal_init();
    mmwlan_init();

    /* BUSY pin workaround - disable power save */
    mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED);

    /* Register TX flow control callback to monitor pool saturation */
    status = mmwlan_register_tx_flow_control_cb(tx_flow_control_cb, NULL);
    printf("TX flow control callback registered (status=%d)\n", status);

    /* Auto rate adaptation: chip picks best MCS/BW/GI based on link quality.
     * Management frames use low MCS for reliable association.
     * Data frames scale up to MCS7+ at close range. */
    status = mmwlan_ate_override_rate_control(MMWLAN_MCS_NONE, MMWLAN_BW_NONE, MMWLAN_GI_NONE);
    printf("Rate control: AUTO (no override) status=%d\n", status);

    /* Enable SGI support for higher throughput when conditions are good */
    mmwlan_set_sgi_enabled(true);

    mmwlan_set_channel_list(load_channel_list());

    struct mmipal_init_args mmipal_init_args = MMIPAL_INIT_ARGS_DEFAULT;
    load_mmipal_init_args(&mmipal_init_args);

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

    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;
    load_mmwlan_sta_args(&sta_args);
    load_mmwlan_settings();

    printf("Attempting to connect to %s ", sta_args.ssid);
    if (sta_args.security_type == MMWLAN_SAE)
        printf("with passphrase %s", sta_args.passphrase);
    printf("\n");
    printf("This may take some time (~30 seconds)\n");

    status = mmwlan_sta_enable(&sta_args, sta_status_callback);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);

    printf("Waiting for link up (IP assignment)...\n");
    while (!mmosal_semb_wait(link_established, 10000))
    {
        printf("  Still waiting for link... (%lu ms elapsed)\n", mmosal_get_time_ms());
    }
    printf("DHCP IP assigned successfully.\n");
}

void app_wlan_stop(void)
{
    mmwlan_shutdown();
}

bool app_wlan_is_connected(void)
{
    return link_up;
}

void app_wlan_arp_send(void)
{
    if (link_up)
    {
        enum mmwlan_status status;
        uint8_t arp_packet[] = {
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
            0x08, 0x06,
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
        }
    }
}
