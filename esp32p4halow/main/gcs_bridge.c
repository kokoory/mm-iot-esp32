/*
 * GCS Bridge - Implementation
 *
 * UDP socket communication for MAVLink. Auto-detects GCS IP from
 * the first received packet, then sends directly to that address.
 * Falls back to broadcast if no GCS has been detected yet.
 */
#include "gcs_bridge.h"
#include "esp_log.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

static const char *TAG = "gcs_bridge";

static int s_sock = -1;
static struct sockaddr_in s_gcs_addr;
static bool s_gcs_addr_known = false;
static gcs_bridge_status_t s_status = GCS_BRIDGE_DISCONNECTED;

int gcs_bridge_init(void)
{
    /* Create UDP socket */
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return -1;
    }

    /* Allow broadcast */
    int broadcast = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    /* Allow address reuse */
    int reuse = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    /* Bind to MAVLink port on all interfaces */
    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(GCS_BRIDGE_UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket to port %d: errno %d",
                 GCS_BRIDGE_UDP_PORT, errno);
        close(s_sock);
        s_sock = -1;
        return -1;
    }

    /* Initialize GCS address to broadcast */
    memset(&s_gcs_addr, 0, sizeof(s_gcs_addr));
    s_gcs_addr.sin_family = AF_INET;
    s_gcs_addr.sin_port = htons(GCS_BRIDGE_UDP_PORT);
    s_gcs_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    s_gcs_addr_known = false;

    s_status = GCS_BRIDGE_WAITING_FOR_GCS;
    ESP_LOGI(TAG, "GCS bridge initialized on UDP port %d", GCS_BRIDGE_UDP_PORT);

    return 0;
}

int gcs_bridge_send(const uint8_t *buf, size_t len)
{
    if (s_sock < 0 || buf == NULL || len == 0) {
        return -1;
    }

    int ret = sendto(s_sock, buf, len, 0,
                     (struct sockaddr *)&s_gcs_addr, sizeof(s_gcs_addr));
    if (ret < 0) {
        ESP_LOGW(TAG, "sendto failed: errno %d", errno);
        return -1;
    }

    return ret;
}

int gcs_bridge_recv(uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    if (s_sock < 0 || buf == NULL || max_len == 0) {
        return -1;
    }

    /* Set receive timeout */
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in src_addr;
    socklen_t src_addr_len = sizeof(src_addr);
    memset(&src_addr, 0, sizeof(src_addr));

    int ret = recvfrom(s_sock, buf, max_len, 0,
                       (struct sockaddr *)&src_addr, &src_addr_len);
    if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; /* timeout */
        }
        ESP_LOGW(TAG, "recvfrom failed: errno %d", errno);
        return -1;
    }

    if (ret > 0 && !s_gcs_addr_known) {
        /* Auto-detect GCS IP from first received packet */
        s_gcs_addr.sin_addr = src_addr.sin_addr;
        s_gcs_addr.sin_port = src_addr.sin_port;
        s_gcs_addr_known = true;
        s_status = GCS_BRIDGE_CONNECTED;
        ESP_LOGI(TAG, "GCS detected at %s:%d",
                 inet_ntoa(src_addr.sin_addr), ntohs(src_addr.sin_port));
    } else if (ret > 0) {
        /* Update GCS address if it changed (e.g. GCS reconnect) */
        if (src_addr.sin_addr.s_addr != s_gcs_addr.sin_addr.s_addr ||
            src_addr.sin_port != s_gcs_addr.sin_port) {
            s_gcs_addr.sin_addr = src_addr.sin_addr;
            s_gcs_addr.sin_port = src_addr.sin_port;
            ESP_LOGI(TAG, "GCS address updated to %s:%d",
                     inet_ntoa(src_addr.sin_addr), ntohs(src_addr.sin_port));
        }
    }

    return ret;
}

gcs_bridge_status_t gcs_bridge_get_status(void)
{
    return s_status;
}

void gcs_bridge_deinit(void)
{
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
    s_gcs_addr_known = false;
    s_status = GCS_BRIDGE_DISCONNECTED;
    ESP_LOGI(TAG, "GCS bridge shut down");
}
