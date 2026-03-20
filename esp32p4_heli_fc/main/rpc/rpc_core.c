/*
 * RPC Core - Implementation using FreeRTOS queues
 *
 * Debug statistics are printed every RPC_STATS_INTERVAL_MS to help
 * verify that inter-core communication is healthy.
 */
#include "rpc_core.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "rpc_core";

/* ── Debug statistics ────────────────────────────────────────── */
#define RPC_STATS_INTERVAL_MS  5000  /* print stats every 5 seconds */

static struct {
    uint32_t telem_sent;
    uint32_t telem_recv;
    uint32_t telem_dropped;
    uint32_t cmd_sent;
    uint32_t cmd_recv;
    uint32_t cmd_dropped;
    int64_t  last_print_us;
} s_stats;

static const char *telem_type_name(uint8_t type)
{
    switch (type) {
    case 0x01: return "HEARTBEAT";
    case 0x02: return "ATTITUDE";
    case 0x03: return "GPS";
    case 0x04: return "ALTITUDE";
    case 0x05: return "BATTERY";
    case 0x06: return "STATUS";
    case 0x07: return "RC_CHANNELS";
    default:   return "UNKNOWN";
    }
}

static const char *cmd_type_name(uint8_t type)
{
    switch (type) {
    case 0x80: return "ARM";
    case 0x81: return "DISARM";
    case 0x82: return "SET_MODE";
    case 0x83: return "RC_OVERRIDE";
    case 0x84: return "PARAM_SET";
    case 0x85: return "REBOOT";
    default:   return "UNKNOWN";
    }
}

static void rpc_maybe_print_stats(void)
{
    int64_t now = esp_timer_get_time();
    if ((now - s_stats.last_print_us) >= (RPC_STATS_INTERVAL_MS * 1000LL)) {
        s_stats.last_print_us = now;
        ESP_LOGI(TAG, "[RPC STATS] telem: sent=%lu recv=%lu drop=%lu | "
                 "cmd: sent=%lu recv=%lu drop=%lu | "
                 "telem_q=%u/%d cmd_q=%u/%d",
                 (unsigned long)s_stats.telem_sent,
                 (unsigned long)s_stats.telem_recv,
                 (unsigned long)s_stats.telem_dropped,
                 (unsigned long)s_stats.cmd_sent,
                 (unsigned long)s_stats.cmd_recv,
                 (unsigned long)s_stats.cmd_dropped,
                 0u, RPC_TELEM_QUEUE_DEPTH,   /* filled at call site */
                 0u, RPC_CMD_QUEUE_DEPTH);
    }
}

/* ── Initialization ──────────────────────────────────────────── */

void rpc_init(rpc_context_t *ctx)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.last_print_us = esp_timer_get_time();

    ctx->telem_queue = xQueueCreate(RPC_TELEM_QUEUE_DEPTH,
                                    sizeof(rpc_telemetry_msg_t));
    if (ctx->telem_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        return;
    }

    ctx->cmd_queue = xQueueCreate(RPC_CMD_QUEUE_DEPTH,
                                  sizeof(rpc_command_msg_t));
    if (ctx->cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
        vQueueDelete(ctx->telem_queue);
        ctx->telem_queue = NULL;
        return;
    }

    ESP_LOGI(TAG, "RPC initialized: telem_depth=%d cmd_depth=%d",
             RPC_TELEM_QUEUE_DEPTH, RPC_CMD_QUEUE_DEPTH);
}

/* ── Telemetry (Core 0 -> Core 1) ────────────────────────────── */

int rpc_send_telemetry(rpc_context_t *ctx, const rpc_telemetry_msg_t *msg)
{
    if (ctx == NULL || ctx->telem_queue == NULL || msg == NULL) {
        return -1;
    }

    /* Use a short timeout so Core 0 never blocks for long */
    BaseType_t ret = xQueueSend(ctx->telem_queue, msg, pdMS_TO_TICKS(5));
    if (ret != pdTRUE) {
        /* Queue full - drop oldest and try again */
        rpc_telemetry_msg_t discard;
        xQueueReceive(ctx->telem_queue, &discard, 0);
        ret = xQueueSend(ctx->telem_queue, msg, 0);
        if (ret != pdTRUE) {
            s_stats.telem_dropped++;
            ESP_LOGD(TAG, "telem DROPPED type=0x%02x (%s)",
                     msg->msg_type, telem_type_name(msg->msg_type));
            return -1;
        }
    }

    s_stats.telem_sent++;
    ESP_LOGV(TAG, "telem TX: type=0x%02x (%s) ts=%lu",
             msg->msg_type, telem_type_name(msg->msg_type),
             (unsigned long)msg->timestamp_ms);

    rpc_maybe_print_stats();
    return 0;
}

/* ── Commands (Core 1 -> Core 0) ─────────────────────────────── */

int rpc_receive_command(rpc_context_t *ctx, rpc_command_msg_t *msg, uint32_t timeout_ms)
{
    if (ctx == NULL || ctx->cmd_queue == NULL || msg == NULL) {
        return -1;
    }

    BaseType_t ret = xQueueReceive(ctx->cmd_queue, msg, pdMS_TO_TICKS(timeout_ms));
    if (ret == pdTRUE) {
        s_stats.cmd_recv++;
        ESP_LOGI(TAG, "cmd RX: type=0x%02x (%s) ts=%lu",
                 msg->msg_type, cmd_type_name(msg->msg_type),
                 (unsigned long)msg->timestamp_ms);
        return 0;
    }
    return -1;
}

int rpc_receive_telemetry(rpc_context_t *ctx, rpc_telemetry_msg_t *msg, uint32_t timeout_ms)
{
    if (ctx == NULL || ctx->telem_queue == NULL || msg == NULL) {
        return -1;
    }

    BaseType_t ret = xQueueReceive(ctx->telem_queue, msg, pdMS_TO_TICKS(timeout_ms));
    if (ret == pdTRUE) {
        s_stats.telem_recv++;
        ESP_LOGV(TAG, "telem RX: type=0x%02x (%s) ts=%lu",
                 msg->msg_type, telem_type_name(msg->msg_type),
                 (unsigned long)msg->timestamp_ms);
        return 0;
    }
    return -1;
}

int rpc_send_command(rpc_context_t *ctx, const rpc_command_msg_t *msg)
{
    if (ctx == NULL || ctx->cmd_queue == NULL || msg == NULL) {
        return -1;
    }

    BaseType_t ret = xQueueSend(ctx->cmd_queue, msg, pdMS_TO_TICKS(100));
    if (ret != pdTRUE) {
        s_stats.cmd_dropped++;
        ESP_LOGW(TAG, "cmd DROPPED (queue full): type=0x%02x (%s)",
                 msg->msg_type, cmd_type_name(msg->msg_type));
        return -1;
    }

    s_stats.cmd_sent++;
    ESP_LOGI(TAG, "cmd TX: type=0x%02x (%s) ts=%lu",
             msg->msg_type, cmd_type_name(msg->msg_type),
             (unsigned long)msg->timestamp_ms);

    rpc_maybe_print_stats();
    return 0;
}
