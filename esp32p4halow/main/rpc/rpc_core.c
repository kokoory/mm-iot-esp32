/*
 * RPC Core - Implementation using FreeRTOS queues
 */
#include "rpc_core.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "rpc_core";

void rpc_init(rpc_context_t *ctx)
{
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
            return -1;
        }
    }
    return 0;
}

int rpc_receive_command(rpc_context_t *ctx, rpc_command_msg_t *msg, uint32_t timeout_ms)
{
    if (ctx == NULL || ctx->cmd_queue == NULL || msg == NULL) {
        return -1;
    }

    BaseType_t ret = xQueueReceive(ctx->cmd_queue, msg, pdMS_TO_TICKS(timeout_ms));
    return (ret == pdTRUE) ? 0 : -1;
}

int rpc_receive_telemetry(rpc_context_t *ctx, rpc_telemetry_msg_t *msg, uint32_t timeout_ms)
{
    if (ctx == NULL || ctx->telem_queue == NULL || msg == NULL) {
        return -1;
    }

    BaseType_t ret = xQueueReceive(ctx->telem_queue, msg, pdMS_TO_TICKS(timeout_ms));
    return (ret == pdTRUE) ? 0 : -1;
}

int rpc_send_command(rpc_context_t *ctx, const rpc_command_msg_t *msg)
{
    if (ctx == NULL || ctx->cmd_queue == NULL || msg == NULL) {
        return -1;
    }

    BaseType_t ret = xQueueSend(ctx->cmd_queue, msg, pdMS_TO_TICKS(100));
    if (ret != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full, msg_type=0x%02x", msg->msg_type);
        return -1;
    }
    return 0;
}
