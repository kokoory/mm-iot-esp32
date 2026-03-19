/*
 * RPC Core - Inter-core communication via FreeRTOS queues
 */
#pragma once
#include "rpc_messages.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define RPC_TELEM_QUEUE_DEPTH  8
#define RPC_CMD_QUEUE_DEPTH    16

typedef struct {
    QueueHandle_t telem_queue;  /* Core 0 -> Core 1 (telemetry) */
    QueueHandle_t cmd_queue;    /* Core 1 -> Core 0 (commands)  */
} rpc_context_t;

/**
 * Initialize RPC context. Must be called from main before starting tasks.
 * Creates the FreeRTOS queues for inter-core communication.
 */
void rpc_init(rpc_context_t *ctx);

/**
 * Core 0 side: send telemetry to Core 1.
 * Returns 0 on success, -1 if queue full.
 */
int rpc_send_telemetry(rpc_context_t *ctx, const rpc_telemetry_msg_t *msg);

/**
 * Core 0 side: receive a command from Core 1.
 * Returns 0 on success, -1 on timeout.
 */
int rpc_receive_command(rpc_context_t *ctx, rpc_command_msg_t *msg, uint32_t timeout_ms);

/**
 * Core 1 side: receive telemetry from Core 0.
 * Returns 0 on success, -1 on timeout.
 */
int rpc_receive_telemetry(rpc_context_t *ctx, rpc_telemetry_msg_t *msg, uint32_t timeout_ms);

/**
 * Core 1 side: send a command to Core 0.
 * Returns 0 on success, -1 if queue full.
 */
int rpc_send_command(rpc_context_t *ctx, const rpc_command_msg_t *msg);
