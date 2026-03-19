/*
 * HaLow Communication Module - Public API
 *
 * Provides the entry point for Core 1 communication subsystem.
 * Must be called from the main FC project with a shared RPC context.
 */
#pragma once
#include "rpc/rpc_core.h"

/**
 * Start the HaLow communication subsystem on Core 1.
 *
 * Initializes the Morse Micro HaLow radio, connects to AP,
 * sets up the GCS UDP bridge, and starts the MAVLink handler task.
 *
 * @param rpc  Shared RPC context (created by FC on Core 0)
 *
 * This function creates a task pinned to Core 1 and returns immediately.
 */
void halow_comm_start(rpc_context_t *rpc);
