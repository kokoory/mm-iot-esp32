/*
 * System Monitor Agent
 * Battery monitoring, sensor health, failsafe, arm/disarm logic, LED indication.
 * Runs on Core 0 at priority 2 (10 Hz).
 */
#pragma once

#include "../rpc/rpc_core.h"

/**
 * Create and start the system monitor task.
 * @param rpc Pointer to the shared RPC context for receiving commands from Core 1.
 *            Can be NULL if RPC is not used.
 */
void sysmon_agent_start(rpc_context_t *rpc);
