/*
 * Actuator Output Agent
 * Drives MCPWM outputs for servos and ESCs from actuator_controls topic.
 * Runs on Core 0 at priority 7 (highest FC priority).
 */
#pragma once

/**
 * Create and start the actuator output task.
 * Must be called after orb_init().
 */
void actuator_agent_start(void);
