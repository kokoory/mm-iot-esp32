/*
 * Flight Controller Agent
 * Runs control loops (attitude, rate, altitude) and publishes actuator commands.
 * Runs on Core 0 at priority 5 (500 Hz).
 */
#pragma once

/**
 * Create and start the flight controller task.
 * Must be called after orb_init() and after sensor_agent_start().
 */
void flight_ctrl_agent_start(void);
