/*
 * Sensor Manager Agent
 * Reads all sensors, runs AHRS and altitude estimator, publishes uORB topics.
 * Runs on Core 0 at priority 6 (1 kHz main loop).
 */
#pragma once

/**
 * Create and start the sensor manager task.
 * Must be called after orb_init().
 */
void sensor_agent_start(void);
