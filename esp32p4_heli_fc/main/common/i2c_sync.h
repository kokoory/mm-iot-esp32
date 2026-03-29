#pragma once

/*
 * I2C bus synchronization between sensor_agent (Core 0) and camera SCCB (Core 1).
 *
 * Sensor I2C init and camera SCCB init must not run concurrently on the shared
 * I2C_NUM_0 bus; doing so causes NACK errors and ESP_ERR_INVALID_STATE.
 *
 * Order: sensor_agent inits first, signals completion, then camera SCCB runs.
 * After SCCB init, camera uses MIPI-CSI (not I2C) so no further contention.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define I2C_SYNC_SENSORS_DONE_BIT  BIT0

extern EventGroupHandle_t g_i2c_sync_event;

static inline void i2c_sync_init(void)
{
    if (!g_i2c_sync_event) {
        g_i2c_sync_event = xEventGroupCreate();
    }
}

/* Called by sensor_agent after all I2C sensor init is complete */
static inline void i2c_sync_sensors_done(void)
{
    if (g_i2c_sync_event) {
        xEventGroupSetBits(g_i2c_sync_event, I2C_SYNC_SENSORS_DONE_BIT);
    }
}

/* Called by camera_h264 before starting SCCB operations.
 * Waits up to 10 seconds for sensor I2C init to complete. */
static inline void i2c_sync_wait_sensors(void)
{
    if (g_i2c_sync_event) {
        xEventGroupWaitBits(g_i2c_sync_event, I2C_SYNC_SENSORS_DONE_BIT,
                            pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
    }
}
