#pragma once

/*
 * Synchronization between sensor_agent (Core 0) and thermal camera (Core 1).
 *
 * sensor_agent creates the shared I2C bus and initializes IMU/MAG.
 * thermal_camera waits for I2C bus to be ready before using it for Lepton CCI.
 * thermal_camera also creates its own dedicated SPI bus for Lepton VoSPI.
 *
 * Order:
 *   1. sensor_agent creates I2C bus + inits IMU/MAG → signals done
 *   2. thermal_camera waits for signal → creates SPI bus + uses I2C for CCI
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

/* Called by sensor_agent after I2C bus creation + sensor init is complete */
static inline void i2c_sync_sensors_done(void)
{
    if (g_i2c_sync_event) {
        xEventGroupSetBits(g_i2c_sync_event, I2C_SYNC_SENSORS_DONE_BIT);
    }
}

/* Called by thermal_camera before using I2C bus for Lepton CCI.
 * Waits up to 10 seconds for sensor_agent to create the I2C bus. */
static inline void i2c_sync_wait_sensors(void)
{
    if (g_i2c_sync_event) {
        xEventGroupWaitBits(g_i2c_sync_event, I2C_SYNC_SENSORS_DONE_BIT,
                            pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
    }
}
