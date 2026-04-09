#pragma once

/*
 * Synchronization between sensor_agent (Core 0) and comm tasks (Core 1).
 *
 * Sensor SPI init must complete before thermal camera adds its SPI device
 * to the shared SPI3_HOST bus. Also ensures I2C bus is ready before Lepton
 * CCI operations.
 *
 * Order: sensor_agent inits SPI sensors first, signals completion, then
 * thermal camera adds Lepton SPI device and creates I2C bus for CCI.
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

/* Called by thermal camera before adding SPI device / starting I2C CCI.
 * Waits up to 10 seconds for sensor SPI init to complete. */
static inline void i2c_sync_wait_sensors(void)
{
    if (g_i2c_sync_event) {
        xEventGroupWaitBits(g_i2c_sync_event, I2C_SYNC_SENSORS_DONE_BIT,
                            pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
    }
}
