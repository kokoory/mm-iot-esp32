#pragma once

/*
 * I2C bus synchronization between camera SCCB (Core 1) and sensor_agent (Core 0).
 *
 * Camera SCCB init and sensor I2C init must not run concurrently on the shared
 * I2C_NUM_0 bus; doing so causes NACK errors and ESP_ERR_INVALID_STATE.
 *
 * camera_h264 calls i2c_sync_camera_done() after SCCB init completes.
 * sensor_agent calls i2c_sync_wait_camera() before starting sensor I2C operations.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define I2C_SYNC_CAMERA_DONE_BIT  BIT0

extern EventGroupHandle_t g_i2c_sync_event;

static inline void i2c_sync_init(void)
{
    if (!g_i2c_sync_event) {
        g_i2c_sync_event = xEventGroupCreate();
    }
}

/* Called by camera_h264 after SCCB init is complete */
static inline void i2c_sync_camera_done(void)
{
    if (g_i2c_sync_event) {
        xEventGroupSetBits(g_i2c_sync_event, I2C_SYNC_CAMERA_DONE_BIT);
    }
}

/* Called by sensor_agent before starting I2C sensor operations.
 * Waits up to 10 seconds for camera SCCB init to complete. */
static inline void i2c_sync_wait_camera(void)
{
    if (g_i2c_sync_event) {
        xEventGroupWaitBits(g_i2c_sync_event, I2C_SYNC_CAMERA_DONE_BIT,
                            pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
    }
}
