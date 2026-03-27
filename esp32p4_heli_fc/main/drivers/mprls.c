/*
 * MPRLS (Honeywell MicroPressure MPR Series) I2C Driver
 *
 * Reads differential pressure from pitot tube for airspeed measurement.
 * Protocol: command → wait → read 24-bit raw → convert to Pa → airspeed.
 */

#include "mprls.h"
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mprls";

/* Air density at sea level (kg/m^3) */
#define RHO_SEA_LEVEL   1.225f

int mprls_init(mprls_t *dev, i2c_master_bus_handle_t bus, uint8_t addr)
{
    dev->addr = addr;
    dev->pressure_pa = 0.0f;
    dev->last_read_us = 0;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPRLS device at 0x%02X: %s", addr, esp_err_to_name(err));
        return -1;
    }

    /* Verify sensor responds by doing a test read */
    float test_pa;
    int ret = mprls_read(dev, &test_pa);
    if (ret != 0) {
        ESP_LOGE(TAG, "MPRLS test read failed");
        return -1;
    }

    ESP_LOGI(TAG, "MPRLS initialized at 0x%02X, test pressure=%.1f Pa", addr, test_pa);
    return 0;
}

int mprls_read(mprls_t *dev, float *pressure_pa)
{
    esp_err_t err;

    /* Send measurement command: 0xAA 0x00 0x00 */
    uint8_t cmd[3] = { 0xAA, 0x00, 0x00 };
    err = i2c_master_transmit(dev->i2c_dev, cmd, 3, 100);
    if (err != ESP_OK) {
        return -1;
    }

    /* Wait for conversion (~5ms typical) */
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Read 4 bytes: [status] [data_msb] [data_mid] [data_lsb] */
    uint8_t buf[4];
    int retries = 5;
    while (retries-- > 0) {
        err = i2c_master_receive(dev->i2c_dev, buf, 4, 100);
        if (err != ESP_OK) {
            return -1;
        }

        /* Check if still busy */
        if (!(buf[0] & MPRLS_STATUS_BUSY)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (buf[0] & MPRLS_STATUS_BUSY) {
        ESP_LOGW(TAG, "MPRLS still busy after retries");
        return -1;
    }

    /* Check for math saturation or integrity failure */
    if (buf[0] & (MPRLS_STATUS_MATH_SAT | MPRLS_STATUS_INTEGRITY)) {
        ESP_LOGW(TAG, "MPRLS status error: 0x%02X", buf[0]);
        return -1;
    }

    /* Assemble 24-bit raw value */
    uint32_t raw = ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];

    /* Transfer function: pressure = (raw - out_min) / (out_max - out_min) * (psi_max - psi_min) + psi_min */
    float psi = ((float)raw - (float)MPRLS_OUTPUT_MIN) /
                ((float)MPRLS_OUTPUT_MAX - (float)MPRLS_OUTPUT_MIN) *
                (MPRLS_PSI_MAX - MPRLS_PSI_MIN) + MPRLS_PSI_MIN;

    /* Convert PSI to Pascals */
    float pa = psi * PSI_TO_PA;

    dev->pressure_pa = pa;
    dev->last_read_us = (uint64_t)esp_timer_get_time();
    *pressure_pa = pa;

    return 0;
}

float mprls_dp_to_airspeed(float dp_pa)
{
    /* Negative differential pressure means reverse flow — clamp to zero */
    if (dp_pa <= 0.0f) {
        return 0.0f;
    }

    /* Bernoulli: IAS = sqrt(2 * dp / rho) */
    return sqrtf(2.0f * dp_pa / RHO_SEA_LEVEL);
}
