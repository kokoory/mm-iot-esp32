/*
 * LIS3MDL 3-axis Magnetometer I2C Driver
 *
 * I2C address: 0x1C (SDO/SA1=GND) or 0x1E (SDO/SA1=VDD)
 * WHO_AM_I (0x0F) = 0x3D
 *
 * Multi-byte read: MSB (bit 7) of sub-address must be set to 1
 * for auto-increment. This is an I2C-specific requirement.
 */

#include "lis3mdl.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lis3mdl";

/*
 * Scale factor for +/-8 Gauss range:
 *   Sensitivity = 3421 LSB/Gauss => 1 LSB = 1/3421 Gauss
 */
#define MAG_SCALE_8G    (1.0f / 3421.0f)

#define I2C_TIMEOUT_MS  50

/* ------------------------------------------------------------------ */
/* Low-level I2C helpers                                               */
/* ------------------------------------------------------------------ */

static esp_err_t lis3mdl_write_reg(lis3mdl_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_dev, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t lis3mdl_read_reg(lis3mdl_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, val, 1, I2C_TIMEOUT_MS);
}

static esp_err_t lis3mdl_read_regs(lis3mdl_t *dev, uint8_t start_reg,
                                    uint8_t *buf, size_t len)
{
    /* LIS3MDL I2C requires bit 7 of sub-address set for auto-increment.
     * This is different from ISM330DHCX (which uses IF_INC register bit). */
    uint8_t reg = start_reg | 0x80;
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int lis3mdl_init(lis3mdl_t *dev, i2c_master_bus_handle_t bus, uint8_t i2c_addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,  /* 400kHz fast mode */
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device (0x%02X) failed: %s", i2c_addr, esp_err_to_name(ret));
        return -1;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    /* Software reset via CTRL_REG2 (bit 2 = SOFT_RST) */
    lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG2, 0x04);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Read WHO_AM_I */
    uint8_t who = 0;
    ret = lis3mdl_read_reg(dev, LIS3MDL_REG_WHO_AM_I, &who);
    if (ret != ESP_OK || who != LIS3MDL_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 who, LIS3MDL_WHO_AM_I_VAL);
        return -1;
    }

    ESP_LOGI(TAG, "LIS3MDL detected (WHO_AM_I=0x%02X) via I2C (addr=0x%02X)",
             who, i2c_addr);
    return 0;
}

int lis3mdl_configure(lis3mdl_t *dev)
{
    esp_err_t ret;

    /* CTRL_REG1: ultra-high perf X/Y, 80Hz ODR, temp enable */
    ret = lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG1, 0xFC);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure REG1 failed"); return -1; }

    /* CTRL_REG2: +/-8 Gauss */
    ret = lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG2, 0x20);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure REG2 failed"); return -1; }

    /* CTRL_REG3: continuous-conversion mode */
    ret = lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG3, 0x00);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure REG3 failed"); return -1; }

    /* CTRL_REG4: ultra-high perf Z, little-endian */
    ret = lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG4, 0x0C);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure REG4 failed"); return -1; }

    /* CTRL_REG5: BDU enabled */
    ret = lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG5, 0x40);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure REG5 failed"); return -1; }

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "configured: ultra-high perf, 80Hz, +/-8G, continuous");
    return 0;
}

int lis3mdl_read(lis3mdl_t *dev, float mag[3])
{
    /* Check data-ready */
    uint8_t status = 0;
    esp_err_t ret = lis3mdl_read_reg(dev, LIS3MDL_REG_STATUS, &status);
    if (ret != ESP_OK) return -1;

    if (!(status & LIS3MDL_STATUS_ZYXDA)) return -1;

    /* Burst read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
     * (bit 7 of start address set in lis3mdl_read_regs for auto-increment) */
    uint8_t buf[6];
    ret = lis3mdl_read_regs(dev, LIS3MDL_REG_OUT_X_L, buf, 6);
    if (ret != ESP_OK) return -1;

    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);

    mag[0] = (float)raw_x * MAG_SCALE_8G;
    mag[1] = (float)raw_y * MAG_SCALE_8G;
    mag[2] = (float)raw_z * MAG_SCALE_8G;

    return 0;
}
