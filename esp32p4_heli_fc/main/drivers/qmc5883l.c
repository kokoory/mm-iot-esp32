/*
 * QMC5883L 3-axis Magnetometer I2C Driver
 *
 * Full implementation with proper scaling for 8 Gauss range.
 */

#include "qmc5883l.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "qmc5883l";

/*
 * Scale factor for 8 Gauss range:
 *   Sensitivity = 3000 LSB/Gauss  =>  1 LSB = 1/3000 Gauss
 */
#define MAG_SCALE_8G    (1.0f / 3000.0f)

/* ------------------------------------------------------------------ */
/* Low-level I2C helpers                                               */
/* ------------------------------------------------------------------ */

static esp_err_t qmc5883l_write_reg(qmc5883l_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_dev, buf, 2, 100);
}

static esp_err_t qmc5883l_read_reg(qmc5883l_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, val, 1, 100);
}

static esp_err_t qmc5883l_read_regs(qmc5883l_t *dev, uint8_t start_reg,
                                     uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &start_reg, 1, buf, len, 100);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int qmc5883l_init(qmc5883l_t *dev, i2c_master_bus_handle_t bus, uint8_t addr)
{
    dev->addr = addr;

    /* Add device to I2C bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        return -1;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    /*
     * Soft reset via CTRL2 register (0x0A):
     *   bit 7 = SOFT_RST
     *   => 0x80
     */
    qmc5883l_write_reg(dev, QMC5883L_REG_CTRL2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    /*
     * Write recommended value to SET/RESET period register:
     *   0x0B = 0x01 (datasheet recommendation)
     */
    qmc5883l_write_reg(dev, QMC5883L_REG_SET_RESET, 0x01);

    /* Read chip ID (register 0x0D should return 0xFF) */
    uint8_t chip_id = 0;
    ret = qmc5883l_read_reg(dev, QMC5883L_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK || chip_id != QMC5883L_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "chip ID mismatch: got 0x%02X, expected 0x%02X",
                 chip_id, QMC5883L_CHIP_ID_VAL);
        return -1;
    }

    ESP_LOGI(TAG, "QMC5883L detected (chip ID=0x%02X)", chip_id);
    return 0;
}

int qmc5883l_configure(qmc5883l_t *dev)
{
    /*
     * CTRL1 register (0x09):
     *   [1:0] MODE = 01    -> Continuous
     *   [3:2] ODR  = 11    -> 200 Hz
     *   [5:4] RNG  = 01    -> 8 Gauss
     *   [7:6] OSR  = 00    -> 512 (highest oversampling)
     *
     *   Bit layout: OSR[1:0] | RNG[1:0] | ODR[1:0] | MODE[1:0]
     *             = 00       | 01       | 11       | 01
     *             = 0x1D
     */
    qmc5883l_write_reg(dev, QMC5883L_REG_CTRL1, 0x1D);

    /*
     * CTRL2 register (0x0A):
     *   [0] INT_ENB = 0  -> interrupt disabled
     *   [6] ROL_PNT = 1  -> pointer roll-over enabled
     *   => 0x40
     */
    qmc5883l_write_reg(dev, QMC5883L_REG_CTRL2, 0x40);

    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "configured: continuous, 200Hz, 8G, OSR 512");
    return 0;
}

int qmc5883l_read(qmc5883l_t *dev, float mag[3])
{
    /* Check data-ready */
    uint8_t status = 0;
    esp_err_t ret = qmc5883l_read_reg(dev, QMC5883L_REG_STATUS, &status);
    if (ret != ESP_OK) {
        return -1;
    }

    if (!(status & QMC5883L_STATUS_DRDY)) {
        return -1;  /* no new data yet */
    }

    /*
     * Burst read 6 bytes starting at DATA_X_LSB (0x00):
     *   [0] X LSB
     *   [1] X MSB
     *   [2] Y LSB
     *   [3] Y MSB
     *   [4] Z LSB
     *   [5] Z MSB
     *
     * Data is signed 16-bit, little-endian.
     */
    uint8_t buf[6];
    ret = qmc5883l_read_regs(dev, QMC5883L_REG_DATA_X_LSB, buf, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "data read failed: %s", esp_err_to_name(ret));
        return -1;
    }

    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);

    /* Check for overflow */
    if (status & QMC5883L_STATUS_OVL) {
        ESP_LOGW(TAG, "magnetic field overflow detected");
    }

    /* Convert to Gauss */
    mag[0] = (float)raw_x * MAG_SCALE_8G;
    mag[1] = (float)raw_y * MAG_SCALE_8G;
    mag[2] = (float)raw_z * MAG_SCALE_8G;

    return 0;
}
