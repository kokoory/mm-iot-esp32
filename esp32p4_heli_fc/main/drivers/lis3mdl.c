/*
 * LIS3MDL 3-axis Magnetometer SPI Driver
 *
 * SPI mode 3 (CPOL=1, CPHA=1), max 10 MHz.
 * Read:  bit7 = 1
 * Write: bit7 = 0
 * Multi-byte: bit6 = 1 (auto-increment)
 */

#include "lis3mdl.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lis3mdl";

/*
 * Scale factor for ±8 Gauss range:
 *   Sensitivity = 3421 LSB/Gauss => 1 LSB = 1/3421 Gauss
 */
#define MAG_SCALE_8G    (1.0f / 3421.0f)

/* ------------------------------------------------------------------ */
/* Low-level SPI helpers                                               */
/* ------------------------------------------------------------------ */

static esp_err_t lis3mdl_write_reg(lis3mdl_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x3F, val };  /* bit7=0 (write), bit6=0 (no inc) */
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    return spi_device_transmit(dev->spi_dev, &t);
}

static esp_err_t lis3mdl_read_reg(lis3mdl_t *dev, uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };  /* bit7=1 (read) */
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t ret = spi_device_transmit(dev->spi_dev, &t);
    if (ret == ESP_OK) *val = rx[1];
    return ret;
}

static esp_err_t lis3mdl_read_regs(lis3mdl_t *dev, uint8_t start_reg,
                                    uint8_t *buf, size_t len)
{
    uint8_t tx[1 + 6];
    uint8_t rx[1 + 6];
    if (len > 6) return ESP_ERR_INVALID_SIZE;

    memset(tx, 0, sizeof(tx));
    tx[0] = start_reg | 0xC0;  /* bit7=1 (read), bit6=1 (auto-increment) */

    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t ret = spi_device_transmit(dev->spi_dev, &t);
    if (ret == ESP_OK) {
        memcpy(buf, &rx[1], len);
    }
    return ret;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int lis3mdl_init(lis3mdl_t *dev, spi_host_device_t host, int cs_pin)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  /* 8 MHz */
        .mode = 3,                            /* SPI mode 3: CPOL=1, CPHA=1 */
        .spics_io_num = cs_pin,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_add_device(host, &devcfg, &dev->spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        return -1;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    /* Software reset via CTRL_REG2 */
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

    ESP_LOGI(TAG, "LIS3MDL detected (WHO_AM_I=0x%02X) via SPI (CS=%d)",
             who, cs_pin);
    return 0;
}

int lis3mdl_configure(lis3mdl_t *dev)
{
    esp_err_t ret;

    /* CTRL_REG1: ultra-high perf X/Y, 80Hz ODR, temp enable */
    ret = lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG1, 0xFC);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure REG1 failed"); return -1; }

    /* CTRL_REG2: ±8 Gauss */
    ret = lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG2, 0x20);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure REG2 failed"); return -1; }

    /* CTRL_REG3: continuous-conversion, SPI 4-wire (SIM=0) */
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

    /* Burst read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H */
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
