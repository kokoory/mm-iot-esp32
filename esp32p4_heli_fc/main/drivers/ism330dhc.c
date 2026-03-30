/*
 * ISM330DHC/ISM330DHCX 6-axis IMU SPI Driver
 *
 * SPI mode 3 (CPOL=1, CPHA=1), max 10 MHz.
 * Read: bit7 of address byte = 1
 * Write: bit7 of address byte = 0
 */

#include "ism330dhc.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ism330dhc";

/*
 * Scaling constants
 *   Accel ±16g: sensitivity = 0.488 mg/LSB -> m/s^2
 *   Gyro ±2000dps: sensitivity = 70 mdps/LSB -> rad/s
 *   Temp: sensitivity = 256 LSB/°C, offset = 25°C
 */
#define ACCEL_SCALE_16G     (0.000488f * 9.80665f)  /* LSB -> m/s^2 */
#define GYRO_SCALE_2000DPS  (0.070f * (float)M_PI / 180.0f)  /* LSB -> rad/s */
#define TEMP_SCALE          (1.0f / 256.0f)
#define TEMP_OFFSET         25.0f

/* ------------------------------------------------------------------ */
/* Low-level SPI helpers                                               */
/* ------------------------------------------------------------------ */

static esp_err_t ism330dhc_write_reg(ism330dhc_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7F, val };  /* bit7 = 0 for write */
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    return spi_device_transmit(dev->spi_dev, &t);
}

static esp_err_t ism330dhc_read_reg(ism330dhc_t *dev, uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };  /* bit7 = 1 for read */
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

static esp_err_t ism330dhc_read_burst(ism330dhc_t *dev, uint8_t start_reg,
                                       uint8_t *buf, size_t len)
{
    uint8_t tx[1 + 14];  /* max burst = 14 bytes */
    uint8_t rx[1 + 14];
    if (len > 14) return ESP_ERR_INVALID_SIZE;

    memset(tx, 0, sizeof(tx));
    tx[0] = start_reg | 0x80;  /* bit7 = 1 for read */

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

int ism330dhc_init(ism330dhc_t *dev, spi_host_device_t host, int cs_pin)
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

    /* Allow device startup time */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Software reset via CTRL3_C */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Probe WHO_AM_I */
    uint8_t who = 0;
    ret = ism330dhc_read_reg(dev, ISM330DHC_REG_WHO_AM_I, &who);
    if (ret != ESP_OK || who != ISM330DHC_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 who, ISM330DHC_WHO_AM_I_VAL);
        return -1;
    }

    ESP_LOGI(TAG, "ISM330DHC detected (WHO_AM_I=0x%02X) via SPI (CS=%d)",
             who, cs_pin);
    return 0;
}

int ism330dhc_configure(ism330dhc_t *dev)
{
    /* CTRL3_C: IF_INC=1, BDU=1, SPI 4-wire mode (SIM=0) */
    esp_err_t ret = ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL3_C, 0x44);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure CTRL3_C failed"); return -1; }

    /* CTRL1_XL: ODR=1.66kHz, FS=±16g */
    ret = ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL1_XL, 0x84);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure CTRL1_XL failed"); return -1; }

    /* CTRL2_G: ODR=1.66kHz, FS=±2000dps */
    ret = ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL2_G, 0x8C);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure CTRL2_G failed"); return -1; }

    /* CTRL4_C: enable gyro LPF1, I2C disable (SPI only) */
    ret = ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL4_C, 0x06);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure CTRL4_C failed"); return -1; }

    /* CTRL6_C: gyro LPF1 widest BW */
    ret = ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL6_C, 0x00);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure CTRL6_C failed"); return -1; }

    /* CTRL8_XL: accel filter defaults */
    ret = ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL8_XL, 0x00);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "configure CTRL8_XL failed"); return -1; }

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "configured: accel +/-16g, gyro +/-2000dps, ODR 1.66kHz");
    return 0;
}

int ism330dhc_read(ism330dhc_t *dev, float accel[3], float gyro[3], float *temp)
{
    uint8_t buf[14];
    esp_err_t ret = ism330dhc_read_burst(dev, ISM330DHC_REG_OUT_TEMP_L, buf, 14);
    if (ret != ESP_OK) {
        return -1;
    }

    int16_t raw_temp = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_gx = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_gy = (int16_t)(buf[5] << 8 | buf[4]);
    int16_t raw_gz = (int16_t)(buf[7] << 8 | buf[6]);
    int16_t raw_ax = (int16_t)(buf[9]  << 8 | buf[8]);
    int16_t raw_ay = (int16_t)(buf[11] << 8 | buf[10]);
    int16_t raw_az = (int16_t)(buf[13] << 8 | buf[12]);

    accel[0] = (float)raw_ax * ACCEL_SCALE_16G;
    accel[1] = (float)raw_ay * ACCEL_SCALE_16G;
    accel[2] = (float)raw_az * ACCEL_SCALE_16G;

    gyro[0] = (float)raw_gx * GYRO_SCALE_2000DPS;
    gyro[1] = (float)raw_gy * GYRO_SCALE_2000DPS;
    gyro[2] = (float)raw_gz * GYRO_SCALE_2000DPS;

    *temp = ((float)raw_temp * TEMP_SCALE) + TEMP_OFFSET;

    return 0;
}
