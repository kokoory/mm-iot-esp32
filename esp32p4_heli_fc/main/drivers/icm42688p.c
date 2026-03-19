/*
 * ICM-42688-P 6-axis IMU SPI Driver
 *
 * Full implementation with burst reads, proper scaling,
 * and low-noise / low-latency configuration for flight control.
 */

#include "icm42688p.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "icm42688p";

/* SPI clock speeds */
#define ICM42688P_SPI_FREQ_INIT     1000000     /* 1 MHz for register config */
#define ICM42688P_SPI_FREQ_DATA     8000000     /* 8 MHz for data reads */

/* Scaling constants */
#define ACCEL_SCALE_16G     (16.0f * 9.80665f / 32768.0f)   /* ±16g: LSB -> m/s^2 */
#define GYRO_SCALE_2000DPS  (2000.0f / 32768.0f * (M_PI / 180.0f))  /* ±2000dps: LSB -> rad/s */
#define TEMP_SCALE          (1.0f / 132.48f)                  /* LSB -> degC */
#define TEMP_OFFSET         25.0f

/* SPI read/write bit */
#define SPI_READ_BIT        0x80

/* ------------------------------------------------------------------ */
/* Low-level SPI helpers                                               */
/* ------------------------------------------------------------------ */

static esp_err_t icm42688p_write_reg(icm42688p_t *dev, uint8_t reg, uint8_t val)
{
    spi_transaction_t txn = {
        .length = 16,                   /* 2 bytes: reg + data */
        .tx_data = { reg & 0x7F, val }, /* bit 7 = 0 for write */
        .flags = SPI_TRANS_USE_TXDATA,
    };
    return spi_device_polling_transmit(dev->spi_dev, &txn);
}

static esp_err_t icm42688p_read_reg(icm42688p_t *dev, uint8_t reg, uint8_t *val)
{
    spi_transaction_t txn = {
        .length = 16,
        .tx_data = { reg | SPI_READ_BIT, 0x00 },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    esp_err_t ret = spi_device_polling_transmit(dev->spi_dev, &txn);
    if (ret == ESP_OK) {
        *val = txn.rx_data[1];
    }
    return ret;
}

static esp_err_t icm42688p_read_burst(icm42688p_t *dev, uint8_t start_reg,
                                       uint8_t *buf, size_t len)
{
    /*
     * Burst read: first byte out is register | 0x80,
     * followed by (len) dummy bytes. Data comes back starting
     * from the second byte received.
     */
    uint8_t tx[1 + len];
    uint8_t rx[1 + len];
    memset(tx, 0, sizeof(tx));
    tx[0] = start_reg | SPI_READ_BIT;

    spi_transaction_t txn = {
        .length = 8 * (1 + len),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi_dev, &txn);
    if (ret == ESP_OK) {
        memcpy(buf, &rx[1], len);
    }
    return ret;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int icm42688p_init(icm42688p_t *dev, spi_host_device_t spi_host, gpio_num_t cs_pin)
{
    dev->spi_host = spi_host;
    dev->cs_pin = cs_pin;

    /* Add the ICM-42688-P as a device on the SPI bus */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = ICM42688P_SPI_FREQ_DATA,
        .mode = 3,                      /* SPI mode 3: CPOL=1, CPHA=1 */
        .spics_io_num = cs_pin,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        .flags = 0,
    };

    esp_err_t ret = spi_bus_add_device(spi_host, &devcfg, &dev->spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* Allow device startup time (1 ms after power-on) */
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Soft reset */
    icm42688p_write_reg(dev, ICM42688P_REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2));

    /* Select register bank 0 */
    icm42688p_write_reg(dev, ICM42688P_REG_BANK_SEL, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Probe WHO_AM_I */
    uint8_t who = 0;
    ret = icm42688p_read_reg(dev, ICM42688P_REG_WHO_AM_I, &who);
    if (ret != ESP_OK || who != ICM42688P_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X", who, ICM42688P_WHO_AM_I_VAL);
        return -1;
    }

    ESP_LOGI(TAG, "ICM-42688-P detected (WHO_AM_I=0x%02X)", who);
    return 0;
}

int icm42688p_configure(icm42688p_t *dev)
{
    /* Ensure bank 0 */
    icm42688p_write_reg(dev, ICM42688P_REG_BANK_SEL, 0x00);

    /*
     * GYRO_CONFIG0 (0x4F):
     *   [7:5] FS_SEL  = 000 -> ±2000 dps
     *   [3:0] ODR     = 0110 -> 1 kHz
     *   => 0x06
     */
    icm42688p_write_reg(dev, ICM42688P_REG_GYRO_CONFIG0, 0x06);

    /*
     * ACCEL_CONFIG0 (0x50):
     *   [7:5] FS_SEL  = 000 -> ±16 g
     *   [3:0] ODR     = 0110 -> 1 kHz
     *   => 0x06
     */
    icm42688p_write_reg(dev, ICM42688P_REG_ACCEL_CONFIG0, 0x06);

    /*
     * GYRO_CONFIG1 (0x51):
     *   [2:0] FILT_BW = 000 -> BW = ODR/2 (Nyquist)
     *   => 0x00 (default is fine)
     */
    icm42688p_write_reg(dev, ICM42688P_REG_GYRO_CONFIG1, 0x00);

    /*
     * GYRO_ACCEL_CONFIG0 (0x52):
     *   [7:4] accel_ui_filt_bw = 0000 -> BW = ODR/2
     *   [3:0] gyro_ui_filt_bw  = 0000 -> BW = ODR/2
     *   => 0x00
     */
    icm42688p_write_reg(dev, ICM42688P_REG_GYRO_ACCEL_CONFIG0, 0x00);

    /*
     * ACCEL_CONFIG1 (0x53):
     *   Default 0x00 is fine (no averaging)
     */
    icm42688p_write_reg(dev, ICM42688P_REG_ACCEL_CONFIG1, 0x00);

    /*
     * PWR_MGMT0 (0x4E):
     *   [3:2] GYRO_MODE  = 11 -> Low-Noise mode
     *   [1:0] ACCEL_MODE = 11 -> Low-Noise mode
     *   [4]   IDLE       = 0
     *   [5]   TEMP_DIS   = 0
     *   => 0x0F
     */
    icm42688p_write_reg(dev, ICM42688P_REG_PWR_MGMT0, 0x0F);

    /* Wait 200 us for gyro startup (datasheet recommendation) */
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "configured: accel ±16g, gyro ±2000dps, ODR 1kHz, low-noise");
    return 0;
}

int icm42688p_read(icm42688p_t *dev, float accel[3], float gyro[3], float *temp)
{
    /*
     * Burst read 14 bytes starting from TEMP_DATA1 (0x1D):
     *   0x1D TEMP_DATA1  (MSB)
     *   0x1E TEMP_DATA0  (LSB)
     *   0x1F ACCEL_X1    (MSB)
     *   0x20 ACCEL_X0    (LSB)
     *   0x21 ACCEL_Y1
     *   0x22 ACCEL_Y0
     *   0x23 ACCEL_Z1
     *   0x24 ACCEL_Z0
     *   0x25 GYRO_X1
     *   0x26 GYRO_X0
     *   0x27 GYRO_Y1
     *   0x28 GYRO_Y0
     *   0x29 GYRO_Z1
     *   0x2A GYRO_Z0
     */
    uint8_t buf[14];
    esp_err_t ret = icm42688p_read_burst(dev, ICM42688P_REG_TEMP_DATA1, buf, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "burst read failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* Temperature: signed 16-bit, big-endian */
    int16_t raw_temp = (int16_t)((buf[0] << 8) | buf[1]);

    /* Accelerometer: signed 16-bit, big-endian */
    int16_t raw_ax = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_ay = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_az = (int16_t)((buf[6] << 8) | buf[7]);

    /* Gyroscope: signed 16-bit, big-endian */
    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* Convert to physical units */
    accel[0] = (float)raw_ax * ACCEL_SCALE_16G;
    accel[1] = (float)raw_ay * ACCEL_SCALE_16G;
    accel[2] = (float)raw_az * ACCEL_SCALE_16G;

    gyro[0] = (float)raw_gx * GYRO_SCALE_2000DPS;
    gyro[1] = (float)raw_gy * GYRO_SCALE_2000DPS;
    gyro[2] = (float)raw_gz * GYRO_SCALE_2000DPS;

    *temp = ((float)raw_temp * TEMP_SCALE) + TEMP_OFFSET;

    return 0;
}
