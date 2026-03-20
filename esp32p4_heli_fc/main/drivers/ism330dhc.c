/*
 * ISM330DHC 6-axis IMU SPI Driver
 *
 * Full implementation with burst reads, proper scaling,
 * and high-performance configuration for flight control.
 *
 * The ISM330DHC is an automotive-grade iNEMO inertial module from ST
 * featuring a 3D accelerometer and 3D gyroscope.
 */

#include "ism330dhc.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ism330dhc";

/* SPI clock speed */
#define ISM330DHC_SPI_FREQ      8000000     /* 8 MHz (max 10 MHz) */

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

/* SPI read/write bit */
#define SPI_READ_BIT        0x80

/* ------------------------------------------------------------------ */
/* Low-level SPI helpers                                               */
/* ------------------------------------------------------------------ */

static esp_err_t ism330dhc_write_reg(ism330dhc_t *dev, uint8_t reg, uint8_t val)
{
    spi_transaction_t txn = {
        .length = 16,
        .tx_data = { reg & 0x7F, val },
        .flags = SPI_TRANS_USE_TXDATA,
    };
    return spi_device_polling_transmit(dev->spi_dev, &txn);
}

static esp_err_t ism330dhc_read_reg(ism330dhc_t *dev, uint8_t reg, uint8_t *val)
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

static esp_err_t ism330dhc_read_burst(ism330dhc_t *dev, uint8_t start_reg,
                                       uint8_t *buf, size_t len)
{
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

int ism330dhc_init(ism330dhc_t *dev, spi_host_device_t spi_host, gpio_num_t cs_pin)
{
    dev->spi_host = spi_host;
    dev->cs_pin = cs_pin;

    /* Add the ISM330DHC as a device on the SPI bus */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = ISM330DHC_SPI_FREQ,
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

    /* Allow device startup time */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Software reset via CTRL3_C */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Probe WHO_AM_I */
    uint8_t who = 0;
    ret = ism330dhc_read_reg(dev, ISM330DHC_REG_WHO_AM_I, &who);
    if (ret != ESP_OK || who != ISM330DHC_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 who, ISM330DHC_WHO_AM_I_VAL);
        return -1;
    }

    ESP_LOGI(TAG, "ISM330DHC detected (WHO_AM_I=0x%02X)", who);
    return 0;
}

int ism330dhc_configure(ism330dhc_t *dev)
{
    /*
     * CTRL3_C (0x12):
     *   [2] IF_INC = 1 (auto-increment address for burst read)
     *   [6] BDU    = 1 (block data update)
     *   => 0x44
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL3_C, 0x44);

    /*
     * CTRL1_XL (0x10): Accelerometer config
     *   [7:4] ODR_XL = 1000 -> 1.66 kHz
     *   [3:2] FS_XL  = 01   -> ±16g
     *   [1]   LPF1_BW_SEL = 0
     *   [0]   (reserved) = 0
     *   => 0x84
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL1_XL, 0x84);

    /*
     * CTRL2_G (0x11): Gyroscope config
     *   [7:4] ODR_G  = 1000 -> 1.66 kHz
     *   [3:2] FS_G   = 11   -> ±2000 dps
     *   [1]   FS_125  = 0
     *   [0]   (reserved) = 0
     *   => 0x8C
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL2_G, 0x8C);

    /*
     * CTRL4_C (0x13):
     *   [1] I2C_disable = 1 (we are using SPI)
     *   => 0x04 (note: bit 2 = LPF1_SEL_G)
     *   Actually bit[2] is LPF1_SEL_G (enable gyro LPF1), bit[1]=I2C_disable
     *   => 0x02 to disable I2C only, or 0x06 to also enable gyro LPF1
     *   Use 0x06: disable I2C + enable gyro LPF1
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL4_C, 0x06);

    /*
     * CTRL6_C (0x15): Gyro LPF1 bandwidth
     *   [2:0] FTYPE = 000 -> widest BW for given ODR
     *   => 0x00
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL6_C, 0x00);

    /*
     * CTRL8_XL (0x17): Accelerometer filter config
     *   [5] HP_SLOPE_XL_EN = 0 (slope filter disabled)
     *   [7:5] = default
     *   => 0x00
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL8_XL, 0x00);

    /* Wait for sensor to settle */
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "configured: accel +/-16g, gyro +/-2000dps, ODR 1.66kHz");
    return 0;
}

int ism330dhc_read(ism330dhc_t *dev, float accel[3], float gyro[3], float *temp)
{
    /*
     * Burst read 14 bytes starting from OUT_TEMP_L (0x20):
     *   0x20 OUT_TEMP_L
     *   0x21 OUT_TEMP_H
     *   0x22 OUTX_L_G (gyro X low)
     *   0x23 OUTX_H_G
     *   0x24 OUTY_L_G
     *   0x25 OUTY_H_G
     *   0x26 OUTZ_L_G
     *   0x27 OUTZ_H_G
     *   0x28 OUTX_L_XL (accel X low)
     *   0x29 OUTX_H_XL
     *   0x2A OUTY_L_XL
     *   0x2B OUTY_H_XL
     *   0x2C OUTZ_L_XL
     *   0x2D OUTZ_H_XL
     */
    uint8_t buf[14];
    esp_err_t ret = ism330dhc_read_burst(dev, ISM330DHC_REG_OUT_TEMP_L, buf, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "burst read failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* Temperature: signed 16-bit, little-endian */
    int16_t raw_temp = (int16_t)(buf[1] << 8 | buf[0]);

    /* Gyroscope: signed 16-bit, little-endian */
    int16_t raw_gx = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_gy = (int16_t)(buf[5] << 8 | buf[4]);
    int16_t raw_gz = (int16_t)(buf[7] << 8 | buf[6]);

    /* Accelerometer: signed 16-bit, little-endian */
    int16_t raw_ax = (int16_t)(buf[9]  << 8 | buf[8]);
    int16_t raw_ay = (int16_t)(buf[11] << 8 | buf[10]);
    int16_t raw_az = (int16_t)(buf[13] << 8 | buf[12]);

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
