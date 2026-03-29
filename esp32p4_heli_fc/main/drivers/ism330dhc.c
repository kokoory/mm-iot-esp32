/*
 * ISM330DHC 6-axis IMU I2C Driver
 *
 * Full implementation with burst reads, proper scaling,
 * and high-performance configuration for flight control.
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
/* Low-level I2C helpers                                               */
/* ------------------------------------------------------------------ */

static esp_err_t ism330dhc_write_reg(ism330dhc_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_dev, buf, 2, 100);
}

static esp_err_t ism330dhc_read_reg(ism330dhc_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, val, 1, 100);
}

static esp_err_t ism330dhc_read_burst(ism330dhc_t *dev, uint8_t start_reg,
                                       uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &start_reg, 1, buf, len, 100);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int ism330dhc_init(ism330dhc_t *dev, i2c_master_bus_handle_t bus, uint8_t i2c_addr)
{
    dev->i2c_addr = i2c_addr;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
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

    ESP_LOGI(TAG, "ISM330DHC detected (WHO_AM_I=0x%02X) at I2C addr 0x%02X",
             who, i2c_addr);
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
     *   => 0x84
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL1_XL, 0x84);

    /*
     * CTRL2_G (0x11): Gyroscope config
     *   [7:4] ODR_G  = 1000 -> 1.66 kHz
     *   [3:2] FS_G   = 11   -> ±2000 dps
     *   => 0x8C
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL2_G, 0x8C);

    /*
     * CTRL4_C (0x13):
     *   [2] LPF1_SEL_G = 1 (enable gyro LPF1)
     *   [1] I2C_disable = 0 (keep I2C enabled)
     *   => 0x04
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL4_C, 0x04);

    /*
     * CTRL6_C (0x15): Gyro LPF1 bandwidth
     *   [2:0] FTYPE = 000 -> widest BW for given ODR
     *   => 0x00
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL6_C, 0x00);

    /*
     * CTRL8_XL (0x17): Accelerometer filter config
     *   => 0x00 (defaults)
     */
    ism330dhc_write_reg(dev, ISM330DHC_REG_CTRL8_XL, 0x00);

    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "configured: accel +/-16g, gyro +/-2000dps, ODR 1.66kHz");
    return 0;
}

int ism330dhc_read(ism330dhc_t *dev, float accel[3], float gyro[3], float *temp)
{
    uint8_t buf[14];
    esp_err_t ret = ism330dhc_read_burst(dev, ISM330DHC_REG_OUT_TEMP_L, buf, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "burst read failed: %s", esp_err_to_name(ret));
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
