/*
 * BMP390 Barometric Pressure Sensor I2C Driver
 *
 * Full implementation with trimming coefficient readout,
 * compensated pressure/temperature, and altitude calculation.
 *
 * Compensation algorithm follows the BMP390 datasheet (Bosch Sensortec).
 */

#include "bmp390.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmp390";

/* Sea-level standard pressure in Pa */
#define SEA_LEVEL_PRESSURE  101325.0f

/* ------------------------------------------------------------------ */
/* Low-level I2C helpers                                               */
/* ------------------------------------------------------------------ */

static esp_err_t bmp390_write_reg(bmp390_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_dev, buf, 2, 100);
}

static esp_err_t bmp390_read_reg(bmp390_t *dev, uint8_t reg, uint8_t *val)
{
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, val, 1, 100);
    return ret;
}

static esp_err_t bmp390_read_regs(bmp390_t *dev, uint8_t start_reg,
                                   uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &start_reg, 1, buf, len, 100);
}

/* ------------------------------------------------------------------ */
/* Trimming coefficient parsing                                        */
/* ------------------------------------------------------------------ */

/*
 * Parse the 21-byte NVM block (registers 0x31-0x45) into float
 * calibration coefficients per the BMP390 datasheet.
 *
 * NVM layout (little-endian unless noted):
 *   [0..1]   nvm_par_t1   uint16
 *   [2..3]   nvm_par_t2   uint16
 *   [4]      nvm_par_t3   int8
 *   [5..6]   nvm_par_p1   int16
 *   [7..8]   nvm_par_p2   int16
 *   [9]      nvm_par_p3   int8
 *   [10]     nvm_par_p4   int8
 *   [11..12] nvm_par_p5   uint16
 *   [13..14] nvm_par_p6   uint16
 *   [15]     nvm_par_p7   int8
 *   [16]     nvm_par_p8   int8
 *   [17..18] nvm_par_p9   int16
 *   [19]     nvm_par_p10  int8
 *   [20]     nvm_par_p11  int8
 */
static void bmp390_parse_calib(bmp390_t *dev, const uint8_t *nvm)
{
    bmp390_calib_t *c = &dev->calib;

    uint16_t nvm_par_t1 = (uint16_t)(nvm[1] << 8 | nvm[0]);
    uint16_t nvm_par_t2 = (uint16_t)(nvm[3] << 8 | nvm[2]);
    int8_t   nvm_par_t3 = (int8_t)nvm[4];

    int16_t  nvm_par_p1  = (int16_t)(nvm[6]  << 8 | nvm[5]);
    int16_t  nvm_par_p2  = (int16_t)(nvm[8]  << 8 | nvm[7]);
    int8_t   nvm_par_p3  = (int8_t)nvm[9];
    int8_t   nvm_par_p4  = (int8_t)nvm[10];
    uint16_t nvm_par_p5  = (uint16_t)(nvm[12] << 8 | nvm[11]);
    uint16_t nvm_par_p6  = (uint16_t)(nvm[14] << 8 | nvm[13]);
    int8_t   nvm_par_p7  = (int8_t)nvm[15];
    int8_t   nvm_par_p8  = (int8_t)nvm[16];
    int16_t  nvm_par_p9  = (int16_t)(nvm[18] << 8 | nvm[17]);
    int8_t   nvm_par_p10 = (int8_t)nvm[19];
    int8_t   nvm_par_p11 = (int8_t)nvm[20];

    /* Convert to floating point per datasheet formulas */
    c->par_t1  = (float)nvm_par_t1  / powf(2, -8);     /* * 2^8   */
    c->par_t2  = (float)nvm_par_t2  / powf(2, 30);     /* / 2^30  */
    c->par_t3  = (float)nvm_par_t3  / powf(2, 48);     /* / 2^48  */

    c->par_p1  = ((float)nvm_par_p1  - powf(2, 14)) / powf(2, 20);
    c->par_p2  = ((float)nvm_par_p2  - powf(2, 14)) / powf(2, 29);
    c->par_p3  = (float)nvm_par_p3  / powf(2, 32);
    c->par_p4  = (float)nvm_par_p4  / powf(2, 37);
    c->par_p5  = (float)nvm_par_p5  / powf(2, -3);     /* * 2^3   */
    c->par_p6  = (float)nvm_par_p6  / powf(2, 6);
    c->par_p7  = (float)nvm_par_p7  / powf(2, 8);
    c->par_p8  = (float)nvm_par_p8  / powf(2, 15);
    c->par_p9  = (float)nvm_par_p9  / powf(2, 48);
    c->par_p10 = (float)nvm_par_p10 / powf(2, 48);
    c->par_p11 = (float)nvm_par_p11 / powf(2, 65);

    ESP_LOGI(TAG, "calibration: T1=%.2f T2=%.10f T3=%.15f",
             c->par_t1, c->par_t2, c->par_t3);
}

/* ------------------------------------------------------------------ */
/* Compensation (from datasheet)                                       */
/* ------------------------------------------------------------------ */

static float bmp390_compensate_temp(bmp390_t *dev, uint32_t raw_temp)
{
    bmp390_calib_t *c = &dev->calib;

    float partial1 = (float)raw_temp - c->par_t1;
    float partial2 = partial1 * c->par_t2;

    dev->t_lin = partial2 + (partial1 * partial1) * c->par_t3;
    return dev->t_lin;
}

static float bmp390_compensate_press(bmp390_t *dev, uint32_t raw_press)
{
    bmp390_calib_t *c = &dev->calib;
    float t = dev->t_lin;  /* must call compensate_temp first */

    float partial1 = c->par_p6 * t;
    float partial2 = c->par_p7 * (t * t);
    float partial3 = c->par_p8 * (t * t * t);
    float partial_out1 = c->par_p5 + partial1 + partial2 + partial3;

    float partial4 = c->par_p2 * t;
    float partial5 = c->par_p3 * (t * t);
    float partial6 = c->par_p4 * (t * t * t);
    float partial_out2 = (float)raw_press *
                         (c->par_p1 + partial4 + partial5 + partial6);

    float partial7 = (float)raw_press * (float)raw_press;
    float partial8 = c->par_p9 + c->par_p10 * t;
    float partial9 = partial7 * partial8;
    float partial10 = partial9 +
                      ((float)raw_press * (float)raw_press * (float)raw_press) * c->par_p11;

    float comp_press = partial_out1 + partial_out2 + partial10;
    return comp_press;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int bmp390_init(bmp390_t *dev, i2c_master_bus_handle_t bus, uint8_t addr)
{
    dev->addr = addr;
    dev->t_lin = 0.0f;

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

    /* Soft reset */
    bmp390_write_reg(dev, BMP390_REG_CMD, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Probe chip ID */
    uint8_t chip_id = 0;
    ret = bmp390_read_reg(dev, BMP390_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK || chip_id != BMP390_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "chip ID mismatch: got 0x%02X, expected 0x%02X",
                 chip_id, BMP390_CHIP_ID_VAL);
        return -1;
    }

    ESP_LOGI(TAG, "BMP390 detected (chip ID=0x%02X)", chip_id);

    /* Read trimming NVM coefficients */
    uint8_t nvm[BMP390_NVM_LEN];
    ret = bmp390_read_regs(dev, BMP390_REG_NVM_START, nvm, BMP390_NVM_LEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to read NVM trimming data");
        return -1;
    }

    bmp390_parse_calib(dev, nvm);
    return 0;
}

int bmp390_configure(bmp390_t *dev)
{
    /*
     * OSR register (0x1C):
     *   [2:0] osr_p = 011 -> pressure oversampling x8
     *   [5:3] osr_t = 000 -> temperature oversampling x1
     *   => 0x03
     */
    bmp390_write_reg(dev, BMP390_REG_OSR, 0x03);

    /*
     * ODR register (0x1D):
     *   [4:0] odr_sel = 0x02 -> 50 Hz (prescaler = 4 from 200 Hz base)
     *   Value 0x02 selects ~50 Hz output
     */
    bmp390_write_reg(dev, BMP390_REG_ODR, 0x02);

    /*
     * CONFIG register (0x1F):
     *   [3:1] iir_filter = 010 -> coefficient 3 (mild filtering)
     *   => 0x04
     */
    bmp390_write_reg(dev, BMP390_REG_CONFIG, 0x04);

    /*
     * PWR_CTRL register (0x1B):
     *   [0] press_en = 1
     *   [1] temp_en  = 1
     *   [5:4] mode   = 11 -> normal mode
     *   => 0x33
     */
    bmp390_write_reg(dev, BMP390_REG_PWR_CTRL, 0x33);

    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "configured: OSR p=x8, t=x1, ODR=50Hz, IIR=3, normal mode");
    return 0;
}

int bmp390_read(bmp390_t *dev, float *pressure_pa, float *temperature)
{
    /*
     * Read 6 bytes: pressure (3) + temperature (3) starting at DATA_0 (0x04)
     *   [0] press_xlsb  (bits 7:0)
     *   [1] press_lsb   (bits 15:8)
     *   [2] press_msb   (bits 23:16)
     *   [3] temp_xlsb
     *   [4] temp_lsb
     *   [5] temp_msb
     */
    uint8_t buf[6];
    esp_err_t ret = bmp390_read_regs(dev, BMP390_REG_DATA_0, buf, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "data read failed: %s", esp_err_to_name(ret));
        return -1;
    }

    uint32_t raw_press = (uint32_t)buf[2] << 16 | (uint32_t)buf[1] << 8 | buf[0];
    uint32_t raw_temp  = (uint32_t)buf[5] << 16 | (uint32_t)buf[4] << 8 | buf[3];

    /* Temperature must be compensated first (sets t_lin for pressure) */
    float comp_temp  = bmp390_compensate_temp(dev, raw_temp);
    float comp_press = bmp390_compensate_press(dev, raw_press);

    *temperature = comp_temp;
    *pressure_pa = comp_press;

    return 0;
}

float bmp390_calc_altitude(float pressure_pa)
{
    /*
     * International barometric formula:
     *   h = 44330.0 * (1.0 - (P / P0)^(1/5.255))
     *
     * where P0 = 101325 Pa (sea-level standard)
     */
    if (pressure_pa <= 0.0f) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(pressure_pa / SEA_LEVEL_PRESSURE, 0.1903f));
}
