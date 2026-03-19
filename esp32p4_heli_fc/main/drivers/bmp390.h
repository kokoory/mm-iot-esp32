/*
 * BMP390 Barometric Pressure Sensor driver (I2C)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

/* Default I2C address (SDO=GND) */
#define BMP390_I2C_ADDR_LOW     0x76
#define BMP390_I2C_ADDR_HIGH    0x77

/* Chip ID */
#define BMP390_CHIP_ID_VAL      0x60

/* ------- Register Map ------- */
#define BMP390_REG_CHIP_ID      0x00
#define BMP390_REG_ERR_REG      0x02
#define BMP390_REG_STATUS       0x03
#define BMP390_REG_DATA_0       0x04    /* press_xlsb */
#define BMP390_REG_DATA_1       0x05    /* press_lsb  */
#define BMP390_REG_DATA_2       0x06    /* press_msb  */
#define BMP390_REG_DATA_3       0x07    /* temp_xlsb  */
#define BMP390_REG_DATA_4       0x08    /* temp_lsb   */
#define BMP390_REG_DATA_5       0x09    /* temp_msb   */
#define BMP390_REG_EVENT        0x10
#define BMP390_REG_INT_STATUS   0x11
#define BMP390_REG_INT_CTRL     0x19
#define BMP390_REG_IF_CONF      0x1A
#define BMP390_REG_PWR_CTRL     0x1B
#define BMP390_REG_OSR          0x1C
#define BMP390_REG_ODR          0x1D
#define BMP390_REG_CONFIG       0x1F
#define BMP390_REG_CMD          0x7E

/* Trimming NVM: 0x31 - 0x45 (21 bytes) */
#define BMP390_REG_NVM_START    0x31
#define BMP390_NVM_LEN          21

/* Trimming parameter structure (from datasheet) */
typedef struct {
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;
} bmp390_calib_t;

/* Device handle */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t                 addr;
    bmp390_calib_t          calib;
    float                   t_lin;  /* linearised temperature for pressure comp */
} bmp390_t;

/**
 * Initialise BMP390 on the given I2C bus.
 * Probes chip ID and reads trimming parameters.
 * Returns 0 on success.
 */
int bmp390_init(bmp390_t *dev, i2c_master_bus_handle_t bus, uint8_t addr);

/**
 * Configure for flight controller use:
 *   Normal mode, pressure OSR x8, temp OSR x1, ODR 50 Hz
 * Returns 0 on success.
 */
int bmp390_configure(bmp390_t *dev);

/**
 * Read compensated pressure (Pa) and temperature (deg C).
 * Returns 0 on success.
 */
int bmp390_read(bmp390_t *dev, float *pressure_pa, float *temperature);

/**
 * Convert pressure in Pa to altitude in meters MSL
 * using the international barometric formula.
 */
float bmp390_calc_altitude(float pressure_pa);
