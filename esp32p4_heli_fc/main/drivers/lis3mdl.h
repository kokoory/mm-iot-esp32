/*
 * LIS3MDL 3-axis Magnetometer driver (I2C)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

/* Default I2C address (SDO/SA1 high) */
#define LIS3MDL_I2C_ADDR        0x1E

/* WHO_AM_I expected value */
#define LIS3MDL_WHO_AM_I_VAL    0x3D

/* ------- Register Map ------- */
#define LIS3MDL_REG_WHO_AM_I    0x0F
#define LIS3MDL_REG_CTRL_REG1   0x20
#define LIS3MDL_REG_CTRL_REG2   0x21
#define LIS3MDL_REG_CTRL_REG3   0x22
#define LIS3MDL_REG_CTRL_REG4   0x23
#define LIS3MDL_REG_CTRL_REG5   0x24
#define LIS3MDL_REG_STATUS      0x27
#define LIS3MDL_REG_OUT_X_L     0x28
#define LIS3MDL_REG_OUT_X_H     0x29
#define LIS3MDL_REG_OUT_Y_L     0x2A
#define LIS3MDL_REG_OUT_Y_H     0x2B
#define LIS3MDL_REG_OUT_Z_L     0x2C
#define LIS3MDL_REG_OUT_Z_H     0x2D
#define LIS3MDL_REG_TEMP_OUT_L  0x2E
#define LIS3MDL_REG_TEMP_OUT_H  0x2F
#define LIS3MDL_REG_INT_CFG     0x30
#define LIS3MDL_REG_INT_SRC     0x31
#define LIS3MDL_REG_INT_THS_L   0x32
#define LIS3MDL_REG_INT_THS_H   0x33

/* Status register bits */
#define LIS3MDL_STATUS_ZYXDA    0x08    /* XYZ new data available */

/* Device handle */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t                 addr;
} lis3mdl_t;

/**
 * Initialise LIS3MDL on the given I2C bus.
 * Probes WHO_AM_I register.
 * Returns 0 on success.
 */
int lis3mdl_init(lis3mdl_t *dev, i2c_master_bus_handle_t bus, uint8_t addr);

/**
 * Configure for flight controller use:
 *   Ultra-high performance, 80 Hz ODR, ±8 Gauss, continuous mode, BDU
 * Returns 0 on success.
 */
int lis3mdl_configure(lis3mdl_t *dev);

/**
 * Read magnetometer XYZ data.
 *   mag[3] in Gauss
 * Returns 0 on success, -1 on failure/no data ready.
 */
int lis3mdl_read(lis3mdl_t *dev, float mag[3]);
