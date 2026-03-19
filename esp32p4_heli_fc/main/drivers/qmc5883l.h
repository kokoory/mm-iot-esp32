/*
 * QMC5883L 3-axis Magnetometer driver (I2C)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

/* Default I2C address */
#define QMC5883L_I2C_ADDR       0x0D

/* ------- Register Map ------- */
#define QMC5883L_REG_DATA_X_LSB 0x00
#define QMC5883L_REG_DATA_X_MSB 0x01
#define QMC5883L_REG_DATA_Y_LSB 0x02
#define QMC5883L_REG_DATA_Y_MSB 0x03
#define QMC5883L_REG_DATA_Z_LSB 0x04
#define QMC5883L_REG_DATA_Z_MSB 0x05
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_TEMP_LSB   0x07
#define QMC5883L_REG_TEMP_MSB   0x08
#define QMC5883L_REG_CTRL1      0x09
#define QMC5883L_REG_CTRL2      0x0A
#define QMC5883L_REG_SET_RESET  0x0B
#define QMC5883L_REG_CHIP_ID    0x0D

/* Expected chip ID */
#define QMC5883L_CHIP_ID_VAL    0xFF

/* Status register bits */
#define QMC5883L_STATUS_DRDY    0x01    /* Data Ready */
#define QMC5883L_STATUS_OVL     0x02    /* Overflow */

/* Device handle */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t                 addr;
} qmc5883l_t;

/**
 * Initialise QMC5883L on the given I2C bus.
 * Performs soft reset and probes chip ID register.
 * Returns 0 on success.
 */
int qmc5883l_init(qmc5883l_t *dev, i2c_master_bus_handle_t bus, uint8_t addr);

/**
 * Configure for flight controller use:
 *   Continuous mode, 200 Hz ODR, 8 Gauss range, OSR 512
 * Returns 0 on success.
 */
int qmc5883l_configure(qmc5883l_t *dev);

/**
 * Read magnetometer XYZ data.
 *   mag[3] in Gauss
 * Returns 0 on success, -1 on failure/no data ready.
 */
int qmc5883l_read(qmc5883l_t *dev, float mag[3]);
