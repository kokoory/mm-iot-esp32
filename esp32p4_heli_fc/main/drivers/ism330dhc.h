/*
 * ISM330DHC 6-axis IMU driver (I2C)
 * Accelerometer + Gyroscope
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

/* WHO_AM_I expected value */
#define ISM330DHC_WHO_AM_I_VAL  0x6B

/* ------- Register Map ------- */
#define ISM330DHC_REG_FUNC_CFG_ACCESS   0x01
#define ISM330DHC_REG_INT1_CTRL         0x0D
#define ISM330DHC_REG_INT2_CTRL         0x0E
#define ISM330DHC_REG_WHO_AM_I          0x0F
#define ISM330DHC_REG_CTRL1_XL          0x10
#define ISM330DHC_REG_CTRL2_G           0x11
#define ISM330DHC_REG_CTRL3_C           0x12
#define ISM330DHC_REG_CTRL4_C           0x13
#define ISM330DHC_REG_CTRL5_C           0x14
#define ISM330DHC_REG_CTRL6_C           0x15
#define ISM330DHC_REG_CTRL7_G           0x16
#define ISM330DHC_REG_CTRL8_XL          0x17
#define ISM330DHC_REG_CTRL9_XL          0x18
#define ISM330DHC_REG_CTRL10_C          0x19
#define ISM330DHC_REG_STATUS            0x1E
#define ISM330DHC_REG_OUT_TEMP_L        0x20
#define ISM330DHC_REG_OUT_TEMP_H        0x21
#define ISM330DHC_REG_OUTX_L_G         0x22
#define ISM330DHC_REG_OUTX_H_G         0x23
#define ISM330DHC_REG_OUTY_L_G         0x24
#define ISM330DHC_REG_OUTY_H_G         0x25
#define ISM330DHC_REG_OUTZ_L_G         0x26
#define ISM330DHC_REG_OUTZ_H_G         0x27
#define ISM330DHC_REG_OUTX_L_XL        0x28
#define ISM330DHC_REG_OUTX_H_XL        0x29
#define ISM330DHC_REG_OUTY_L_XL        0x2A
#define ISM330DHC_REG_OUTY_H_XL        0x2B
#define ISM330DHC_REG_OUTZ_L_XL        0x2C
#define ISM330DHC_REG_OUTZ_H_XL        0x2D

/* Configuration handle */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t                 i2c_addr;
} ism330dhc_t;

/**
 * Initialise the ISM330DHC on an existing I2C bus.
 * Probes WHO_AM_I to verify communication.
 * Returns 0 on success, -1 on failure.
 */
int ism330dhc_init(ism330dhc_t *dev, i2c_master_bus_handle_t bus, uint8_t i2c_addr);

/**
 * Configure sensor for flight controller use:
 *   Accel: ±16 g, 1.66 kHz ODR
 *   Gyro:  ±2000 dps, 1.66 kHz ODR
 *   BDU enabled, IF_INC enabled
 * Returns 0 on success.
 */
int ism330dhc_configure(ism330dhc_t *dev);

/**
 * Burst-read accelerometer, gyroscope and temperature.
 *   accel[3] in m/s^2
 *   gyro[3]  in rad/s
 *   *temp    in degrees C
 * Returns 0 on success.
 */
int ism330dhc_read(ism330dhc_t *dev, float accel[3], float gyro[3], float *temp);
