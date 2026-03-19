/*
 * ICM-42688-P 6-axis IMU driver (SPI)
 * Accelerometer + Gyroscope
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

/* WHO_AM_I expected value */
#define ICM42688P_WHO_AM_I_VAL  0x47

/* ------- Register Map (Bank 0) ------- */
#define ICM42688P_REG_DEVICE_CONFIG     0x11
#define ICM42688P_REG_DRIVE_CONFIG      0x13
#define ICM42688P_REG_INT_CONFIG        0x14
#define ICM42688P_REG_FIFO_CONFIG       0x16
#define ICM42688P_REG_TEMP_DATA1        0x1D
#define ICM42688P_REG_TEMP_DATA0        0x1E
#define ICM42688P_REG_ACCEL_DATA_X1     0x1F
#define ICM42688P_REG_ACCEL_DATA_X0     0x20
#define ICM42688P_REG_ACCEL_DATA_Y1     0x21
#define ICM42688P_REG_ACCEL_DATA_Y0     0x22
#define ICM42688P_REG_ACCEL_DATA_Z1     0x23
#define ICM42688P_REG_ACCEL_DATA_Z0     0x24
#define ICM42688P_REG_GYRO_DATA_X1      0x25
#define ICM42688P_REG_GYRO_DATA_X0      0x26
#define ICM42688P_REG_GYRO_DATA_Y1      0x27
#define ICM42688P_REG_GYRO_DATA_Y0      0x28
#define ICM42688P_REG_GYRO_DATA_Z1      0x29
#define ICM42688P_REG_GYRO_DATA_Z0      0x2A
#define ICM42688P_REG_INT_STATUS        0x2D
#define ICM42688P_REG_PWR_MGMT0        0x4E
#define ICM42688P_REG_GYRO_CONFIG0      0x4F
#define ICM42688P_REG_ACCEL_CONFIG0     0x50
#define ICM42688P_REG_GYRO_CONFIG1      0x51
#define ICM42688P_REG_GYRO_ACCEL_CONFIG0 0x52
#define ICM42688P_REG_ACCEL_CONFIG1     0x53
#define ICM42688P_REG_INT_CONFIG0       0x63
#define ICM42688P_REG_INT_CONFIG1       0x64
#define ICM42688P_REG_INT_SOURCE0       0x65
#define ICM42688P_REG_WHO_AM_I         0x75
#define ICM42688P_REG_BANK_SEL          0x76

/* Configuration handle */
typedef struct {
    spi_device_handle_t spi_dev;
    gpio_num_t          cs_pin;
    spi_host_device_t   spi_host;
} icm42688p_t;

/**
 * Initialise the ICM-42688-P.
 * Configures SPI device on the given host, probes WHO_AM_I.
 * Returns 0 on success, -1 on failure.
 */
int icm42688p_init(icm42688p_t *dev, spi_host_device_t spi_host, gpio_num_t cs_pin);

/**
 * Configure sensor for flight controller use:
 *   Accel: +/-16 g, 1 kHz ODR
 *   Gyro:  +/-2000 dps, 1 kHz ODR
 *   Low-noise mode, BW filter
 * Returns 0 on success.
 */
int icm42688p_configure(icm42688p_t *dev);

/**
 * Burst-read accelerometer, gyroscope and temperature.
 *   accel[3] in m/s^2
 *   gyro[3]  in rad/s
 *   *temp    in degrees C
 * Returns 0 on success.
 */
int icm42688p_read(icm42688p_t *dev, float accel[3], float gyro[3], float *temp);
