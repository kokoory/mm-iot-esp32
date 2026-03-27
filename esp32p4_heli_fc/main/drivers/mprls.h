/*
 * MPRLS (Honeywell MicroPressure MPR Series) I2C Driver
 *
 * Differential/gauge pressure sensor for airspeed measurement via pitot tube.
 * Honeywell MPRLS0025PA00001A: 0-25 PSI, I2C address 0x18.
 *
 * Protocol:
 *   1. Send command: 0xAA 0x00 0x00 (start measurement)
 *   2. Wait ~5ms for conversion
 *   3. Read 4 bytes: [status] [data_msb] [data_mid] [data_lsb]
 *   4. Convert 24-bit raw → pressure
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

/* Default I2C address */
#define MPRLS_I2C_ADDR          0x18

/* Transfer function constants (10% to 90% of 2^24) */
#define MPRLS_OUTPUT_MIN        1677722     /* 0x19999A = 10% of 2^24 */
#define MPRLS_OUTPUT_MAX        15099494    /* 0xE66666 = 90% of 2^24 */

/* Pressure range: 0 - 25 PSI (MPRLS0025PA) */
#define MPRLS_PSI_MIN           0.0f
#define MPRLS_PSI_MAX           25.0f

/* Conversion: 1 PSI = 6894.757 Pa */
#define PSI_TO_PA               6894.757f

/* Status byte flags */
#define MPRLS_STATUS_BUSY       (1 << 5)
#define MPRLS_STATUS_MATH_SAT   (1 << 0)
#define MPRLS_STATUS_INTEGRITY  (1 << 2)

/* Device handle */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t                 addr;
    float                   pressure_pa;    /* Last reading in Pa */
    uint64_t                last_read_us;
} mprls_t;

/**
 * Initialize MPRLS on the given I2C bus.
 * Returns 0 on success.
 */
int mprls_init(mprls_t *dev, i2c_master_bus_handle_t bus, uint8_t addr);

/**
 * Read differential pressure in Pascals.
 * Sends measurement command, waits for result, converts to Pa.
 * Returns 0 on success.
 */
int mprls_read(mprls_t *dev, float *pressure_pa);

/**
 * Convert differential pressure (Pa) to indicated airspeed (m/s).
 * Uses simplified Bernoulli: IAS = sqrt(2 * dp / rho)
 * where rho = 1.225 kg/m^3 (sea level standard atmosphere).
 */
float mprls_dp_to_airspeed(float dp_pa);
