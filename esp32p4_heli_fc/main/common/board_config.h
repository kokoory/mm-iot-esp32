#pragma once

#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

/*
 * ESP32-P4 Helicopter FC - Pin Assignment
 *
 * Available GPIOs: 52,51,31,30,29,28,50,49,5,4,47,48,46,33,27,26,22,21,20
 *   (excludes HaLow: GPIO 0,1,2,3,23,24,32,36)
 *   (dedicated I2C: GPIO 7=SDA, 8=SCL)
 *
 * Spare: GPIO 22, 26, 31, 51, 52
 */

/* ── IMU (ISM330DHC) - SPI3 ──────────────────────────────────── */
#define PIN_IMU_SPI_SCK     4
#define PIN_IMU_SPI_MOSI    5
#define PIN_IMU_SPI_MISO    21
#define PIN_IMU_SPI_CS      20
#define IMU_SPI_HOST        SPI3_HOST
#define IMU_SPI_FREQ_HZ     8000000   /* 8 MHz (ISM330DHC max 10 MHz) */

/* ── I2C0 (Baro BMP390 + Mag LIS3MDL) ───────────────────────── */
#define PIN_I2C_SDA         7
#define PIN_I2C_SCL         8
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000

/* ── GPS (NMEA UART) ─────────────────────────────────────────── */
#define PIN_GPS_TX          27
#define PIN_GPS_RX          33
#define GPS_UART_NUM        UART_NUM_1
#define GPS_BAUD_RATE       9600

/* ── SBUS RC Receiver (inverted UART) ────────────────────────── */
#define PIN_SBUS_RX         46
#define SBUS_UART_NUM       UART_NUM_2
#define SBUS_BAUD_RATE      100000    /* SBUS: 100kbps, 8E2 */

/* ── PWM - MCPWM (Servos + ESCs) ─────────────────────────────── */
#define PIN_SWASH_SERVO_1   47        /* CCPM swashplate servo 1 (0 deg) */
#define PIN_SWASH_SERVO_2   48        /* CCPM swashplate servo 2 (120 deg) */
#define PIN_SWASH_SERVO_3   49        /* CCPM swashplate servo 3 (240 deg) */
#define PIN_TAIL_ESC        50        /* Tail rotor ESC signal */
#define PIN_MAIN_ESC        28        /* Main rotor ESC signal */

/* ── Battery ADC ─────────────────────────────────────────────── */
#define PIN_BATT_ADC        29
#define BATT_ADC_ATTEN      ADC_ATTEN_DB_12
#define BATT_VOLTAGE_DIVIDER_RATIO  11.0f

/* ── Status LED ──────────────────────────────────────────────── */
#define PIN_STATUS_LED      30

/* ── I2C Addresses ───────────────────────────────────────────── */
#define BMP390_I2C_ADDR     0x77
#define LIS3MDL_I2C_ADDR    0x1E

/* ── Task Priorities ─────────────────────────────────────────── */
#define SENSOR_TASK_PRIORITY    6
#define FLIGHT_CTRL_PRIORITY    5
#define ACTUATOR_TASK_PRIORITY  7
#define SYSMON_TASK_PRIORITY    2
#define HALOW_TASK_PRIORITY     4
#define MAVLINK_TASK_PRIORITY   3

/* ── Task Stack Sizes ────────────────────────────────────────── */
#define SENSOR_TASK_STACK      8192
#define FLIGHT_CTRL_STACK      8192
#define ACTUATOR_TASK_STACK    4096
#define SYSMON_TASK_STACK      4096

/* ── Core Assignment ─────────────────────────────────────────── */
#define FC_CORE     0   /* Flight controller on Core 0 */
#define COMM_CORE   1   /* Communication on Core 1 */
