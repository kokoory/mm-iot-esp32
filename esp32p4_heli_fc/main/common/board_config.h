#pragma once

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

/*
 * ESP32-P4 Helicopter FC - Pin Assignment
 *
 * Board: Waveshare ESP32-P4-Module-DEV-KIT
 * Available GPIOs: 2,3,4,5,7,8,20,21,22,23,24,25,26,27,28,29,30,31,32,33,
 *                  46,47,48,49,50,51,52
 *
 * HaLow (MMECH06): GPIO 2,3,4,5,20,21,23,32 (reference branch config)
 * I2C (shared):    GPIO 7=SDA, 8=SCL
 *
 * Spare: GPIO 28, 29, 30
 */

/* ── IMU (ISM330DHC) - I2C ───────────────────────────────────── */
#define ISM330DHC_I2C_ADDR  0x6A      /* SDO/SA0 = GND */

/* ── I2C0 (Baro + Mag + MPRLS + Camera SCCB) ────────────────── */
#define PIN_I2C_SDA         7
#define PIN_I2C_SCL         8
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         100000    /* 100kHz — internal pull-ups too weak for 400kHz */

/* ── GPS (NMEA UART) ─────────────────────────────────────────── */
#define PIN_GPS_TX          33
#define PIN_GPS_RX          22
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
#define PIN_MAIN_ESC        51        /* Main rotor ESC signal */

/* ── Battery ADC ─────────────────────────────────────────────── */
#define PIN_BATT_ADC        52
#define BATT_ADC_ATTEN      ADC_ATTEN_DB_12
#define BATT_VOLTAGE_DIVIDER_RATIO  11.0f

/* ── Status LED ──────────────────────────────────────────────── */
#define PIN_STATUS_LED      31

/* ── I2C Addresses ───────────────────────────────────────────── */
#define BMP390_I2C_ADDR     0x77
#define LIS3MDL_I2C_ADDR    0x1C      /* SDO/SA1 = GND (confirmed by bus scan) */
#define MPRLS_I2C_ADDR      0x18      /* Honeywell MPRLS differential pressure */

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
#define ACTUATOR_TASK_STACK    8192
#define SYSMON_TASK_STACK      8192

/* ── Core Assignment ─────────────────────────────────────────── */
#define FC_CORE     0   /* Flight controller on Core 0 */
#define COMM_CORE   1   /* Communication on Core 1 */
