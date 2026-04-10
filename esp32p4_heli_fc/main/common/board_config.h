#pragma once

#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

/*
 * ESP32-P4 Helicopter FC - Pin Assignment
 *
 * Board: Waveshare ESP32-P4-Module-DEV-KIT
 * Available GPIOs: 2,3,4,5,7,8,20,21,22,23,24,25,26,27,28,29,30,31,32,33,
 *                  46,47,48,49,50,51,52
 *
 * HaLow (MMECH06):   GPIO 2,3,4,5,20,21,23,32 (SPI2_HOST)
 * Lepton VoSPI:      GPIO 22=SCLK, 30=MISO, 31=CS (SPI3_HOST, dedicated)
 * Sensor + Lepton I2C: GPIO 7=SDA, 8=SCL (I2C0, shared)
 */

/* ── I2C Bus (ISM330DHCX + LIS3MDL + Lepton CCI) ──────────────────── */
#define PIN_I2C_SDA         7
#define PIN_I2C_SCL         8
#define I2C_PORT            I2C_NUM_0
#define I2C_SENSOR_FREQ_HZ  400000    /* 400kHz — IMU/MAG fast mode */
#define I2C_LEPTON_FREQ_HZ  100000    /* 100kHz — Lepton CCI */

/* IMU: ISM330DHCX (SA0=GND → 0x6A, SA0=VDD → 0x6B) */
#define IMU_I2C_ADDR        0x6A

/* Magnetometer: LIS3MDL (SDO/SA1=GND → 0x1C, SDO/SA1=VDD → 0x1E) */
#define MAG_I2C_ADDR        0x1C

/* ── Thermal Camera: FLIR Lepton 3.5 (PureThermal Breakout Board) ─── */
/* VoSPI: SPI3_HOST (dedicated — no other devices on this bus)
 * CCI:   I2C0 shared with IMU/MAG (addr 0x2A, no conflict)
 * EN:    Active HIGH — controls Lepton power on PureThermal Breakout
 *
 * PureThermal Breakout Board pinout:
 *   1=SCL  2=SDA  3=VIN  4=GND  5=CLK
 *   6=MISO 7=MOSI(NC) 8=CS 9=VSYNC 10=EN
 */
#define LEPTON_SPI_HOST     SPI3_HOST
#define PIN_LEPTON_SPI_SCLK 22
#define PIN_LEPTON_SPI_MISO 30
#define PIN_LEPTON_CS       31
#define PIN_LEPTON_EN       28        /* PureThermal EN (active HIGH) */
#define LEPTON_SPI_FREQ     20000000  /* 20 MHz max per Lepton datasheet */
#define LEPTON_I2C_ADDR     0x2A      /* 7-bit CCI address */

/* ── GPS (NMEA UART) ─────────────────────────────────────────────── */
#define PIN_GPS_TX          33
#define PIN_GPS_RX          27
#define GPS_UART_NUM        UART_NUM_1
#define GPS_BAUD_RATE       9600

/* ── SBUS RC Receiver (inverted UART) ────────────────────────────── */
#define PIN_SBUS_RX         46
#define SBUS_UART_NUM       UART_NUM_2
#define SBUS_BAUD_RATE      100000    /* SBUS: 100kbps, 8E2 */

/* ── PWM - MCPWM (Servos + ESCs) ─────────────────────────────────── */
#define PIN_SWASH_SERVO_1   47        /* CCPM swashplate servo 1 (0 deg) */
#define PIN_SWASH_SERVO_2   48        /* CCPM swashplate servo 2 (120 deg) */
#define PIN_SWASH_SERVO_3   49        /* CCPM swashplate servo 3 (240 deg) */
#define PIN_TAIL_ESC        50        /* Tail rotor ESC signal */
#define PIN_MAIN_ESC        51        /* Main rotor ESC signal */

/* ── Battery ADC (DISABLED) ──────────────────────────────────────── */
// #define PIN_BATT_ADC        (unused)
// #define BATT_ADC_ATTEN      ADC_ATTEN_DB_12
// #define BATT_VOLTAGE_DIVIDER_RATIO  11.0f

/* ── Freed GPIOs ─────────────────────────────────────────────────── */
/* GPIO 29 — was MAG SPI CS, now available
 * GPIO 52 — was sensor SPI MOSI, now available (VoSPI has no MOSI) */

/* ── Task Priorities ─────────────────────────────────────────────── */
#define SENSOR_TASK_PRIORITY    6
#define FLIGHT_CTRL_PRIORITY    5
#define ACTUATOR_TASK_PRIORITY  7
#define SYSMON_TASK_PRIORITY    2
#define HALOW_TASK_PRIORITY     4
#define MAVLINK_TASK_PRIORITY   3

/* ── Task Stack Sizes ────────────────────────────────────────────── */
#define SENSOR_TASK_STACK      8192
#define FLIGHT_CTRL_STACK      8192
#define ACTUATOR_TASK_STACK    8192
#define SYSMON_TASK_STACK      8192

/* ── Core Assignment ─────────────────────────────────────────────── */
#define FC_CORE     0   /* Flight controller on Core 0 */
#define COMM_CORE   1   /* Communication on Core 1 */
