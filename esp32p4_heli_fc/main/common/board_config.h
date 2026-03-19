#pragma once

#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

// IMU (ICM-42688-P) - SPI3
#define PIN_IMU_SPI_SCK     4
#define PIN_IMU_SPI_MOSI    5
#define PIN_IMU_SPI_MISO    6
#define PIN_IMU_SPI_CS      7
#define IMU_SPI_HOST        SPI3_HOST
#define IMU_SPI_FREQ_HZ     10000000  // 10MHz

// I2C0 (Baro + Mag)
#define PIN_I2C_SDA         8
#define PIN_I2C_SCL         9
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000

// GPS UART
#define PIN_GPS_TX          10
#define PIN_GPS_RX          11
#define GPS_UART_NUM        UART_NUM_1
#define GPS_BAUD_RATE       9600

// PWM - MCPWM
#define PIN_SWASH_SERVO_1   12
#define PIN_SWASH_SERVO_2   13
#define PIN_SWASH_SERVO_3   14
#define PIN_TAIL_ESC        15
#define PIN_MAIN_ESC        16

// Battery ADC
#define PIN_BATT_ADC        17
#define BATT_ADC_CHANNEL    ADC_CHANNEL_0
#define BATT_ADC_ATTEN      ADC_ATTEN_DB_12
#define BATT_VOLTAGE_DIVIDER_RATIO  11.0f  // voltage divider

// LED
#define PIN_STATUS_LED      18

// I2C Addresses
#define BMP390_I2C_ADDR     0x77
#define QMC5883L_I2C_ADDR   0x0D

// Task config
#define SENSOR_TASK_PRIORITY    6
#define FLIGHT_CTRL_PRIORITY    5
#define ACTUATOR_TASK_PRIORITY  7
#define SYSMON_TASK_PRIORITY    2
#define HALOW_TASK_PRIORITY     4
#define MAVLINK_TASK_PRIORITY   3

#define SENSOR_TASK_STACK      8192
#define FLIGHT_CTRL_STACK      8192
#define ACTUATOR_TASK_STACK    4096
#define SYSMON_TASK_STACK      4096

#define FC_CORE     0   // Flight controller on Core 0
#define COMM_CORE   1   // Communication on Core 1
