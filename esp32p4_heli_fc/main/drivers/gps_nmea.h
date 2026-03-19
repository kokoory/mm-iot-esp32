/*
 * GPS NMEA-0183 UART Parser
 * Parses GPRMC and GPGGA sentences for position, speed, and fix info.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "../uorb/topics/sensor_gps.h"

/* Maximum NMEA sentence length (including CR/LF) */
#define NMEA_MAX_LEN    128

/* GPS device handle */
typedef struct {
    uart_port_t     uart_num;
    sensor_gps_t    data;           /* latest parsed fix */
    bool            data_valid;     /* at least one valid fix received */
    SemaphoreHandle_t mutex;        /* protects .data */
} gps_handle_t;

/**
 * Initialise UART for GPS reception and start the parser task.
 * Returns 0 on success.
 */
int gps_init(gps_handle_t *gps, uart_port_t uart_num,
             int tx_pin, int rx_pin, uint32_t baud);

/**
 * Copy the latest GPS data into *out.
 * Returns 0 if valid data is available, -1 otherwise.
 */
int gps_get_data(gps_handle_t *gps, sensor_gps_t *out);
