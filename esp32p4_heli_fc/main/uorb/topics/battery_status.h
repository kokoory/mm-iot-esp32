/*
 * uORB topic: Battery Status
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint64_t timestamp_us;
    float voltage_v;         /* battery voltage in volts */
    float voltage_filtered;  /* low-pass filtered voltage */
    uint8_t cell_count;      /* estimated cell count */
    float voltage_per_cell;  /* volts per cell */
    bool warning;            /* true if low voltage */
    bool critical;           /* true if critical voltage */
} battery_status_t;
