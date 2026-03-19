/*
 * uORB topic: Sensor IMU
 * Combined accelerometer + gyroscope data
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint64_t timestamp_us;
    float accel_x, accel_y, accel_z;    /* m/s^2 */
    float gyro_x, gyro_y, gyro_z;      /* rad/s */
    float temperature;                   /* degrees C */
} sensor_imu_t;
