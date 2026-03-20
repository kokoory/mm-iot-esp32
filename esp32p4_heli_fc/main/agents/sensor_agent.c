/*
 * Sensor Manager Agent Implementation
 *
 * Runs on Core 0 at 1 kHz.
 *   - IMU (ISM330DHC via SPI) read every cycle (1 kHz)
 *   - Barometer (BMP390 via I2C) read every 10th cycle (100 Hz)
 *   - Magnetometer (LIS3MDL via I2C) read every 10th cycle, offset by 5 (100 Hz)
 *   - GPS (NMEA via UART) read in gps_init internal task, polled here for publishing
 *   - AHRS update every cycle with IMU data
 *   - Altitude estimator updated with baro and accel data
 */

#include "sensor_agent.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "../common/board_config.h"
#include "../common/math_utils.h"
#include "../uorb/uorb.h"
#include "../uorb/topics/sensor_imu.h"
#include "../uorb/topics/sensor_baro.h"
#include "../uorb/topics/sensor_mag.h"
#include "../uorb/topics/sensor_gps.h"
#include "../uorb/topics/vehicle_attitude.h"
#include "../uorb/topics/vehicle_local_position.h"
#include "../drivers/ism330dhc.h"
#include "../drivers/bmp390.h"
#include "../drivers/lis3mdl.h"
#include "../drivers/gps_nmea.h"
#include "../estimator/ahrs.h"
#include "../estimator/altitude_estimator.h"

static const char *TAG = "sensor_agent";

/* Sensor instances */
static ism330dhc_t  s_imu;
static bmp390_t     s_baro;
static lis3mdl_t    s_mag;
static gps_handle_t s_gps;

/* Estimators */
static ahrs_t           s_ahrs;
static alt_estimator_t  s_alt_est;

/* Latest mag data for AHRS 9-DOF update */
static float s_mag_data[3] = {0};
static bool  s_mag_valid = false;

/* ------------------------------------------------------------------ */
static int init_spi_bus(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_IMU_SPI_MOSI,
        .miso_io_num = PIN_IMU_SPI_MISO,
        .sclk_io_num = PIN_IMU_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(IMU_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return -1;
    }
    ESP_LOGI(TAG, "SPI bus initialized");
    return 0;
}

static i2c_master_bus_handle_t init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return NULL;
    }
    ESP_LOGI(TAG, "I2C bus initialized");
    return bus;
}

/* ------------------------------------------------------------------ */
static void sensor_task(void *param)
{
    (void)param;

    /* ---- Initialize buses ---- */
    if (init_spi_bus() != 0) {
        ESP_LOGE(TAG, "SPI init failed, task aborting");
        vTaskDelete(NULL);
        return;
    }

    i2c_master_bus_handle_t i2c_bus = init_i2c_bus();
    if (!i2c_bus) {
        ESP_LOGE(TAG, "I2C init failed, task aborting");
        vTaskDelete(NULL);
        return;
    }

    /* ---- Initialize sensors ---- */
    bool imu_ok = (ism330dhc_init(&s_imu, IMU_SPI_HOST, PIN_IMU_SPI_CS) == 0);
    if (imu_ok) {
        imu_ok = (ism330dhc_configure(&s_imu) == 0);
    }
    if (imu_ok) {
        ESP_LOGI(TAG, "IMU (ISM330DHC) initialized");
    } else {
        ESP_LOGE(TAG, "IMU init/configure FAILED");
    }

    bool baro_ok = (bmp390_init(&s_baro, i2c_bus, BMP390_I2C_ADDR) == 0);
    if (baro_ok) {
        baro_ok = (bmp390_configure(&s_baro) == 0);
    }
    if (baro_ok) {
        ESP_LOGI(TAG, "Baro (BMP390) initialized");
    } else {
        ESP_LOGE(TAG, "Baro init/configure FAILED");
    }

    bool mag_ok = (lis3mdl_init(&s_mag, i2c_bus, LIS3MDL_I2C_ADDR) == 0);
    if (mag_ok) {
        mag_ok = (lis3mdl_configure(&s_mag) == 0);
    }
    if (mag_ok) {
        ESP_LOGI(TAG, "Mag (LIS3MDL) initialized");
    } else {
        ESP_LOGE(TAG, "Mag init/configure FAILED");
    }

    /* GPS init (starts its own internal UART parser task) */
    bool gps_ok = (gps_init(&s_gps, GPS_UART_NUM, PIN_GPS_TX, PIN_GPS_RX, GPS_BAUD_RATE) == 0);
    if (gps_ok) {
        ESP_LOGI(TAG, "GPS UART initialized");
    } else {
        ESP_LOGE(TAG, "GPS init FAILED");
    }

    /* ---- Initialize estimators ---- */
    ahrs_init(&s_ahrs, 0.1f);
    alt_estimator_init(&s_alt_est);

    /* ---- Advertise uORB topics ---- */
    orb_advertise(ORB_ID_SENSOR_IMU, sizeof(sensor_imu_t));
    orb_advertise(ORB_ID_SENSOR_BARO, sizeof(sensor_baro_t));
    orb_advertise(ORB_ID_SENSOR_MAG, sizeof(sensor_mag_t));
    orb_advertise(ORB_ID_SENSOR_GPS, sizeof(sensor_gps_t));
    orb_advertise(ORB_ID_VEHICLE_ATTITUDE, sizeof(vehicle_attitude_t));
    orb_advertise(ORB_ID_VEHICLE_LOCAL_POSITION, sizeof(vehicle_local_position_t));

    /* ---- Main loop at 1 kHz ---- */
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle = 0;
    const float dt = 0.001f;  /* 1 kHz -> 1 ms */

    while (1) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();

        /* ---- IMU: every cycle (1 kHz) ---- */
        if (imu_ok) {
            sensor_imu_t imu_msg;
            float accel[3], gyro[3], temp;

            if (ism330dhc_read(&s_imu, accel, gyro, &temp) == 0) {
                imu_msg.timestamp_us = now_us;
                imu_msg.accel_x = accel[0];
                imu_msg.accel_y = accel[1];
                imu_msg.accel_z = accel[2];
                imu_msg.gyro_x = gyro[0];
                imu_msg.gyro_y = gyro[1];
                imu_msg.gyro_z = gyro[2];
                imu_msg.temperature = temp;
                orb_publish(ORB_ID_SENSOR_IMU, &imu_msg);

                /* ---- AHRS update ---- */
                float gyro_arr[3] = {gyro[0], gyro[1], gyro[2]};
                float accel_arr[3] = {accel[0], accel[1], accel[2]};

                if (s_mag_valid) {
                    ahrs_update(&s_ahrs, gyro_arr, accel_arr, s_mag_data, dt);
                } else {
                    ahrs_update_imu(&s_ahrs, gyro_arr, accel_arr, dt);
                }

                /* Publish attitude */
                vehicle_attitude_t att_msg;
                att_msg.timestamp_us = now_us;
                ahrs_get_quaternion(&s_ahrs, att_msg.q);
                ahrs_get_euler(&s_ahrs, &att_msg.roll, &att_msg.pitch, &att_msg.yaw);
                att_msg.rollspeed  = gyro[0];
                att_msg.pitchspeed = gyro[1];
                att_msg.yawspeed   = gyro[2];
                orb_publish(ORB_ID_VEHICLE_ATTITUDE, &att_msg);

                /* ---- Altitude estimator: accel update ---- */
                /* Convert body-frame accel to world-frame Z using quaternion */
                float q0 = s_ahrs.q0, q1 = s_ahrs.q1;
                float q2 = s_ahrs.q2, q3 = s_ahrs.q3;

                /* Rotate accel vector to world frame, extract Z component */
                /* R(2,:) * accel = world Z acceleration */
                float az_world = 2.0f * (q1 * q3 - q0 * q2) * accel[0] +
                                 2.0f * (q2 * q3 + q0 * q1) * accel[1] +
                                 (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * accel[2];

                /* Remove gravity (NED: gravity is +9.81 in Z-down).
                 * For positive-up altitude, invert and remove gravity. */
                float accel_z_up = -(az_world - 9.80665f);

                alt_estimator_update_accel(&s_alt_est, accel_z_up, dt);
            }
        }

        /* ---- Barometer: every 10th cycle (100 Hz) ---- */
        if (baro_ok && (cycle % 10) == 0) {
            float pressure, baro_temp;
            if (bmp390_read(&s_baro, &pressure, &baro_temp) == 0) {
                float alt_msl = bmp390_calc_altitude(pressure);

                sensor_baro_t baro_msg;
                baro_msg.timestamp_us = now_us;
                baro_msg.pressure_pa = pressure;
                baro_msg.temperature = baro_temp;
                baro_msg.altitude_msl = alt_msl;
                orb_publish(ORB_ID_SENSOR_BARO, &baro_msg);

                /* Update altitude estimator with barometer */
                alt_estimator_update_baro(&s_alt_est, alt_msl);
            }
        }

        /* ---- Magnetometer: every 10th cycle, offset by 5 (100 Hz) ---- */
        if (mag_ok && (cycle % 10) == 5) {
            float mag[3];
            if (lis3mdl_read(&s_mag, mag) == 0) {
                s_mag_data[0] = mag[0];
                s_mag_data[1] = mag[1];
                s_mag_data[2] = mag[2];
                s_mag_valid = true;

                sensor_mag_t mag_msg;
                mag_msg.timestamp_us = now_us;
                mag_msg.mag_x = mag[0];
                mag_msg.mag_y = mag[1];
                mag_msg.mag_z = mag[2];
                orb_publish(ORB_ID_SENSOR_MAG, &mag_msg);
            }
        }

        /* ---- GPS: poll for new data every 100th cycle (10 Hz) ---- */
        if (gps_ok && (cycle % 100) == 0) {
            sensor_gps_t gps_data;
            if (gps_get_data(&s_gps, &gps_data) == 0) {
                gps_data.timestamp_us = now_us;
                orb_publish(ORB_ID_SENSOR_GPS, &gps_data);
            }
        }

        /* ---- Publish local position (from altitude estimator) every 10th cycle ---- */
        if ((cycle % 10) == 2) {
            vehicle_local_position_t pos_msg;
            memset(&pos_msg, 0, sizeof(pos_msg));
            pos_msg.timestamp_us = now_us;

            float alt, climb;
            alt_estimator_get(&s_alt_est, &alt, &climb);
            pos_msg.alt = alt;
            pos_msg.climb_rate = climb;
            pos_msg.z = -alt;           /* NED: Z is down */
            pos_msg.vz = -climb;        /* NED: vz positive downward */
            pos_msg.z_valid = baro_ok;
            pos_msg.v_z_valid = baro_ok;
            orb_publish(ORB_ID_VEHICLE_LOCAL_POSITION, &pos_msg);
        }

        cycle++;
        /* Sleep until next 1 ms boundary */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

void sensor_agent_start(void)
{
    ESP_LOGI(TAG, "Starting sensor agent on Core %d, priority %d",
             FC_CORE, SENSOR_TASK_PRIORITY);

    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_agent",
        SENSOR_TASK_STACK,
        NULL,
        SENSOR_TASK_PRIORITY,
        NULL,
        FC_CORE
    );
}
