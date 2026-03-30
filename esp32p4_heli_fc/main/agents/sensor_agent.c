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
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "../common/board_config.h"
#include "../common/i2c_sync.h"
#include "../common/math_utils.h"
#include "../uorb/uorb.h"
#include "../uorb/topics/sensor_imu.h"
#include "../uorb/topics/sensor_baro.h"
#include "../uorb/topics/sensor_mag.h"
#include "../uorb/topics/sensor_gps.h"
#include "../uorb/topics/vehicle_attitude.h"
#include "../uorb/topics/vehicle_local_position.h"
#include "../uorb/topics/rc_channels.h"
#include "../uorb/topics/airspeed.h"
#include "../drivers/ism330dhc.h"
#include "../drivers/bmp390.h"
#include "../drivers/lis3mdl.h"
#include "../drivers/gps_nmea.h"
#include "../drivers/sbus.h"
#include "../drivers/mprls.h"
#include "../estimator/ahrs.h"
#include "../estimator/altitude_estimator.h"
#include "../common/param.h"

static const char *TAG = "sensor_agent";

/* Sensor instances */
static ism330dhc_t  s_imu;
static bmp390_t     s_baro;
static lis3mdl_t    s_mag;
static gps_handle_t s_gps;
static mprls_t      s_mprls;

/* Estimators */
static ahrs_t           s_ahrs;
static alt_estimator_t  s_alt_est;

/* Latest mag data for AHRS 9-DOF update */
static float s_mag_data[3] = {0};
static bool  s_mag_valid = false;

/* ── Sensor validation thresholds ──────────────────────────── */
#define IMU_ACCEL_MAX     160.0f   /* m/s^2 (~16g), reject spikes above this */
#define IMU_GYRO_MAX      34.9f    /* rad/s (~2000 deg/s), reject spikes */
#define BARO_ALT_MAX_CHANGE 50.0f  /* m, max plausible altitude change between readings */

/* Previous baro for outlier detection */
static float s_prev_baro_alt = 0.0f;
static bool  s_prev_baro_valid = false;

/* IMU low-pass filter state for vibration rejection */
static float s_accel_filt[3] = {0};
static float s_gyro_filt[3] = {0};
static bool  s_imu_filt_initialized = false;
#define IMU_ACCEL_FILTER_ALPHA 0.8f  /* higher = more raw data, lower = more smoothing */
#define IMU_GYRO_FILTER_ALPHA  0.9f

/* ------------------------------------------------------------------ */
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

    /* ---- Initialize I2C bus ---- */
    i2c_master_bus_handle_t i2c_bus = init_i2c_bus();
    if (!i2c_bus) {
        ESP_LOGE(TAG, "I2C init failed, task aborting");
        vTaskDelete(NULL);
        return;
    }

    /* ---- Initialize sensors (exclusive I2C access) ---- */
    bool imu_ok = (ism330dhc_init(&s_imu, i2c_bus, ISM330DHC_I2C_ADDR) == 0);
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

    /* MPRLS differential pressure sensor (pitot tube airspeed) */
    bool mprls_ok = (mprls_init(&s_mprls, i2c_bus, MPRLS_I2C_ADDR) == 0);
    if (mprls_ok) {
        ESP_LOGI(TAG, "MPRLS pressure sensor initialized");
    } else {
        ESP_LOGW(TAG, "MPRLS init FAILED (airspeed unavailable)");
    }

    /* Signal camera that I2C sensor init is complete — safe to use SCCB now */
    ESP_LOGI(TAG, "I2C sensor init complete, releasing bus for camera SCCB");
    i2c_sync_sensors_done();

    /* GPS init (starts its own internal UART parser task) */
    bool gps_ok = (gps_init(&s_gps, GPS_UART_NUM, PIN_GPS_TX, PIN_GPS_RX, GPS_BAUD_RATE) == 0);
    if (gps_ok) {
        ESP_LOGI(TAG, "GPS (NMEA UART%d) initialized", GPS_UART_NUM);
    } else {
        ESP_LOGW(TAG, "GPS init FAILED (position unavailable)");
    }

    /* SBUS RC receiver init */
    bool sbus_ok = (sbus_init() == 0);
    if (sbus_ok) {
        ESP_LOGI(TAG, "SBUS RC receiver initialized");
    } else {
        ESP_LOGW(TAG, "SBUS init FAILED (RC unavailable)");
    }

    /* ---- Initialize estimators ---- */
    ahrs_init(&s_ahrs, param_get(PARAM_AHRS_BETA));
    alt_estimator_init(&s_alt_est);
    s_alt_est.alpha = param_get(PARAM_ALT_CF_ALPHA);

    /* ---- Advertise uORB topics ---- */
    orb_advertise(ORB_ID_SENSOR_IMU, sizeof(sensor_imu_t));
    orb_advertise(ORB_ID_SENSOR_BARO, sizeof(sensor_baro_t));
    orb_advertise(ORB_ID_SENSOR_MAG, sizeof(sensor_mag_t));
    orb_advertise(ORB_ID_SENSOR_GPS, sizeof(sensor_gps_t));
    orb_advertise(ORB_ID_VEHICLE_ATTITUDE, sizeof(vehicle_attitude_t));
    orb_advertise(ORB_ID_VEHICLE_LOCAL_POSITION, sizeof(vehicle_local_position_t));

    /* RC_CHANNELS: populated by SBUS receiver or RPC RC override from GCS */
    orb_advertise(ORB_ID_RC_CHANNELS, sizeof(rc_channels_t));
    orb_advertise(ORB_ID_AIRSPEED, sizeof(airspeed_t));

    /* Suppress I2C driver error logs during main loop —
     * NACK errors are expected when sensors are absent/failing
     * and will flood the console and trigger watchdog at 1kHz. */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    /* ---- Main loop at 1 kHz ---- */
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle = 0;
    uint64_t prev_us = (uint64_t)esp_timer_get_time();

    /* Sensor health counters for diagnostics */
    uint32_t imu_read_errors = 0;
    uint32_t imu_consecutive_errors = 0;
    uint32_t imu_spike_rejects = 0;
    uint32_t baro_read_errors = 0;
    uint32_t baro_outlier_rejects = 0;

    #define SENSOR_MAX_CONSECUTIVE_ERRORS 50  /* disable sensor after 50 consecutive fails */

    while (1) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        float dt = (float)(now_us - prev_us) * 1.0e-6f;
        if (dt <= 0.0f || dt > 0.01f) dt = 0.001f;  /* sanity clamp */
        prev_us = now_us;

        /* ---- IMU: every cycle (1 kHz) ---- */
        if (imu_ok) {
            sensor_imu_t imu_msg;
            float accel[3], gyro[3], temp;

            if (ism330dhc_read(&s_imu, accel, gyro, &temp) == 0) {
                imu_consecutive_errors = 0; /* reset on success */
                /* --- Spike rejection: reject readings outside physical limits --- */
                float accel_mag = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
                float gyro_mag = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);

                if (accel_mag > IMU_ACCEL_MAX || gyro_mag > IMU_GYRO_MAX) {
                    imu_spike_rejects++;
                    if ((imu_spike_rejects % 100) == 1) {
                        ESP_LOGW(TAG, "IMU spike rejected (count=%lu): accel=%.1f gyro=%.1f",
                                 (unsigned long)imu_spike_rejects, accel_mag, gyro_mag);
                    }
                    goto imu_done;  /* skip this sample, keep using previous filtered values */
                }

                /* --- Low-pass filter for vibration rejection --- */
                if (!s_imu_filt_initialized) {
                    for (int i = 0; i < 3; i++) {
                        s_accel_filt[i] = accel[i];
                        s_gyro_filt[i] = gyro[i];
                    }
                    s_imu_filt_initialized = true;
                } else {
                    for (int i = 0; i < 3; i++) {
                        s_accel_filt[i] += IMU_ACCEL_FILTER_ALPHA * (accel[i] - s_accel_filt[i]);
                        s_gyro_filt[i]  += IMU_GYRO_FILTER_ALPHA  * (gyro[i]  - s_gyro_filt[i]);
                    }
                }

                /* Use filtered values for estimators, raw for publishing */
                imu_msg.timestamp_us = now_us;
                imu_msg.accel_x = accel[0];
                imu_msg.accel_y = accel[1];
                imu_msg.accel_z = accel[2];
                imu_msg.gyro_x = gyro[0];
                imu_msg.gyro_y = gyro[1];
                imu_msg.gyro_z = gyro[2];
                imu_msg.temperature = temp;
                orb_publish(ORB_ID_SENSOR_IMU, &imu_msg);

                /* ---- AHRS update (uses filtered data) ---- */
                float gyro_arr[3] = {s_gyro_filt[0], s_gyro_filt[1], s_gyro_filt[2]};
                float accel_arr[3] = {s_accel_filt[0], s_accel_filt[1], s_accel_filt[2]};

                if (s_mag_valid) {
                    ahrs_update(&s_ahrs, gyro_arr, accel_arr, s_mag_data, dt);
                } else {
                    ahrs_update_imu(&s_ahrs, gyro_arr, accel_arr, dt);
                }

                /* Publish attitude (use bias-corrected filtered gyro for rate feedback) */
                float gyro_bias[3];
                ahrs_get_gyro_bias(&s_ahrs, gyro_bias);

                vehicle_attitude_t att_msg;
                att_msg.timestamp_us = now_us;
                ahrs_get_quaternion(&s_ahrs, att_msg.q);
                ahrs_get_euler(&s_ahrs, &att_msg.roll, &att_msg.pitch, &att_msg.yaw);
                att_msg.rollspeed  = s_gyro_filt[0] - gyro_bias[0];
                att_msg.pitchspeed = s_gyro_filt[1] - gyro_bias[1];
                att_msg.yawspeed   = s_gyro_filt[2] - gyro_bias[2];
                orb_publish(ORB_ID_VEHICLE_ATTITUDE, &att_msg);

                /* ---- Altitude estimator: accel update ---- */
                float q0 = s_ahrs.q0, q1 = s_ahrs.q1;
                float q2 = s_ahrs.q2, q3 = s_ahrs.q3;

                float az_world = 2.0f * (q1 * q3 - q0 * q2) * s_accel_filt[0] +
                                 2.0f * (q2 * q3 + q0 * q1) * s_accel_filt[1] +
                                 (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * s_accel_filt[2];

                float accel_z_up = -(az_world - 9.80665f);
                alt_estimator_update_accel(&s_alt_est, accel_z_up, dt);
            } else {
                imu_read_errors++;
                imu_consecutive_errors++;
                if (imu_consecutive_errors == SENSOR_MAX_CONSECUTIVE_ERRORS) {
                    ESP_LOGE(TAG, "IMU: %lu consecutive read failures, disabling",
                             (unsigned long)imu_consecutive_errors);
                    imu_ok = false;
                } else if (imu_consecutive_errors == 1 || (imu_consecutive_errors % 100) == 0) {
                    ESP_LOGW(TAG, "IMU read error (total=%lu)", (unsigned long)imu_read_errors);
                }
            }
        }
imu_done:

        /* ---- Barometer: every 10th cycle (100 Hz) ---- */
        if (baro_ok && (cycle % 10) == 0) {
            float pressure, baro_temp;
            if (bmp390_read(&s_baro, &pressure, &baro_temp) == 0) {
                float alt_msl = bmp390_calc_altitude(pressure);

                /* --- Baro outlier rejection --- */
                bool baro_valid = true;
                if (s_prev_baro_valid) {
                    float delta = fabsf(alt_msl - s_prev_baro_alt);
                    if (delta > BARO_ALT_MAX_CHANGE) {
                        baro_outlier_rejects++;
                        baro_valid = false;
                        if ((baro_outlier_rejects % 10) == 1) {
                            ESP_LOGW(TAG, "Baro outlier rejected (count=%lu): delta=%.1fm",
                                     (unsigned long)baro_outlier_rejects, delta);
                        }
                    }
                }

                if (baro_valid) {
                    s_prev_baro_alt = alt_msl;
                    s_prev_baro_valid = true;

                    sensor_baro_t baro_msg;
                    baro_msg.timestamp_us = now_us;
                    baro_msg.pressure_pa = pressure;
                    baro_msg.temperature = baro_temp;
                    baro_msg.altitude_msl = alt_msl;
                    orb_publish(ORB_ID_SENSOR_BARO, &baro_msg);

                    alt_estimator_update_baro(&s_alt_est, alt_msl, now_us);
                }
            } else {
                baro_read_errors++;
            }
        }

        /* ---- Magnetometer: every 10th cycle, offset by 5 (100 Hz) ---- */
        if (mag_ok && (cycle % 10) == 5) {
            float mag[3];
            if (lis3mdl_read(&s_mag, mag) == 0) {
                /* Reject obviously bad mag readings (>10 gauss ~= noise/interference) */
                float mag_mag = sqrtf(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
                if (mag_mag > 0.01f && mag_mag < 10.0f) {
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
        }

        /* ---- GPS: poll for new data every 100th cycle (10 Hz) ---- */
        if (gps_ok && (cycle % 100) == 0) {
            sensor_gps_t gps_data;
            if (gps_get_data(&s_gps, &gps_data) == 0) {
                gps_data.timestamp_us = now_us;
                orb_publish(ORB_ID_SENSOR_GPS, &gps_data);
            }
        }

        /* ---- SBUS RC: poll every 10th cycle (100 Hz) ---- */
        if (sbus_ok && (cycle % 10) == 3) {
            sbus_data_t sbus;
            if (sbus_read(&sbus) == 0 && !sbus.failsafe) {
                rc_channels_t rc_msg;
                memset(&rc_msg, 0, sizeof(rc_msg));
                rc_msg.timestamp_us = now_us;
                rc_msg.channel_count = 8;
                rc_msg.signal_lost = sbus.frame_lost;
                /* Map first 8 SBUS channels to normalized -1..+1 */
                for (int i = 0; i < 8; i++) {
                    rc_msg.channels[i] = sbus_channel_to_float(sbus.channels[i]);
                }
                orb_publish(ORB_ID_RC_CHANNELS, &rc_msg);
            }
        }

        /* ---- MPRLS Airspeed: every 50th cycle (20 Hz) ---- */
        if (mprls_ok && (cycle % 50) == 7) {
            float dp_pa;
            if (mprls_read(&s_mprls, &dp_pa) == 0) {
                airspeed_t as_msg;
                as_msg.timestamp_us = now_us;
                as_msg.differential_pressure_pa = dp_pa;
                as_msg.indicated_airspeed = mprls_dp_to_airspeed(dp_pa);
                as_msg.temperature = 0.0f;  /* TODO: fill from baro temp for TAS */
                as_msg.valid = (dp_pa >= 0.0f && dp_pa < 6000.0f);  /* ~100 m/s max */
                orb_publish(ORB_ID_AIRSPEED, &as_msg);
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

        /* ---- Sensor health log: every 10 seconds (10000 cycles at 1kHz) ---- */
        if ((cycle % 10000) == 0 && cycle > 0) {
            ESP_LOGI(TAG, "Sensor health: IMU errs=%lu spikes=%lu, Baro errs=%lu outliers=%lu",
                     (unsigned long)imu_read_errors, (unsigned long)imu_spike_rejects,
                     (unsigned long)baro_read_errors, (unsigned long)baro_outlier_rejects);
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

    xTaskCreatePinnedToCoreWithCaps(
        sensor_task,
        "sensor_agent",
        SENSOR_TASK_STACK,
        NULL,
        SENSOR_TASK_PRIORITY,
        NULL,
        FC_CORE,
        MALLOC_CAP_SPIRAM
    );
}
