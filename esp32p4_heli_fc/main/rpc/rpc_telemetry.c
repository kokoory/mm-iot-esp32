/*
 * RPC Telemetry Helpers - Implementation
 */
#include "rpc_telemetry.h"
#include "esp_timer.h"
#include <string.h>

static uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

int rpc_telem_send_attitude(rpc_context_t *ctx, const vehicle_attitude_t *att)
{
    if (ctx == NULL || att == NULL) {
        return -1;
    }

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_ATTITUDE;
    msg.timestamp_ms = get_time_ms();
    msg.data.attitude.roll       = att->roll;
    msg.data.attitude.pitch      = att->pitch;
    msg.data.attitude.yaw        = att->yaw;
    msg.data.attitude.rollspeed  = att->rollspeed;
    msg.data.attitude.pitchspeed = att->pitchspeed;
    msg.data.attitude.yawspeed   = att->yawspeed;

    return rpc_send_telemetry(ctx, &msg);
}

int rpc_telem_send_gps(rpc_context_t *ctx, const sensor_gps_t *gps)
{
    if (ctx == NULL || gps == NULL) {
        return -1;
    }

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_GPS;
    msg.timestamp_ms = get_time_ms();
    msg.data.gps.lat          = gps->latitude;
    msg.data.gps.lon          = gps->longitude;
    msg.data.gps.alt          = gps->altitude_msl;
    msg.data.gps.fix_type     = gps->fix_type;
    msg.data.gps.satellites   = gps->satellites;
    msg.data.gps.hdop         = gps->hdop;
    msg.data.gps.ground_speed = gps->ground_speed;
    msg.data.gps.course       = gps->course;

    return rpc_send_telemetry(ctx, &msg);
}

int rpc_telem_send_altitude(rpc_context_t *ctx, const vehicle_local_position_t *pos)
{
    if (ctx == NULL || pos == NULL) {
        return -1;
    }

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_ALTITUDE;
    msg.timestamp_ms = get_time_ms();
    msg.data.altitude.alt_msl    = pos->alt;       /* use local alt as MSL approximation */
    msg.data.altitude.alt_rel    = pos->alt;       /* relative alt (above home) */
    msg.data.altitude.climb_rate = pos->climb_rate;

    return rpc_send_telemetry(ctx, &msg);
}

int rpc_telem_send_battery(rpc_context_t *ctx, const battery_status_t *bat)
{
    if (ctx == NULL || bat == NULL) {
        return -1;
    }

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_BATTERY;
    msg.timestamp_ms = get_time_ms();
    msg.data.battery.voltage       = bat->voltage_v;
    msg.data.battery.current       = 0.0f;  /* no current sensor */
    /* Estimate remaining from per-cell voltage: 4.2V=100%, 3.3V=0% */
    float pct = 0.0f;
    if (bat->cell_count > 0) {
        pct = (bat->voltage_per_cell - 3.3f) / (4.2f - 3.3f) * 100.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
    }
    msg.data.battery.remaining_pct = (uint8_t)pct;

    return rpc_send_telemetry(ctx, &msg);
}

int rpc_telem_send_status(rpc_context_t *ctx, const vehicle_status_t *status)
{
    if (ctx == NULL || status == NULL) {
        return -1;
    }

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_STATUS;
    msg.timestamp_ms = get_time_ms();
    msg.data.status.armed       = status->arm_state;
    msg.data.status.flight_mode = status->flight_mode;
    msg.data.status.failsafe    = status->failsafe;

    /* Pack sensor health into bitmask */
    uint8_t health = 0;
    if (status->sensor_imu_ok)  health |= 0x01;
    if (status->sensor_baro_ok) health |= 0x02;
    if (status->sensor_mag_ok)  health |= 0x04;
    if (status->sensor_gps_ok)  health |= 0x08;
    msg.data.status.sensor_health = health;

    return rpc_send_telemetry(ctx, &msg);
}

int rpc_telem_send_imu_raw(rpc_context_t *ctx, const sensor_imu_t *imu)
{
    if (ctx == NULL || imu == NULL) return -1;

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_IMU_RAW;
    msg.timestamp_ms = get_time_ms();
    msg.data.imu_raw.accel_x = imu->accel_x;
    msg.data.imu_raw.accel_y = imu->accel_y;
    msg.data.imu_raw.accel_z = imu->accel_z;
    msg.data.imu_raw.gyro_x  = imu->gyro_x;
    msg.data.imu_raw.gyro_y  = imu->gyro_y;
    msg.data.imu_raw.gyro_z  = imu->gyro_z;
    msg.data.imu_raw.temperature = imu->temperature;

    return rpc_send_telemetry(ctx, &msg);
}

int rpc_telem_send_mag_raw(rpc_context_t *ctx, const sensor_mag_t *mag)
{
    if (ctx == NULL || mag == NULL) return -1;

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_MAG_RAW;
    msg.timestamp_ms = get_time_ms();
    msg.data.mag_raw.x = mag->mag_x;
    msg.data.mag_raw.y = mag->mag_y;
    msg.data.mag_raw.z = mag->mag_z;

    return rpc_send_telemetry(ctx, &msg);
}

int rpc_telem_send_baro_raw(rpc_context_t *ctx, const sensor_baro_t *baro)
{
    if (ctx == NULL || baro == NULL) return -1;

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_BARO_RAW;
    msg.timestamp_ms = get_time_ms();
    msg.data.baro_raw.pressure    = baro->pressure_pa;
    msg.data.baro_raw.temperature = baro->temperature;
    msg.data.baro_raw.altitude    = baro->altitude_msl;

    return rpc_send_telemetry(ctx, &msg);
}

int rpc_telem_send_estimator(rpc_context_t *ctx,
                             uint16_t flags,
                             float vel_ratio, float pos_horiz_ratio,
                             float pos_vert_ratio,
                             float pos_horiz_accuracy, float pos_vert_accuracy)
{
    if (ctx == NULL) return -1;

    rpc_telemetry_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = RPC_MSG_ESTIMATOR;
    msg.timestamp_ms = get_time_ms();
    msg.data.estimator.flags              = flags;
    msg.data.estimator.vel_ratio          = vel_ratio;
    msg.data.estimator.pos_horiz_ratio    = pos_horiz_ratio;
    msg.data.estimator.pos_vert_ratio     = pos_vert_ratio;
    msg.data.estimator.pos_horiz_accuracy = pos_horiz_accuracy;
    msg.data.estimator.pos_vert_accuracy  = pos_vert_accuracy;

    return rpc_send_telemetry(ctx, &msg);
}
