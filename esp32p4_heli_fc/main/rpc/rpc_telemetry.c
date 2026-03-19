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
