/*
 * uORB topic: RC Channels
 * Normalized RC input channels
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define RC_CHANNELS_MAX  16

typedef struct {
    uint64_t timestamp_us;
    float channels[RC_CHANNELS_MAX];  /* normalized -1.0 to 1.0 (throttle: 0 to 1) */
    uint8_t channel_count;
    bool signal_lost;
    /* Standard channel mapping */
    /* ch[0] = roll, ch[1] = pitch, ch[2] = throttle/collective, ch[3] = yaw */
    /* ch[4] = flight mode switch, ch[5] = arm switch */
} rc_channels_t;
