#pragma once

typedef enum {
    FLIGHT_MODE_MANUAL = 0,
    FLIGHT_MODE_STABILIZE,
    FLIGHT_MODE_ALT_HOLD,
    FLIGHT_MODE_LOITER,      /* GPS position hold */
    FLIGHT_MODE_RTH,         /* Return to home */
    FLIGHT_MODE_LAND,        /* Automated landing */
    FLIGHT_MODE_ACRO,        /* Rate-only (no attitude stabilization) */
    FLIGHT_MODE_COUNT
} flight_mode_t;

typedef enum {
    ARM_STATE_DISARMED = 0,
    ARM_STATE_ARMED,
} arm_state_t;

typedef enum {
    FAILSAFE_NONE = 0,
    FAILSAFE_RC_LOST,
    FAILSAFE_BATTERY_LOW,
    FAILSAFE_BATTERY_CRITICAL,
    FAILSAFE_SENSOR_FAILURE,
    FAILSAFE_GCS_LOST,
} failsafe_state_t;
