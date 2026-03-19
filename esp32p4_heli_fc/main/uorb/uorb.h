/*
 * uORB - Micro Object Request Broker
 * Simplified PX4-style publish/subscribe message bus for ESP32-P4
 * Uses FreeRTOS queues for inter-task communication
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define ORB_MAX_TOPICS      16
#define ORB_MAX_SUBSCRIBERS  4

typedef uint8_t orb_id_t;

/* Topic IDs */
enum {
    ORB_ID_SENSOR_IMU = 0,
    ORB_ID_SENSOR_BARO,
    ORB_ID_SENSOR_MAG,
    ORB_ID_SENSOR_GPS,
    ORB_ID_VEHICLE_ATTITUDE,
    ORB_ID_VEHICLE_LOCAL_POSITION,
    ORB_ID_ACTUATOR_CONTROLS,
    ORB_ID_BATTERY_STATUS,
    ORB_ID_VEHICLE_STATUS,
    ORB_ID_RC_CHANNELS,
    ORB_ID_COUNT
};

typedef struct {
    orb_id_t        topic_id;
    QueueHandle_t   queue;      /* single-item queue for latest value */
} orb_subscription_t;

/* Initialize the uORB system - must be called before any other orb_ function */
void orb_init(void);

/* Advertise a topic - registers the data size for this topic.
 * Returns 0 on success, -1 on failure. */
int orb_advertise(orb_id_t topic, size_t data_size);

/* Publish data to a topic - copies data to all subscriber queues.
 * Returns 0 on success, -1 on failure. */
int orb_publish(orb_id_t topic, const void *data);

/* Subscribe to a topic - returns a subscription handle.
 * Returns NULL on failure. */
orb_subscription_t *orb_subscribe(orb_id_t topic);

/* Copy the latest data from a subscription into the caller's buffer.
 * Returns 0 on success, -1 if no data available. */
int orb_copy(orb_subscription_t *sub, void *data);

/* Check if new data is available on a subscription (non-blocking). */
bool orb_check(orb_subscription_t *sub);
