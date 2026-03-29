/*
 * Mission Manager - Shared mission item storage for Core 0
 *
 * Stores up to MISSION_MAX_ITEMS waypoints received from Core 1 via RPC.
 * flight_ctrl_agent reads mission items for autonomous waypoint following.
 * sysmon_agent writes mission items in response to RPC commands from GCS.
 *
 * All functions are safe to call from any Core 0 task (single-writer model:
 * sysmon writes, flight_ctrl reads).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MISSION_MAX_ITEMS 50

typedef struct {
    uint16_t seq;
    uint8_t  frame;
    uint16_t command;
    uint8_t  autocontinue;
    float    param1, param2, param3, param4;
    int32_t  x;  /* lat*1e7 */
    int32_t  y;  /* lon*1e7 */
    float    z;  /* alt meters */
} mission_item_t;

/**
 * Initialize the mission manager. Call once at startup before other functions.
 */
void mission_mgr_init(void);

/**
 * Get the number of stored mission items.
 */
int mission_mgr_get_count(void);

/**
 * Get a pointer to a stored mission item by sequence number.
 * Returns NULL if seq is out of range or no item stored at that index.
 */
const mission_item_t *mission_mgr_get_item(uint16_t seq);

/**
 * Get the current mission item sequence index.
 */
uint16_t mission_mgr_get_current(void);

/**
 * Set the current mission item sequence index.
 */
void mission_mgr_set_current(uint16_t seq);

/**
 * Advance to the next mission item. If already at or past the last item,
 * current remains at count (mission complete).
 */
void mission_mgr_advance(void);

/**
 * Clear all stored mission items and reset state.
 */
void mission_mgr_clear(void);

/**
 * Set the expected mission item count. Called when receiving MISSION_COUNT
 * from the GCS to prepare for incoming MISSION_ITEM messages.
 * Clears any existing items and resets current index to 0.
 */
void mission_mgr_set_count(uint16_t count);

/**
 * Store a mission item at the given sequence number.
 * seq must be < the count set by mission_mgr_set_count().
 */
void mission_mgr_store_item(uint16_t seq, const mission_item_t *item);

/**
 * Returns true if the current index has reached or exceeded the item count,
 * meaning all mission items have been executed.
 */
bool mission_mgr_is_complete(void);
