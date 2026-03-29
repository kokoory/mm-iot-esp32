/*
 * Mission Manager Implementation
 *
 * Simple array-based storage for mission items on Core 0.
 * No NVS persistence - missions are lost on reboot and must be re-uploaded.
 *
 * Thread safety: single-writer (sysmon_agent) / single-reader (flight_ctrl_agent)
 * model on Core 0. The count and current index are written atomically (uint16_t),
 * so no mutex is needed for this use case.
 */

#include "mission_mgr.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "mission_mgr";

static mission_item_t s_items[MISSION_MAX_ITEMS];
static uint16_t s_count   = 0;   /* expected/total item count */
static uint16_t s_stored  = 0;   /* number of items actually received */
static uint16_t s_current = 0;   /* current mission item index */

void mission_mgr_init(void)
{
    memset(s_items, 0, sizeof(s_items));
    s_count   = 0;
    s_stored  = 0;
    s_current = 0;
    ESP_LOGI(TAG, "Mission manager initialized (max %d items)", MISSION_MAX_ITEMS);
}

int mission_mgr_get_count(void)
{
    return (int)s_count;
}

const mission_item_t *mission_mgr_get_item(uint16_t seq)
{
    if (seq >= s_count || seq >= MISSION_MAX_ITEMS) {
        return NULL;
    }
    return &s_items[seq];
}

uint16_t mission_mgr_get_current(void)
{
    return s_current;
}

void mission_mgr_set_current(uint16_t seq)
{
    if (seq < s_count) {
        s_current = seq;
        ESP_LOGI(TAG, "Current mission item set to %u", seq);
    } else {
        ESP_LOGW(TAG, "set_current: seq %u out of range (count=%u)", seq, s_count);
    }
}

void mission_mgr_advance(void)
{
    if (s_current < s_count) {
        s_current++;
        ESP_LOGI(TAG, "Advanced to mission item %u/%u", s_current, s_count);
    }
}

void mission_mgr_clear(void)
{
    memset(s_items, 0, sizeof(s_items));
    s_count   = 0;
    s_stored  = 0;
    s_current = 0;
    ESP_LOGI(TAG, "Mission cleared");
}

void mission_mgr_set_count(uint16_t count)
{
    if (count > MISSION_MAX_ITEMS) {
        ESP_LOGW(TAG, "Requested count %u exceeds max %d, clamping",
                 count, MISSION_MAX_ITEMS);
        count = MISSION_MAX_ITEMS;
    }

    /* Clear existing items and prepare for new upload */
    memset(s_items, 0, sizeof(s_items));
    s_count   = count;
    s_stored  = 0;
    s_current = 0;
    ESP_LOGI(TAG, "Expecting %u mission items", count);
}

void mission_mgr_store_item(uint16_t seq, const mission_item_t *item)
{
    if (item == NULL) {
        return;
    }
    if (seq >= s_count || seq >= MISSION_MAX_ITEMS) {
        ESP_LOGW(TAG, "store_item: seq %u out of range (count=%u)", seq, s_count);
        return;
    }

    memcpy(&s_items[seq], item, sizeof(mission_item_t));
    s_items[seq].seq = seq; /* ensure seq field matches index */
    s_stored++;

    ESP_LOGI(TAG, "Stored item %u: cmd=%u frame=%u x=%ld y=%ld z=%.1f (%u/%u received)",
             seq, item->command, item->frame,
             (long)item->x, (long)item->y, item->z,
             s_stored, s_count);
}

bool mission_mgr_is_complete(void)
{
    return (s_count == 0) || (s_current >= s_count);
}
