/*
 * uORB - Micro Object Request Broker Implementation
 *
 * Each topic has an internal metadata entry storing:
 *   - data size
 *   - latest data buffer (heap-allocated)
 *   - mutex for thread-safe access
 *   - array of subscriber queues
 *
 * Publishing overwrites every subscriber's queue (depth-1, overwrite mode)
 * so subscribers always see the latest value without blocking the publisher.
 */

#include "uorb.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "uorb";

/* Per-topic metadata */
typedef struct {
    bool            advertised;
    size_t          data_size;
    void           *latest_data;                        /* heap buffer for latest copy */
    SemaphoreHandle_t mutex;
    QueueHandle_t   subscriber_queues[ORB_MAX_SUBSCRIBERS];
    uint8_t         subscriber_count;
} orb_topic_meta_t;

static orb_topic_meta_t s_topics[ORB_MAX_TOPICS];
static SemaphoreHandle_t s_global_mutex;
static bool s_initialized = false;

/* ------------------------------------------------------------------ */
void orb_init(void)
{
    if (s_initialized) {
        return;
    }

    s_global_mutex = xSemaphoreCreateMutex();
    configASSERT(s_global_mutex);

    memset(s_topics, 0, sizeof(s_topics));
    s_initialized = true;

    ESP_LOGI(TAG, "uORB initialised (%d topic slots)", ORB_MAX_TOPICS);
}

/* ------------------------------------------------------------------ */
int orb_advertise(orb_id_t topic, size_t data_size)
{
    if (topic >= ORB_MAX_TOPICS || data_size == 0) {
        ESP_LOGE(TAG, "advertise: invalid topic %d or size %u", topic, (unsigned)data_size);
        return -1;
    }

    xSemaphoreTake(s_global_mutex, portMAX_DELAY);

    orb_topic_meta_t *meta = &s_topics[topic];

    if (meta->advertised) {
        /* Already advertised – just verify matching size */
        if (meta->data_size != data_size) {
            ESP_LOGE(TAG, "advertise: topic %d size mismatch (%u vs %u)",
                     topic, (unsigned)meta->data_size, (unsigned)data_size);
            xSemaphoreGive(s_global_mutex);
            return -1;
        }
        xSemaphoreGive(s_global_mutex);
        return 0;
    }

    meta->data_size = data_size;
    meta->latest_data = calloc(1, data_size);
    if (!meta->latest_data) {
        ESP_LOGE(TAG, "advertise: OOM for topic %d", topic);
        xSemaphoreGive(s_global_mutex);
        return -1;
    }

    meta->mutex = xSemaphoreCreateMutex();
    configASSERT(meta->mutex);

    meta->subscriber_count = 0;
    meta->advertised = true;

    xSemaphoreGive(s_global_mutex);

    ESP_LOGI(TAG, "topic %d advertised (size %u)", topic, (unsigned)data_size);
    return 0;
}

/* ------------------------------------------------------------------ */
int orb_publish(orb_id_t topic, const void *data)
{
    if (topic >= ORB_MAX_TOPICS || !data) {
        return -1;
    }

    orb_topic_meta_t *meta = &s_topics[topic];

    if (!meta->advertised) {
        ESP_LOGW(TAG, "publish: topic %d not advertised", topic);
        return -1;
    }

    xSemaphoreTake(meta->mutex, portMAX_DELAY);

    /* Store latest copy */
    memcpy(meta->latest_data, data, meta->data_size);

    /* Push to every subscriber queue (overwrite if full) */
    for (uint8_t i = 0; i < meta->subscriber_count; i++) {
        QueueHandle_t q = meta->subscriber_queues[i];
        if (q) {
            xQueueOverwrite(q, data);
        }
    }

    xSemaphoreGive(meta->mutex);
    return 0;
}

/* ------------------------------------------------------------------ */
orb_subscription_t *orb_subscribe(orb_id_t topic)
{
    if (topic >= ORB_MAX_TOPICS) {
        ESP_LOGE(TAG, "subscribe: invalid topic %d", topic);
        return NULL;
    }

    xSemaphoreTake(s_global_mutex, portMAX_DELAY);

    orb_topic_meta_t *meta = &s_topics[topic];

    /* Auto-advertise with zero size is not allowed; topic must be advertised */
    if (!meta->advertised) {
        /*
         * Allow subscribing before the publisher has advertised.
         * We will lazily create the queue once we know the data size,
         * but for simplicity require advertise first.
         */
        ESP_LOGW(TAG, "subscribe: topic %d not yet advertised", topic);
        xSemaphoreGive(s_global_mutex);
        return NULL;
    }

    if (meta->subscriber_count >= ORB_MAX_SUBSCRIBERS) {
        ESP_LOGE(TAG, "subscribe: topic %d subscriber limit reached", topic);
        xSemaphoreGive(s_global_mutex);
        return NULL;
    }

    /* Allocate subscription handle */
    orb_subscription_t *sub = calloc(1, sizeof(orb_subscription_t));
    if (!sub) {
        xSemaphoreGive(s_global_mutex);
        return NULL;
    }

    /* Create a depth-1 queue sized to hold one message */
    QueueHandle_t q = xQueueCreate(1, meta->data_size);
    if (!q) {
        free(sub);
        xSemaphoreGive(s_global_mutex);
        return NULL;
    }

    sub->topic_id = topic;
    sub->queue = q;

    /* Register in topic metadata */
    xSemaphoreTake(meta->mutex, portMAX_DELAY);
    meta->subscriber_queues[meta->subscriber_count++] = q;
    xSemaphoreGive(meta->mutex);

    xSemaphoreGive(s_global_mutex);

    ESP_LOGI(TAG, "topic %d: subscriber %d added", topic, meta->subscriber_count);
    return sub;
}

/* ------------------------------------------------------------------ */
int orb_copy(orb_subscription_t *sub, void *data)
{
    if (!sub || !data || !sub->queue) {
        return -1;
    }

    /* Non-blocking receive – returns latest value and removes it from queue */
    if (xQueueReceive(sub->queue, data, 0) == pdTRUE) {
        return 0;
    }

    return -1;  /* no new data */
}

/* ------------------------------------------------------------------ */
bool orb_check(orb_subscription_t *sub)
{
    if (!sub || !sub->queue) {
        return false;
    }

    /* Peek without consuming */
    return (uxQueueMessagesWaiting(sub->queue) > 0);
}
