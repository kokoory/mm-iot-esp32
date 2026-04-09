/*
 * USB Camera — Logitech C920 (or compatible UVC webcam)
 *
 * USB Host UVC driver receives MJPEG frames.
 * Frames are double-buffered in PSRAM for thread-safe access.
 *
 * Previously: FLIR Lepton via PureThermal (Y16 160x120 @9fps)
 * Now: Logitech C920 (MJPEG 640x480 @15fps)
 */

#include "thermal_camera.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

static const char *TAG = "usb_cam";

/* USB camera configuration — Logitech C920 */
#define USB_CAM_VID         0x046D  /* Logitech */
#define USB_CAM_PID         0x082D  /* C920 HD Pro Webcam */
#define USB_CAM_WIDTH       640
#define USB_CAM_HEIGHT      480
#define USB_CAM_FPS         15
#define USB_CAM_MAX_FRAME   (100 * 1024)  /* 100KB max MJPEG frame */

/* Negotiated resolution (set after stream open) */
static unsigned s_width = 0;
static unsigned s_height = 0;
static size_t s_frame_size = 0;

/* Double buffer in PSRAM (MJPEG frames are variable size) */
static uint8_t *s_frame_buf = NULL;
static size_t s_frame_len = 0;         /* Actual MJPEG frame length */
static SemaphoreHandle_t s_frame_mutex = NULL;
static bool s_frame_valid = false;
static bool s_active = false;

/* User callback */
static thermal_frame_cb_t s_user_cb = NULL;
static void *s_user_ctx = NULL;

/* UVC handles */
static uvc_host_stream_hdl_t s_stream = NULL;


/* ------------------------------------------------------------------ */
/* USB Host library event handler task                                 */
/* ------------------------------------------------------------------ */

static void usb_host_lib_task(void *param)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            /* All clients deregistered */
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            /* All devices freed */
        }
    }
}


/* ------------------------------------------------------------------ */
/* UVC frame callback                                                  */
/* ------------------------------------------------------------------ */

static bool uvc_frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    static uint32_t s_frame_count = 0;

    if (!frame || !frame->data || frame->data_len == 0) return true;

    s_frame_count++;
    if (s_frame_count == 1) {
        const uint8_t *p = (const uint8_t *)frame->data;
        ESP_LOGI(TAG, "First frame: len=%u  bytes[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x",
                 (unsigned)frame->data_len,
                 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        /* MJPEG should start with FF D8 (SOI marker) */
        if (p[0] == 0xFF && p[1] == 0xD8) {
            ESP_LOGI(TAG, "MJPEG format confirmed (SOI marker)");
        } else {
            ESP_LOGW(TAG, "Unexpected format — not MJPEG?");
        }
    }
    if ((s_frame_count % 150) == 0) {
        ESP_LOGI(TAG, "USB cam frame #%lu  len=%u",
                 (unsigned long)s_frame_count, (unsigned)frame->data_len);
    }

    /* Copy to double buffer under mutex */
    if (xSemaphoreTake(s_frame_mutex, 0) == pdTRUE) {
        size_t copy_len = (frame->data_len < USB_CAM_MAX_FRAME) ? frame->data_len : USB_CAM_MAX_FRAME;
        memcpy(s_frame_buf, frame->data, copy_len);
        s_frame_len = copy_len;
        s_frame_valid = true;
        xSemaphoreGive(s_frame_mutex);
    }

    return true;
}

/* ------------------------------------------------------------------ */
/* UVC stream event callback                                           */
/* ------------------------------------------------------------------ */

static void uvc_stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGW(TAG, "UVC transfer error");
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "USB camera disconnected");
        s_active = false;
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame buffer overflow");
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t thermal_camera_init(thermal_frame_cb_t frame_cb, void *user_ctx)
{
    s_user_cb = frame_cb;
    s_user_ctx = user_ctx;

    s_frame_mutex = xSemaphoreCreateMutex();
    if (!s_frame_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Install USB Host Library */
    usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB host install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start USB Host event handling task */
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host", 4096, NULL, 2, NULL, 1);

    /* Install UVC driver */
    uvc_host_driver_config_t uvc_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 5,
        .xCoreID = 1,
        .create_background_task = true,
    };
    ret = uvc_host_install(&uvc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UVC host install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Open UVC stream — Logitech C920 (or any UVC webcam)
     * Request MJPEG format for compressed video.
     * If VID/PID match fails, retry with VID/PID=0 to accept any UVC device. */
    uvc_host_stream_config_t stream_config = {
        .event_cb = uvc_stream_callback,
        .frame_cb = uvc_frame_callback,
        .user_ctx = NULL,
        .usb = {
            .vid = USB_CAM_VID,
            .pid = USB_CAM_PID,
        },
        .vs_format = {
            .h_res = USB_CAM_WIDTH,
            .v_res = USB_CAM_HEIGHT,
            .fps = USB_CAM_FPS,
            .format = UVC_VS_FORMAT_MJPEG,
        },
        .advanced = {
            .number_of_frame_buffers = 3,
            .frame_size = USB_CAM_MAX_FRAME,
            .frame_heap_caps = MALLOC_CAP_SPIRAM,
            .number_of_urbs = 5,
            .urb_size = 20 * 1024,  /* Larger URBs for MJPEG data */
        },
    };

    ESP_LOGI(TAG, "Waiting for USB camera (VID=0x%04X PID=0x%04X MJPEG %dx%d@%dfps)...",
             USB_CAM_VID, USB_CAM_PID, USB_CAM_WIDTH, USB_CAM_HEIGHT, USB_CAM_FPS);
    ret = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(10000), &s_stream);
    if (ret != ESP_OK) {
        /* Exact VID/PID failed — retry accepting any UVC device */
        ESP_LOGW(TAG, "C920 open failed (%s), retrying with any UVC device...",
                 esp_err_to_name(ret));
        stream_config.usb.vid = 0;
        stream_config.usb.pid = 0;
        ret = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(10000), &s_stream);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "UVC stream open failed: %s (is USB camera connected?)",
                     esp_err_to_name(ret));
            return ret;
        }
    }

    s_width = stream_config.vs_format.h_res;
    s_height = stream_config.vs_format.v_res;
    s_frame_size = USB_CAM_MAX_FRAME;  /* MJPEG: variable size, allocate max */
    ESP_LOGI(TAG, "Opened: %ux%u MJPEG @%dfps (max frame=%uKB)",
             s_width, s_height, stream_config.vs_format.fps,
             (unsigned)(s_frame_size / 1024));

    /* Allocate frame buffer in PSRAM */
    s_frame_buf = heap_caps_calloc(1, s_frame_size, MALLOC_CAP_SPIRAM);
    if (!s_frame_buf) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer (%u bytes)", (unsigned)s_frame_size);
        return ESP_ERR_NO_MEM;
    }

    s_active = true;
    ESP_LOGI(TAG, "USB camera connected (%ux%u MJPEG)", s_width, s_height);
    return ESP_OK;
}

esp_err_t thermal_camera_start(void)
{
    if (!s_stream) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = uvc_host_stream_start(s_stream);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Thermal streaming started");
    }
    return ret;
}

esp_err_t thermal_camera_stop(void)
{
    if (!s_stream) return ESP_ERR_INVALID_STATE;
    s_active = false;
    return uvc_host_stream_stop(s_stream);
}

bool thermal_camera_get_frame(uint16_t *buf)
{
    if (!s_frame_valid || !s_frame_mutex) return false;

    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_frame_valid) {
            memcpy(buf, s_frame_buf, s_frame_len);
            xSemaphoreGive(s_frame_mutex);
            return true;
        }
        xSemaphoreGive(s_frame_mutex);
    }
    return false;
}

bool thermal_camera_get_jpeg(uint8_t *buf, size_t buf_size, size_t *out_len)
{
    if (!s_frame_valid || !s_frame_mutex) return false;

    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_frame_valid && s_frame_len <= buf_size) {
            memcpy(buf, s_frame_buf, s_frame_len);
            *out_len = s_frame_len;
            xSemaphoreGive(s_frame_mutex);
            return true;
        }
        xSemaphoreGive(s_frame_mutex);
    }
    return false;
}

bool thermal_camera_is_active(void)
{
    return s_active;
}

unsigned thermal_camera_width(void)
{
    return s_width;
}

unsigned thermal_camera_height(void)
{
    return s_height;
}

size_t thermal_camera_frame_size(void)
{
    return s_frame_size;
}
