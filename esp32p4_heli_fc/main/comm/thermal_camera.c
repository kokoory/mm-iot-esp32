/*
 * Thermal Camera — FLIR Lepton via PureThermal USB UVC
 *
 * USB Host UVC driver receives Y16 frames at ~9fps.
 * Resolution is auto-negotiated (80x60 or 160x120).
 * Frames are double-buffered in PSRAM for thread-safe access.
 */

#include "thermal_camera.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

static const char *TAG = "thermal_cam";

/* UVC_VS_FORMAT_Y16 is added by tools/patch_uvc_y16.py.
 * If the patch hasn't been applied yet, fall back to 0 (device default). */
#ifndef UVC_VS_FORMAT_Y16
#define UVC_VS_FORMAT_Y16 0
#endif

/* Negotiated resolution (set after stream open) */
static unsigned s_width = 0;
static unsigned s_height = 0;
static size_t s_frame_size = 0;

/* Double buffer in PSRAM */
static uint16_t *s_frame_buf = NULL;
static SemaphoreHandle_t s_frame_mutex = NULL;
static bool s_frame_valid = false;
static bool s_active = false;

/* User callback */
static thermal_frame_cb_t s_user_cb = NULL;
static void *s_user_ctx = NULL;

/* UVC handles */
static uvc_host_stream_hdl_t s_stream = NULL;

/* UDP RTP streaming on port 5601 */
#define THERMAL_RTP_PORT    5601
#define THERMAL_RTP_MTU     1400
#define THERMAL_RTP_PT      96
#define THERMAL_RTP_SSRC    0x54484552  /* 'THER' */
static int s_rtp_sock = -1;
static struct sockaddr_in s_rtp_dest;
static uint16_t s_rtp_seq = 0;
static uint32_t s_rtp_timestamp = 0;

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
/* UDP RTP thermal frame sender                                        */
/* ------------------------------------------------------------------ */

static void rtp_send_thermal_frame(const uint8_t *data, size_t len)
{
    if (s_rtp_sock < 0 || len == 0) return;

    uint8_t pkt[12 + THERMAL_RTP_MTU];
    size_t offset = 0;

    while (offset < len) {
        size_t chunk = len - offset;
        bool last = true;
        if (chunk > THERMAL_RTP_MTU) {
            chunk = THERMAL_RTP_MTU;
            last = false;
        }

        /* RTP header */
        pkt[0] = 0x80;
        pkt[1] = THERMAL_RTP_PT | (last ? 0x80 : 0);  /* marker on last packet */
        pkt[2] = (s_rtp_seq >> 8) & 0xFF;
        pkt[3] = s_rtp_seq & 0xFF;
        pkt[4] = (s_rtp_timestamp >> 24) & 0xFF;
        pkt[5] = (s_rtp_timestamp >> 16) & 0xFF;
        pkt[6] = (s_rtp_timestamp >> 8) & 0xFF;
        pkt[7] = s_rtp_timestamp & 0xFF;
        pkt[8] = (THERMAL_RTP_SSRC >> 24) & 0xFF;
        pkt[9] = (THERMAL_RTP_SSRC >> 16) & 0xFF;
        pkt[10] = (THERMAL_RTP_SSRC >> 8) & 0xFF;
        pkt[11] = THERMAL_RTP_SSRC & 0xFF;

        memcpy(pkt + 12, data + offset, chunk);
        sendto(s_rtp_sock, pkt, 12 + chunk, 0,
               (struct sockaddr *)&s_rtp_dest, sizeof(s_rtp_dest));
        s_rtp_seq++;
        offset += chunk;
    }

    s_rtp_timestamp += 90000 / 9;  /* 90kHz clock / 9fps */
}

/* ------------------------------------------------------------------ */
/* UVC frame callback                                                  */
/* ------------------------------------------------------------------ */

static bool uvc_frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    static uint32_t s_frame_count = 0;

    if (!frame || !frame->data || !s_frame_size) return true;

    s_frame_count++;
    if (s_frame_count == 1) {
        /* First frame: dump format diagnostics to help identify Y16 vs YUY2 */
        const uint8_t *p = (const uint8_t *)frame->data;
        const uint16_t *u16 = (const uint16_t *)frame->data;
        ESP_LOGI(TAG, "First frame: len=%u expect=%u  bytes[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x  u16[0..3]=%u %u %u %u",
                 (unsigned)frame->data_len, (unsigned)s_frame_size,
                 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
                 u16[0], u16[1], u16[2], u16[3]);
    }
    if ((s_frame_count % 90) == 0) {
        ESP_LOGI(TAG, "Thermal frame #%lu  len=%u",
                 (unsigned long)s_frame_count, (unsigned)frame->data_len);
    }

    /* Copy to double buffer under mutex */
    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        size_t copy_len = (frame->data_len < s_frame_size) ? frame->data_len : s_frame_size;
        memcpy(s_frame_buf, frame->data, copy_len);
        s_frame_valid = true;
        xSemaphoreGive(s_frame_mutex);
    }

    /* Send via UDP RTP */
    rtp_send_thermal_frame(frame->data, frame->data_len);

    /* Notify user callback */
    if (s_user_cb) {
        s_user_cb((const uint16_t *)frame->data, frame->data_len, s_user_ctx);
    }

    return true;  /* Driver can reclaim frame buffer immediately */
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
        ESP_LOGW(TAG, "PureThermal disconnected");
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

    /* Open UVC stream — PureThermal Lepton
     * Request Y16 (16-bit grayscale) format explicitly for radiometric data.
     * UVC_VS_FORMAT_Y16 is added by tools/patch_uvc_y16.py at build time.
     * Pre-allocate frame buffers for max possible size (160x120 Y16). */
    const size_t max_frame_size = 160 * 120 * 2;
    uvc_host_stream_config_t stream_config = {
        .event_cb = uvc_stream_callback,
        .frame_cb = uvc_frame_callback,
        .user_ctx = NULL,
        .usb = {
            .vid = 0x1E4E,  /* GroupGets PureThermal VID */
            .pid = 0x0100,  /* PureThermal PID */
        },
        .vs_format = {
            .h_res = 160,   /* Lepton 3.5 native 160x120 */
            .v_res = 120,
            .fps = 9,       /* ~9fps for Lepton */
            .format = UVC_VS_FORMAT_Y16,
        },
        .advanced = {
            .number_of_frame_buffers = 3,
            .frame_size = max_frame_size,
            .frame_heap_caps = MALLOC_CAP_SPIRAM,
            .number_of_urbs = 3,
            .urb_size = 10 * 1024,
        },
    };

    ESP_LOGI(TAG, "Waiting for PureThermal USB connection (Y16 160x120)...");
    ret = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(10000), &s_stream);
    if (ret != ESP_OK) {
        /* Y16 not matched — retry with DEFAULT to accept any format */
        ESP_LOGW(TAG, "Y16 open failed (%s), retrying with DEFAULT format...",
                 esp_err_to_name(ret));
        stream_config.vs_format.format = 0;  /* device default */
        ret = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(10000), &s_stream);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "UVC stream open failed: %s (is PureThermal connected?)",
                     esp_err_to_name(ret));
            return ret;
        }
    }

    /* Read negotiated format and set frame dimensions */
    uvc_host_stream_format_t negotiated = {0};
    if (uvc_host_stream_format_get(s_stream, &negotiated) == ESP_OK) {
        s_width = negotiated.h_res;
        s_height = negotiated.v_res;
        s_frame_size = s_width * s_height * 2;  /* Y16 = 2 bytes/pixel */
        ESP_LOGI(TAG, "Negotiated: %ux%u @ %.1f fps (format=%d, frame=%u bytes)",
                 s_width, s_height, negotiated.fps, negotiated.format, (unsigned)s_frame_size);
    } else {
        /* Fallback */
        s_width = 80;
        s_height = 60;
        s_frame_size = 80 * 60 * 2;
        ESP_LOGW(TAG, "Could not read format, assuming 80x60");
    }

    /* Allocate frame buffer in PSRAM to match actual resolution */
    s_frame_buf = heap_caps_calloc(1, s_frame_size, MALLOC_CAP_SPIRAM);
    if (!s_frame_buf) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer (%u bytes)", (unsigned)s_frame_size);
        return ESP_ERR_NO_MEM;
    }

    /* Create UDP RTP socket for thermal streaming on port 5601 */
    s_rtp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_rtp_sock >= 0) {
        memset(&s_rtp_dest, 0, sizeof(s_rtp_dest));
        s_rtp_dest.sin_family = AF_INET;
        s_rtp_dest.sin_port = htons(THERMAL_RTP_PORT);
        s_rtp_dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        int broadcast = 1;
        setsockopt(s_rtp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
        int flags = fcntl(s_rtp_sock, F_GETFL, 0);
        fcntl(s_rtp_sock, F_SETFL, flags | O_NONBLOCK);
        s_rtp_seq = 0;
        s_rtp_timestamp = 0;
        ESP_LOGI(TAG, "Thermal RTP socket ready (broadcast port %d)", THERMAL_RTP_PORT);
    }

    s_active = true;
    ESP_LOGI(TAG, "PureThermal Lepton connected (%ux%u)", s_width, s_height);
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
            memcpy(buf, s_frame_buf, s_frame_size);
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
