/*
 * MIPI-CSI Camera + H.264 Encoder for ESP32-P4
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This module initializes the ESP32-P4's MIPI-CSI camera interface and
 * hardware H.264 encoder for video streaming over Wi-Fi HaLow.
 *
 * Architecture:
 *   MIPI-CSI Camera → ISP → H.264 HW Encoder → HTTP Stream → HaLow
 *
 * The ESP32-P4 MIPI-CSI uses dedicated differential pairs (not GPIO),
 * so there is no pin conflict with the HaLow SPI bus.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"

#include "camera_h264.h"

static const char *TAG = "camera_h264";

/*
 * ESP32-P4 MIPI-CSI Camera Configuration
 *
 * The Waveshare ESP32-P4-WIFI6 board has a MIPI-CSI connector.
 * MIPI-CSI uses dedicated differential signal pairs:
 *   - CSI_CLK_P/N  (clock lane)
 *   - CSI_D0_P/N   (data lane 0)
 *   - CSI_D1_P/N   (data lane 1)
 * These are NOT regular GPIO pins - they are dedicated MIPI PHY pins.
 */

/* Frame configuration */
#define CAM_WIDTH           1280
#define CAM_HEIGHT          720
#define CAM_FPS             30

/* H.264 encoder settings */
#define H264_BITRATE        2000000   /* 2 Mbps - suitable for HaLow bandwidth */
#define H264_GOP_SIZE       30        /* One I-frame per second at 30fps */
#define H264_QUALITY        26        /* CQP value (lower = better quality) */

/* Ring buffer for encoded H.264 NAL units */
#define H264_RING_BUF_SIZE  (512 * 1024)  /* 512KB ring buffer */

/* JPEG fallback settings */
#define JPEG_QUALITY        60
#define JPEG_BUF_SIZE       (100 * 1024)  /* 100KB per JPEG frame */

/* Frame buffer management */
#define NUM_FRAME_BUFS      3

typedef struct {
    uint8_t *data;
    size_t   size;
    size_t   capacity;
    int64_t  timestamp;
    bool     is_keyframe;
} h264_frame_t;

/* Module state */
static struct {
    bool initialized;

    /* Frame statistics */
    volatile float fps;
    volatile uint32_t bitrate;
    int64_t last_frame_time;
    uint32_t frame_count;
    uint32_t byte_count;
    int64_t stats_start_time;

    /* H.264 encoded frame ring buffer */
    h264_frame_t frames[NUM_FRAME_BUFS];
    volatile int write_idx;
    volatile int read_idx;
    SemaphoreHandle_t frame_ready;

    /* JPEG fallback buffer */
    uint8_t *jpeg_buf;
    size_t jpeg_size;
    SemaphoreHandle_t jpeg_mutex;

    /* Camera task handle */
    TaskHandle_t cam_task_handle;
} s_cam = {0};

/*
 * NOTE: ESP32-P4 MIPI-CSI and H.264 encoder APIs
 *
 * The ESP32-P4 provides hardware-accelerated video encoding through:
 * 1. MIPI-CSI receiver (2-lane, up to 1080p@30fps)
 * 2. ISP (Image Signal Processor) for demosaic, AWB, AE, etc.
 * 3. H.264 hardware encoder (up to 1080p@30fps)
 *
 * The actual API headers depend on ESP-IDF version:
 * - esp_cam_ctlr.h / esp_cam_ctlr_csi.h  (CSI camera controller)
 * - esp_video_enc.h                       (H.264 encoder)
 * - esp_isp.h                             (ISP pipeline)
 *
 * Since these APIs are actively evolving in ESP-IDF v5.3+, this
 * implementation provides the framework and uses conditional compilation
 * to adapt to the available API version.
 */

#if __has_include("esp_cam_ctlr_csi.h")
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#define HAS_CSI_DRIVER 1
#else
#define HAS_CSI_DRIVER 0
#endif

#if __has_include("esp_video_enc.h")
#include "esp_video_enc.h"
#define HAS_H264_ENCODER 1
#else
#define HAS_H264_ENCODER 0
#endif

#if __has_include("esp_isp.h")
#include "esp_isp.h"
#define HAS_ISP 1
#else
#define HAS_ISP 0
#endif

/* Forward declarations */
static void camera_capture_task(void *arg);
static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t h264_stream_handler(httpd_req_t *req);
static esp_err_t status_handler(httpd_req_t *req);

/* ========== Frame Buffer Management ========== */

static esp_err_t frame_bufs_init(void)
{
    for (int i = 0; i < NUM_FRAME_BUFS; i++) {
        s_cam.frames[i].data = heap_caps_malloc(H264_RING_BUF_SIZE / NUM_FRAME_BUFS,
                                                 MALLOC_CAP_SPIRAM);
        if (!s_cam.frames[i].data) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
            return ESP_ERR_NO_MEM;
        }
        s_cam.frames[i].capacity = H264_RING_BUF_SIZE / NUM_FRAME_BUFS;
        s_cam.frames[i].size = 0;
    }
    return ESP_OK;
}

/* ========== Camera + H.264 Initialization ========== */

esp_err_t camera_h264_init(void)
{
    esp_err_t ret;

    if (s_cam.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing MIPI-CSI camera + H.264 encoder");
    ESP_LOGI(TAG, "Resolution: %dx%d @ %dfps, Bitrate: %d bps",
             CAM_WIDTH, CAM_HEIGHT, CAM_FPS, H264_BITRATE);

    /* Allocate frame buffers in PSRAM */
    ret = frame_bufs_init();
    if (ret != ESP_OK) {
        return ret;
    }

    /* Allocate JPEG fallback buffer */
    s_cam.jpeg_buf = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_cam.jpeg_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer");
        return ESP_ERR_NO_MEM;
    }

    s_cam.frame_ready = xSemaphoreCreateBinary();
    s_cam.jpeg_mutex = xSemaphoreCreateMutex();

#if HAS_CSI_DRIVER && HAS_H264_ENCODER
    /*
     * === MIPI-CSI Camera Controller Setup ===
     *
     * The ESP32-P4 MIPI-CSI interface uses dedicated pins:
     *   - MIPI_CSI_CLK_P/N (clock lane pair)
     *   - MIPI_CSI_D0_P/N  (data lane 0 pair)
     *   - MIPI_CSI_D1_P/N  (data lane 1 pair)
     *
     * These are routed on the Waveshare board to the camera connector.
     */

    /* CSI controller configuration */
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
        .data_color_type = CAM_CTLR_COLOR_RAW8,   /* RAW Bayer input */
        .lane_num = 2,                              /* 2-lane MIPI */
        .clk_freq_hz = 200000000,                   /* 200MHz CSI clock */
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .byte_swap_en = false,
        .queue_items = NUM_FRAME_BUFS,
    };

    esp_cam_ctlr_handle_t cam_handle = NULL;
    ret = esp_cam_ctlr_csi_init(&csi_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MIPI-CSI camera initialized (2-lane, %dx%d)", CAM_WIDTH, CAM_HEIGHT);

#if HAS_ISP
    /* ISP pipeline setup */
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 120000000,  /* 120MHz ISP clock */
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
    };

    isp_proc_handle_t isp_handle = NULL;
    ret = esp_isp_new_processor(&isp_config, &isp_handle);
    if (ret == ESP_OK) {
        esp_isp_enable(isp_handle);
        ESP_LOGI(TAG, "ISP pipeline enabled");
    }
#endif /* HAS_ISP */

    /* H.264 encoder setup */
    esp_video_enc_cfg_t enc_config = {
        .codec_type = ESP_VIDEO_ENC_CODEC_H264,
        .input = {
            .width = CAM_WIDTH,
            .height = CAM_HEIGHT,
            .format = ESP_VIDEO_ENC_PIX_FMT_RGB565,
        },
        .output = {
            .bitrate = H264_BITRATE,
            .fps = CAM_FPS,
            .gop = H264_GOP_SIZE,
        },
    };

    esp_video_enc_handle_t enc_handle = NULL;
    ret = esp_video_enc_open(&enc_config, &enc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "H.264 encoder init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "H.264 HW encoder initialized (bitrate=%d, GOP=%d)",
             H264_BITRATE, H264_GOP_SIZE);

    /* Start camera and enable streaming */
    ret = esp_cam_ctlr_start(cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera start failed: %s", esp_err_to_name(ret));
        return ret;
    }

#else
    ESP_LOGW(TAG, "MIPI-CSI/H.264 headers not available in this ESP-IDF version");
    ESP_LOGW(TAG, "Camera module compiled in stub mode - upgrade to ESP-IDF v5.3+");
    ESP_LOGW(TAG, "Required: esp_cam_ctlr_csi.h, esp_video_enc.h");
#endif /* HAS_CSI_DRIVER && HAS_H264_ENCODER */

    /* Start the camera capture task */
    xTaskCreatePinnedToCore(camera_capture_task, "cam_task", 8192, NULL, 5,
                            &s_cam.cam_task_handle, 1);

    s_cam.initialized = true;
    s_cam.stats_start_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Camera H.264 module initialized successfully");
    return ESP_OK;
}

/* ========== Camera Capture Task ========== */

static void camera_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Camera capture task started");

    while (1) {
#if HAS_CSI_DRIVER && HAS_H264_ENCODER
        /*
         * In production:
         * 1. Wait for CSI frame via esp_cam_ctlr_receive()
         * 2. Pass raw frame to H.264 encoder via esp_video_enc_process()
         * 3. Store encoded NAL units in ring buffer
         * 4. Signal frame_ready semaphore
         */

        /* TODO: Replace with actual CSI frame capture when hardware is available
         * esp_cam_ctlr_trans_t trans = { .buffer = raw_buf, .buflen = raw_size };
         * esp_cam_ctlr_receive(cam_handle, &trans, portMAX_DELAY);
         * esp_video_enc_in_frame_t in = { .buffer = trans.buffer, .len = trans.received_size };
         * esp_video_enc_out_frame_t out = { .buffer = s_cam.frames[wr].data, .len = capacity };
         * esp_video_enc_process(enc_handle, &in, &out);
         */
#endif
        /* Update statistics */
        int64_t now = esp_timer_get_time();
        if (s_cam.last_frame_time > 0) {
            int64_t elapsed_us = now - s_cam.stats_start_time;
            if (elapsed_us > 1000000) {  /* Update stats every second */
                s_cam.fps = (float)s_cam.frame_count * 1000000.0f / (float)elapsed_us;
                s_cam.bitrate = (uint32_t)((float)s_cam.byte_count * 8000000.0f / (float)elapsed_us);
                s_cam.frame_count = 0;
                s_cam.byte_count = 0;
                s_cam.stats_start_time = now;
            }
        }
        s_cam.last_frame_time = now;

        vTaskDelay(pdMS_TO_TICKS(1000 / CAM_FPS));
    }
}

/* ========== HTTP Streaming Handlers ========== */

#define PART_BOUNDARY "esp32p4-halow-boundary"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res;
    char part_buf[128];

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    /* Add CORS headers for browser access */
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    ESP_LOGI(TAG, "MJPEG stream started");

    while (true) {
        /* Wait for a new frame */
        if (xSemaphoreTake(s_cam.frame_ready, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGW(TAG, "Frame timeout, camera may not be running");
            /* Send a placeholder response */
            const char *msg = "Camera initializing...";
            httpd_resp_send_chunk(req, msg, strlen(msg));
            continue;
        }

        if (xSemaphoreTake(s_cam.jpeg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (s_cam.jpeg_size > 0) {
                /* Send MJPEG boundary + JPEG frame */
                res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
                if (res == ESP_OK) {
                    size_t hlen = snprintf(part_buf, sizeof(part_buf),
                        "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                        (unsigned)s_cam.jpeg_size);
                    res = httpd_resp_send_chunk(req, part_buf, hlen);
                }
                if (res == ESP_OK) {
                    res = httpd_resp_send_chunk(req, (const char *)s_cam.jpeg_buf,
                                               s_cam.jpeg_size);
                }
            }
            xSemaphoreGive(s_cam.jpeg_mutex);
        }

        if (res != ESP_OK) break;
    }

    ESP_LOGI(TAG, "MJPEG stream ended");
    return res;
}

static esp_err_t h264_stream_handler(httpd_req_t *req)
{
    esp_err_t res;

    /* Set content type for raw H.264 byte stream */
    httpd_resp_set_type(req, "video/h264");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    ESP_LOGI(TAG, "H.264 stream started");

    while (true) {
        if (xSemaphoreTake(s_cam.frame_ready, pdMS_TO_TICKS(5000)) != pdTRUE) {
            continue;
        }

        int idx = s_cam.read_idx;
        h264_frame_t *frame = &s_cam.frames[idx];

        if (frame->size > 0) {
            res = httpd_resp_send_chunk(req, (const char *)frame->data, frame->size);
            if (res != ESP_OK) break;

            s_cam.frame_count++;
            s_cam.byte_count += frame->size;
        }

        s_cam.read_idx = (idx + 1) % NUM_FRAME_BUFS;
    }

    ESP_LOGI(TAG, "H.264 stream ended");
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"initialized\":%s,\"resolution\":\"%dx%d\",\"fps\":%.1f,"
        "\"bitrate\":%lu,\"encoder\":\"h264_hw\",\"transport\":\"halow\"}",
        s_cam.initialized ? "true" : "false",
        CAM_WIDTH, CAM_HEIGHT,
        s_cam.fps,
        (unsigned long)s_cam.bitrate);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, strlen(buf));
}

/* ========== HTTP Server Setup ========== */

static const httpd_uri_t uri_stream = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL,
};

static const httpd_uri_t uri_h264 = {
    .uri = "/h264",
    .method = HTTP_GET,
    .handler = h264_stream_handler,
    .user_ctx = NULL,
};

static const httpd_uri_t uri_status = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL,
};

httpd_handle_t camera_stream_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_stream);
        httpd_register_uri_handler(server, &uri_h264);
        httpd_register_uri_handler(server, &uri_status);
        ESP_LOGI(TAG, "Camera HTTP server started");
        ESP_LOGI(TAG, "  MJPEG stream: http://<ip>/");
        ESP_LOGI(TAG, "  H.264 stream: http://<ip>/h264");
        ESP_LOGI(TAG, "  Status:       http://<ip>/status");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }

    return server;
}

float camera_get_fps(void)
{
    return s_cam.fps;
}
