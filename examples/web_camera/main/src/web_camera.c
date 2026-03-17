/*
 * Copyright 2024 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Web Camera Example - XIAO ESP32S3 Sense + Wi-Fi HaLow
 *
 * This example demonstrates streaming MJPEG video from the XIAO ESP32S3 Sense
 * camera (OV2640/OV5640) over a Wi-Fi HaLow network. It connects to a HaLow AP
 * as a station, then serves a simple web page with live camera streaming.
 *
 * Endpoints:
 *   /       - Web page with embedded MJPEG stream
 *   /stream - Raw MJPEG stream (multipart/x-mixed-replace)
 *   /capture - Single JPEG snapshot
 */

#include <string.h>
#include <stdio.h>

#include "mmosal.h"
#include "mmwlan.h"
#include "mmipal.h"
#include "mm_app_common.h"

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "web_camera";

/*
 * XIAO ESP32S3 Sense Camera Pin Definitions
 *
 * These pins are fixed on the XIAO ESP32S3 Sense board and connect to the
 * onboard OV2640/OV5640 camera module via the detachable camera connector.
 *
 * Note: The SD card on the XIAO ESP32S3 Sense uses GPIO 7, 8, 9 which conflict
 * with the Wi-Fi HaLow SPI bus. SD card cannot be used simultaneously with HaLow.
 */
#define CAMERA_PIN_PWDN    (-1)
#define CAMERA_PIN_RESET   (-1)
#define CAMERA_PIN_XCLK    10
#define CAMERA_PIN_SIOD    40
#define CAMERA_PIN_SIOC    39
#define CAMERA_PIN_Y9      48
#define CAMERA_PIN_Y8      11
#define CAMERA_PIN_Y7      12
#define CAMERA_PIN_Y6      14
#define CAMERA_PIN_Y5      16
#define CAMERA_PIN_Y4      18
#define CAMERA_PIN_Y3      17
#define CAMERA_PIN_Y2      15
#define CAMERA_PIN_VSYNC   38
#define CAMERA_PIN_HREF    47
#define CAMERA_PIN_PCLK    13

/* HTTP multipart stream boundary */
#define STREAM_BOUNDARY    "frame"
#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY
#define STREAM_PART_HDR    "\r\n--" STREAM_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

/* Simple HTML page with embedded MJPEG stream */
static const char index_html[] =
    "<!DOCTYPE html>"
    "<html><head><title>XIAO HaLow Camera</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:sans-serif;text-align:center;background:#1a1a2e;color:#e0e0e0;margin:0;padding:20px}"
    "h1{color:#00d4ff}img{max-width:100%;border:2px solid #00d4ff;border-radius:8px}"
    ".info{margin:10px;padding:10px;background:#16213e;border-radius:8px;font-size:14px}"
    "</style></head><body>"
    "<h1>XIAO ESP32S3 Sense</h1>"
    "<p>Wi-Fi HaLow (802.11ah) Camera Stream</p>"
    "<img src='/stream' alt='Camera Stream'>"
    "<div class='info'>"
    "<p><a href='/capture' style='color:#00d4ff'>Capture Snapshot (JPEG)</a></p>"
    "</div>"
    "</body></html>";

static esp_err_t init_camera(void)
{
    camera_config_t config = {
        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,
        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_sccb_sda = CAMERA_PIN_SIOD,
        .pin_sccb_scl = CAMERA_PIN_SIOC,
        .pin_d7 = CAMERA_PIN_Y9,
        .pin_d6 = CAMERA_PIN_Y8,
        .pin_d5 = CAMERA_PIN_Y7,
        .pin_d4 = CAMERA_PIN_Y6,
        .pin_d3 = CAMERA_PIN_Y5,
        .pin_d2 = CAMERA_PIN_Y4,
        .pin_d1 = CAMERA_PIN_Y3,
        .pin_d0 = CAMERA_PIN_Y2,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_pclk = CAMERA_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

/* GET / - Serve the HTML page */
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, sizeof(index_html) - 1);
}

/* GET /capture - Single JPEG snapshot */
static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

/* GET /stream - MJPEG stream */
static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;
    char part_hdr[128];

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "15");

    ESP_LOGI(TAG, "Stream started");

    while (true)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed during stream");
            res = ESP_FAIL;
            break;
        }

        int hdr_len = snprintf(part_hdr, sizeof(part_hdr), STREAM_PART_HDR, fb->len);

        res = httpd_resp_send_chunk(req, part_hdr, hdr_len);
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }

        esp_camera_fb_return(fb);

        if (res != ESP_OK)
        {
            ESP_LOGI(TAG, "Stream ended (client disconnected)");
            break;
        }
    }

    return res;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 4;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
    };
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
    };
    httpd_register_uri_handler(server, &capture_uri);

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
    };
    httpd_register_uri_handler(server, &stream_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return server;
}

void app_main(void)
{
    printf("\n\nXIAO ESP32S3 Sense - Wi-Fi HaLow Web Camera (Built " __DATE__ " " __TIME__ ")\n\n");

    /* Initialize camera first */
    if (init_camera() != ESP_OK)
    {
        printf("ERROR: Camera initialization failed. Ensure OV2640/OV5640 is connected.\n");
        return;
    }

    /* Initialize WLAN and connect to HaLow AP */
    app_wlan_init();
    app_wlan_start();

    printf("Wi-Fi HaLow connected. Starting web camera server...\n");

    /* Start HTTP server for camera streaming */
    httpd_handle_t server = start_webserver();
    if (server == NULL)
    {
        printf("ERROR: Failed to start web server\n");
        return;
    }

    printf("\n========================================\n");
    printf("  Web Camera Server Running!\n");
    printf("  Open http://<device-ip>/ in browser\n");
    printf("  Endpoints:\n");
    printf("    /        - Live stream page\n");
    printf("    /stream  - MJPEG stream\n");
    printf("    /capture - JPEG snapshot\n");
    printf("========================================\n\n");
}
