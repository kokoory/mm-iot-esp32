/*
 * Copyright 2022-2023 Morse Micro
 *
 * This file is licensed under terms that can be found in the LICENSE.md file in the root
 * directory of the Morse Micro IoT SDK software package.
 */

/**
 * @file
 * @brief Web camera streaming over Wi-Fi HaLow.
 *
 * This example demonstrates streaming MJPEG video from the XIAO ESP32S3 Sense
 * camera (OV2640/OV5640) over a Wi-Fi HaLow network.
 *
 * @note It is assumed that you have followed the steps in the @ref GETTING_STARTED guide and are
 * therefore familiar with how to build, flash, and monitor an application using the MM-IoT-SDK
 * framework.
 */

#include <endian.h>
#include <string.h>
#include "mmosal.h"
#include "mmwlan.h"
#include "mmipal.h"

#include "mm_app_common.h"

#include <esp_system.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_http_server.h"

static const char *TAG = "esp32-cam Webserver";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

#define CONFIG_XCLK_FREQ 20000000

/*
 * XIAO ESP32S3 Sense Camera Pin Definitions
 *
 * These pins are fixed on the XIAO ESP32S3 Sense board and connect to the
 * onboard OV2640/OV5640 camera module via the detachable camera connector.
 *
 * Note: The SD card on the XIAO ESP32S3 Sense uses GPIO 7, 8, 9 which conflict
 * with the Wi-Fi HaLow SPI bus. SD card cannot be used simultaneously with HaLow.
 */
static esp_err_t init_camera(void)
{
    camera_config_t camera_config = {
        .pin_pwdn  = -1,
        .pin_reset = -1,
        .pin_xclk  = 10,
        .pin_sccb_sda = 40,
        .pin_sccb_scl = 39,

        .pin_d7    = 48,
        .pin_d6    = 11,
        .pin_d5    = 12,
        .pin_d4    = 14,
        .pin_d3    = 16,
        .pin_d2    = 18,
        .pin_d1    = 17,
        .pin_d0    = 15,
        .pin_vsync = 38,
        .pin_href  = 47,
        .pin_pclk  = 13,

        .xclk_freq_hz = CONFIG_XCLK_FREQ,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_VGA,

        .jpeg_quality = 10,
        .fb_count     = 1,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t *_jpg_buf;
    char part_buf[64];
    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    while (true)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if (fb->format != PIXFORMAT_JPEG)
        {
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if (!jpeg_converted)
            {
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        }
        else
        {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        if (fb->format != PIXFORMAT_JPEG)
        {
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);

        if (res != ESP_OK)
        {
            break;
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)",
                 (uint32_t)(_jpg_buf_len / 1024),
                 (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

    last_frame = 0;
    return res;
}

httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = jpg_stream_httpd_handler,
    .user_ctx = NULL,
};

httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t stream_httpd = NULL;

    if (httpd_start(&stream_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd, &uri_get);
    }

    return stream_httpd;
}

/**
 * Main entry point to the application. This will be invoked in a thread once operating system
 * and hardware initialization has completed. It may return, but it does not have to.
 */
void app_main(void)
{
    esp_err_t err;

    printf("\n\nWeb Camera Serve Demo (Built " __DATE__ " " __TIME__ ")\n\n");

    /* Initialize and connect to WiFi, blocks till connected */
    app_wlan_init();
    app_wlan_start();

    esp_event_loop_create_default();

    err = init_camera();
    if (err != ESP_OK)
    {
        printf("err: %s\n", esp_err_to_name(err));
        return;
    }

    setup_server();
    ESP_LOGI(TAG, "ESP32 CAM Web Server is up and running\n");

    while (1)
    {
        app_wlan_arp_send();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
