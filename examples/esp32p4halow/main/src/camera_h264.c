/*
 * MIPI-CSI Camera + HW JPEG Encoder for ESP32-P4
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pipeline: OV5647 → MIPI-CSI → ISP (RAW8→RGB565) → HW JPEG → HTTP Stream
 *
 * Based on ESP-IDF v5.3 camera_dsi example and adapted for HTTP streaming
 * over Wi-Fi HaLow instead of DSI display output.
 *
 * References:
 *   - ESP-IDF examples/peripherals/camera/camera_dsi
 *   - esp_cam_sensor component (OV5647 driver)
 *   - ESP32-P4 hardware JPEG encoder
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"

#include "camera_h264.h"

static const char *TAG = "camera_h264";

/*
 * Camera Configuration
 *
 * Waveshare ESP32-P4-WIFI6 board:
 *   - MIPI-CSI: dedicated differential pairs (not GPIO)
 *   - SCCB (I2C): GPIO8 (SCL), GPIO7 (SDA)
 *   - LDO channel 3 at 2500mV for MIPI PHY
 *
 * OV5647 supported formats (from esp_cam_sensor):
 *   - MIPI_2lane_24Minput_RAW8_800x640_50fps
 *   - MIPI_2lane_24Minput_RAW8_800x1280_50fps
 *   - MIPI_2lane_24Minput_RAW8_1024x600_30fps
 */

/* I2C / SCCB pins for camera sensor */
#define CAM_SCCB_SCL_IO     8
#define CAM_SCCB_SDA_IO     7
#define CAM_SCCB_FREQ       100000

/* LDO for MIPI PHY */
#define MIPI_LDO_CHAN_ID     3
#define MIPI_LDO_VOLTAGE_MV  2500

/* Camera format - must match a format string from esp_cam_sensor OV5647 driver */
#define CAM_FORMAT          "MIPI_2lane_24Minput_RAW8_800x640_50fps"
#define CAM_WIDTH           800
#define CAM_HEIGHT          640

/* MIPI CSI lane bitrate */
#define CSI_LANE_BITRATE_MBPS  200

/* JPEG quality (1-100, higher = better quality, larger file) */
#define JPEG_QUALITY        80
#define JPEG_BUF_SIZE       (200 * 1024)  /* 200KB should be enough for 800x640 */

/* Double buffer for JPEG output */
#define NUM_JPEG_BUFS       2

/* Check for required ESP-IDF components */
#if __has_include("esp_cam_ctlr_csi.h") && __has_include("driver/isp.h")
#define HAS_CAMERA_PIPELINE 1
#else
#define HAS_CAMERA_PIPELINE 0
#endif

#if __has_include("driver/jpeg_encode.h")
#define HAS_HW_JPEG 1
#else
#define HAS_HW_JPEG 0
#endif

#if HAS_CAMERA_PIPELINE
#include "esp_ldo_regulator.h"
#include "driver/i2c_master.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#endif

#if HAS_HW_JPEG
#include "driver/jpeg_encode.h"
#endif

/* Module state */
static struct {
    bool initialized;

    /* JPEG double buffer */
    uint8_t *jpeg_buf[NUM_JPEG_BUFS];
    size_t jpeg_size[NUM_JPEG_BUFS];
    volatile int jpeg_write_idx;
    volatile int jpeg_read_idx;
    SemaphoreHandle_t frame_ready;
    SemaphoreHandle_t jpeg_mutex;

    /* Frame statistics */
    volatile float fps;
    uint32_t frame_count;
    int64_t stats_start_time;

    /* Camera task handle */
    TaskHandle_t cam_task_handle;

#if HAS_CAMERA_PIPELINE
    esp_cam_ctlr_handle_t cam_handle;
#endif
#if HAS_HW_JPEG
    jpeg_encoder_handle_t jpeg_handle;
#endif

    /* Raw frame buffer (RGB565 from ISP) */
    uint8_t *raw_buf;
    size_t raw_buf_size;
} s_cam = {0};

/* Forward declarations */
static void camera_capture_task(void *arg);
static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t status_handler(httpd_req_t *req);

#if HAS_CAMERA_PIPELINE
/* CSI callback: provide a new buffer for next frame capture */
static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans,
                                        void *user_data)
{
    esp_cam_ctlr_trans_t *ref = (esp_cam_ctlr_trans_t *)user_data;
    trans->buffer = ref->buffer;
    trans->buflen = ref->buflen;
    return false;
}

/* CSI callback: frame capture finished */
static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle,
                                         esp_cam_ctlr_trans_t *trans,
                                         void *user_data)
{
    return false;
}

/*
 * Initialize OV5647 sensor via SCCB (I2C) and auto-detect.
 * Based on ESP-IDF v5.3 example_sensor_init pattern.
 */
static esp_err_t sensor_init(void)
{
    /* Create I2C master bus for SCCB */
    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = CAM_SCCB_SDA_IO,
        .scl_io_num = CAM_SCCB_SCL_IO,
        .i2c_port = I2C_NUM_0,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_bus_conf, &i2c_bus_handle),
                        TAG, "I2C bus init failed");

    ESP_LOGI(TAG, "SCCB I2C bus created (SCL=%d, SDA=%d)", CAM_SCCB_SCL_IO, CAM_SCCB_SDA_IO);

    /* Auto-detect camera sensor using esp_cam_sensor component */
    esp_sccb_io_handle_t sccb_io_handle = NULL;
    esp_cam_sensor_config_t cam_config = {
        .sccb_handle = sccb_io_handle,
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };

    esp_cam_sensor_device_t *cam = NULL;
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
        sccb_i2c_config_t i2c_config = {
            .scl_speed_hz = CAM_SCCB_FREQ,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        ESP_ERROR_CHECK(sccb_new_i2c_io(i2c_bus_handle, &i2c_config, &cam_config.sccb_handle));

        cam = (*(p->detect))(&cam_config);
        if (cam) {
            if (p->port != ESP_CAM_SENSOR_MIPI_CSI) {
                ESP_LOGE(TAG, "Detected sensor with wrong interface (expected MIPI-CSI)");
                return ESP_ERR_NOT_SUPPORTED;
            }
            break;
        }
        ESP_ERROR_CHECK(esp_sccb_del_i2c_io(cam_config.sccb_handle));
    }

    if (!cam) {
        ESP_LOGE(TAG, "No camera sensor detected! Check ribbon cable connection.");
        return ESP_ERR_NOT_FOUND;
    }

    /* List supported formats */
    esp_cam_sensor_format_array_t fmt_array = {0};
    esp_cam_sensor_query_format(cam, &fmt_array);
    const esp_cam_sensor_format_t *formats = fmt_array.format_array;
    for (int i = 0; i < fmt_array.count; i++) {
        ESP_LOGI(TAG, "  Sensor format[%d]: %s", i, formats[i].name);
    }

    /* Select our target format */
    esp_cam_sensor_format_t *target_fmt = NULL;
    for (int i = 0; i < fmt_array.count; i++) {
        if (!strcmp(formats[i].name, CAM_FORMAT)) {
            target_fmt = (esp_cam_sensor_format_t *)&formats[i];
            break;
        }
    }

    if (!target_fmt) {
        ESP_LOGE(TAG, "Camera format '%s' not supported by sensor", CAM_FORMAT);
        ESP_LOGW(TAG, "Using first available format: %s", formats[0].name);
        target_fmt = (esp_cam_sensor_format_t *)&formats[0];
    }

    ESP_RETURN_ON_ERROR(esp_cam_sensor_set_format(cam, target_fmt),
                        TAG, "Set camera format failed");
    ESP_LOGI(TAG, "Camera format: %s", target_fmt->name);

    /* Start sensor streaming */
    int enable = 1;
    ESP_RETURN_ON_ERROR(esp_cam_sensor_ioctl(cam, ESP_CAM_SENSOR_IOC_S_STREAM, &enable),
                        TAG, "Start sensor stream failed");

    ESP_LOGI(TAG, "Camera sensor initialized and streaming");
    return ESP_OK;
}
#endif /* HAS_CAMERA_PIPELINE */

/* ========== Camera + JPEG Initialization ========== */

esp_err_t camera_h264_init(void)
{
    esp_err_t ret;

    if (s_cam.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing MIPI-CSI camera pipeline");
    ESP_LOGI(TAG, "Target: %dx%d, format: %s", CAM_WIDTH, CAM_HEIGHT, CAM_FORMAT);

    /* Allocate JPEG output double buffers in PSRAM */
    for (int i = 0; i < NUM_JPEG_BUFS; i++) {
        s_cam.jpeg_buf[i] = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
        if (!s_cam.jpeg_buf[i]) {
            ESP_LOGE(TAG, "Failed to allocate JPEG buffer %d", i);
            return ESP_ERR_NO_MEM;
        }
    }

    s_cam.frame_ready = xSemaphoreCreateBinary();
    s_cam.jpeg_mutex = xSemaphoreCreateMutex();

    /* Allocate raw frame buffer (RGB565: 2 bytes per pixel) */
    s_cam.raw_buf_size = CAM_WIDTH * CAM_HEIGHT * 2;

#if HAS_CAMERA_PIPELINE
    /*
     * === Step 1: LDO for MIPI PHY ===
     */
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = MIPI_LDO_CHAN_ID,
        .voltage_mv = MIPI_LDO_VOLTAGE_MV,
    };
    ret = esp_ldo_acquire_channel(&ldo_config, &ldo_mipi_phy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LDO init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "MIPI PHY LDO enabled (channel %d, %dmV)", MIPI_LDO_CHAN_ID, MIPI_LDO_VOLTAGE_MV);

    /*
     * === Step 2: Camera sensor init (SCCB/I2C + OV5647 auto-detect) ===
     */
    ret = sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera sensor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * === Step 3: Allocate frame buffer with cache alignment ===
     */
    s_cam.raw_buf = heap_caps_aligned_calloc(64, 1, s_cam.raw_buf_size,
                                              MALLOC_CAP_SPIRAM);
    if (!s_cam.raw_buf) {
        ESP_LOGE(TAG, "Failed to allocate raw frame buffer (%u bytes)",
                 (unsigned)s_cam.raw_buf_size);
        return ESP_ERR_NO_MEM;
    }

    esp_cam_ctlr_trans_t cam_trans = {
        .buffer = s_cam.raw_buf,
        .buflen = s_cam.raw_buf_size,
    };

    /*
     * === Step 4: CSI controller init ===
     */
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
        .lane_bit_rate_mbps = CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
    };

    ret = esp_cam_new_csi_ctlr(&csi_config, &s_cam.cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register CSI event callbacks */
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_register_event_callbacks(s_cam.cam_handle, &cbs, &cam_trans),
                        TAG, "CSI callback registration failed");

    ESP_RETURN_ON_ERROR(esp_cam_ctlr_enable(s_cam.cam_handle),
                        TAG, "CSI enable failed");

    ESP_LOGI(TAG, "MIPI-CSI controller initialized (2-lane, %dx%d)", CAM_WIDTH, CAM_HEIGHT);

    /*
     * === Step 5: ISP pipeline (RAW8 → RGB565) ===
     */
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
    };
    ret = esp_isp_new_processor(&isp_config, &isp_proc);
    if (ret == ESP_OK) {
        esp_isp_enable(isp_proc);
        ESP_LOGI(TAG, "ISP pipeline enabled (RAW8 → RGB565)");
    } else {
        ESP_LOGW(TAG, "ISP init failed: %s (continuing without ISP)", esp_err_to_name(ret));
    }

    /*
     * === Step 6: Start CSI capture ===
     */
    ret = esp_cam_ctlr_start(s_cam.cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "CSI capture started");

#else
    ESP_LOGW(TAG, "MIPI-CSI driver headers not available - camera in stub mode");
    ESP_LOGW(TAG, "Required: esp_cam_ctlr_csi.h, driver/isp.h (ESP-IDF v5.3+)");

    /* Allocate raw buffer anyway for stub mode */
    s_cam.raw_buf = heap_caps_malloc(s_cam.raw_buf_size, MALLOC_CAP_SPIRAM);
    if (!s_cam.raw_buf) {
        return ESP_ERR_NO_MEM;
    }
#endif /* HAS_CAMERA_PIPELINE */

#if HAS_HW_JPEG
    /*
     * === Step 7: Hardware JPEG encoder init ===
     */
    jpeg_encode_engine_cfg_t enc_cfg = {
        .timeout_ms = 100,
    };
    ret = jpeg_new_encoder_engine(&enc_cfg, &s_cam.jpeg_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HW JPEG encoder init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Hardware JPEG encoder initialized");
#else
    ESP_LOGW(TAG, "HW JPEG encoder not available (driver/jpeg_encode.h missing)");
#endif

    /* Start the capture + encode task */
    xTaskCreatePinnedToCore(camera_capture_task, "cam_task", 8192, NULL, 5,
                            &s_cam.cam_task_handle, 1);

    s_cam.initialized = true;
    s_cam.stats_start_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Camera pipeline initialized successfully");
    return ESP_OK;
}

/* ========== Camera Capture + JPEG Encode Task ========== */

static void camera_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Capture task started");

#if HAS_CAMERA_PIPELINE
    esp_cam_ctlr_trans_t trans = {
        .buffer = s_cam.raw_buf,
        .buflen = s_cam.raw_buf_size,
    };

    while (1) {
        /* Block until a frame is captured from MIPI-CSI */
        esp_err_t ret = esp_cam_ctlr_receive(s_cam.cam_handle, &trans,
                                              pdMS_TO_TICKS(2000));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Frame receive timeout");
            continue;
        }

        /* Sync cache: DMA wrote to PSRAM, CPU needs to read it */
        esp_cache_msync(s_cam.raw_buf, s_cam.raw_buf_size,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);

        int wr_idx = s_cam.jpeg_write_idx;

#if HAS_HW_JPEG
        /* Encode RGB565 frame to JPEG using hardware encoder */
        jpeg_encode_cfg_t jpeg_cfg = {
            .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
            .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
            .image_quality = JPEG_QUALITY,
            .width = CAM_WIDTH,
            .height = CAM_HEIGHT,
        };

        uint32_t jpg_size = 0;
        ret = jpeg_encoder_process(s_cam.jpeg_handle, &jpeg_cfg,
                                   s_cam.raw_buf, s_cam.raw_buf_size,
                                   s_cam.jpeg_buf[wr_idx], JPEG_BUF_SIZE,
                                   &jpg_size);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(ret));
            continue;
        }
#else
        /* No HW JPEG: can't encode, skip */
        uint32_t jpg_size = 0;
        continue;
#endif

        /* Update JPEG buffer */
        if (xSemaphoreTake(s_cam.jpeg_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_cam.jpeg_size[wr_idx] = jpg_size;
            s_cam.jpeg_write_idx = (wr_idx + 1) % NUM_JPEG_BUFS;
            s_cam.jpeg_read_idx = wr_idx;  /* Point reader to latest frame */
            xSemaphoreGive(s_cam.jpeg_mutex);
        }

        /* Signal HTTP handler that a frame is ready */
        xSemaphoreGive(s_cam.frame_ready);

        /* Update FPS stats */
        s_cam.frame_count++;
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - s_cam.stats_start_time;
        if (elapsed > 1000000) {
            s_cam.fps = (float)s_cam.frame_count * 1000000.0f / (float)elapsed;
            s_cam.frame_count = 0;
            s_cam.stats_start_time = now;
        }
    }

#else
    /* Stub mode: no real camera, just sleep */
    while (1) {
        ESP_LOGW(TAG, "Camera stub mode - no frames available");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
#endif
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

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    ESP_LOGI(TAG, "MJPEG stream client connected");

    while (true) {
        /* Wait for a new frame */
        if (xSemaphoreTake(s_cam.frame_ready, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGW(TAG, "Stream: no frame available (timeout)");
            continue;
        }

        if (xSemaphoreTake(s_cam.jpeg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int rd_idx = s_cam.jpeg_read_idx;
            size_t jpg_size = s_cam.jpeg_size[rd_idx];

            if (jpg_size > 0) {
                /* Send MJPEG boundary + JPEG frame */
                res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
                if (res == ESP_OK) {
                    size_t hlen = snprintf(part_buf, sizeof(part_buf),
                        "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                        (unsigned)jpg_size);
                    res = httpd_resp_send_chunk(req, part_buf, hlen);
                }
                if (res == ESP_OK) {
                    res = httpd_resp_send_chunk(req,
                        (const char *)s_cam.jpeg_buf[rd_idx], jpg_size);
                }
            }
            xSemaphoreGive(s_cam.jpeg_mutex);
        }

        if (res != ESP_OK) break;
    }

    ESP_LOGI(TAG, "MJPEG stream client disconnected");
    return res;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"initialized\":%s,\"resolution\":\"%dx%d\",\"fps\":%.1f,"
        "\"encoder\":\"jpeg_hw\",\"transport\":\"halow\","
        "\"pipeline\":\"csi_isp_jpeg\"}",
        s_cam.initialized ? "true" : "false",
        CAM_WIDTH, CAM_HEIGHT,
        s_cam.fps);

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
        httpd_register_uri_handler(server, &uri_status);
        ESP_LOGI(TAG, "Camera HTTP server started");
        ESP_LOGI(TAG, "  MJPEG stream: http://<ip>/");
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
