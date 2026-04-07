/*
 * MIPI-CSI Camera + HW JPEG/H.264 Encoder for ESP32-P4
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pipelines:
 *   H.264: OV5647 → MIPI-CSI → ISP (RAW8→YUV420) → HW H.264 → UDP RTP :5600
 *   MJPEG: (disabled by default, set ENABLE_MJPEG=1 to enable)
 *
 * References:
 *   - ESP-IDF examples/peripherals/camera/camera_dsi
 *   - esp_cam_sensor component (OV5647 driver)
 *   - espressif/esp_h264 component (HW H.264 encoder)
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_netif.h"

#include "camera_h264.h"
#include "thermal_camera.h"
#include "gcs_bridge.h"
#include "mm_app_common.h"
#include "../common/i2c_sync.h"

static const char *TAG = "camera_h264";

/*
 * Camera Configuration
 *
 * Waveshare ESP32-P4-WIFI6 board:
 *   - MIPI-CSI: dedicated differential pairs (not GPIO)
 *   - SCCB (I2C): GPIO8 (SCL), GPIO7 (SDA)
 *   - LDO channel 3 at 2500mV for MIPI PHY
 */

/* I2C / SCCB pins for camera sensor */
#define CAM_SCCB_SCL_IO     8
#define CAM_SCCB_SDA_IO     7
#define CAM_SCCB_FREQ       100000

/* LDO for MIPI PHY */
#define MIPI_LDO_CHAN_ID     3
#define MIPI_LDO_VOLTAGE_MV  2500

/* Camera format */
#define CAM_FORMAT          "MIPI_2lane_24Minput_RAW8_800x640_50fps"
#define CAM_WIDTH           800
#define CAM_HEIGHT          640

/* MIPI CSI lane bitrate */
#define CSI_LANE_BITRATE_MBPS  200

/* JPEG quality (1-100, higher = better quality, larger file) */
#define JPEG_QUALITY        30           /* Low quality for HaLow bandwidth */
#define JPEG_BUF_SIZE       (100 * 1024) /* 100KB for low-quality 800x640 */

/* H.264 encoder settings */
#define H264_GOP            10           /* Reduced from 60 to 10 for faster recovery on HaLow loss */
#define H264_FPS            5            /* Encode at 5fps for HaLow bandwidth */
#define H264_QP_MIN         32
#define H264_QP_MAX         48           /* Aggressive compression for HaLow */
#define H264_BITRATE        150000       /* 150 Kbps target (MCS2/2MHz ~400Kbps usable) */
#define H264_BUF_SIZE       (100 * 1024) /* 100KB per encoded frame */

/* H.264 delivered via UDP RTP + HTTP/TCP backup */
#define RTP_PORT            5600
#define RTP_PKT_MAX_SIZE    1200         /* Small packets for HaLow stability */
#define RTP_HEADER_SIZE     12
#define RTP_PAYLOAD_TYPE    96           /* Dynamic PT for H.264 */
#define RTP_PACING_MS       5            /* Delay between packets to avoid TX queue overflow */
#define RTP_DEFAULT_DEST_IP "192.168.1.143"  /* Default GCS IP, updated by MAVLink heartbeat */

/* Stream frame rate limit (camera captures at 50fps, we stream fewer) */
#define STREAM_TARGET_FPS   5

/* Set to 1 to enable MJPEG HTTP streaming (requires YUV422 ISP output — conflicts with H.264 YUV420) */
#define ENABLE_MJPEG        0

/* Double buffer for raw frames and encoded output */
#define NUM_BUFS            2

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

#if __has_include("esp_h264_enc_single_hw.h")
#define HAS_HW_H264 1
#else
#define HAS_HW_H264 0
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

#if HAS_HW_H264
#include "esp_h264_types.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_single_hw.h"
#endif

/* Module state */
static struct {
    bool initialized;

    /* JPEG double buffer (MJPEG stream) */
    uint8_t *jpeg_buf[NUM_BUFS];
    size_t jpeg_size[NUM_BUFS];
    volatile int jpeg_write_idx;
    volatile int jpeg_read_idx;
    SemaphoreHandle_t frame_ready;
    SemaphoreHandle_t jpeg_mutex;

    /* H.264 encode output buffer */
    uint8_t *h264_buf;
    size_t h264_size;

    /* H.264 frame signaling for HTTP streaming */
    SemaphoreHandle_t h264_ready;
    SemaphoreHandle_t h264_mutex;
    uint8_t *h264_send_buf;       /* Copy for HTTP client */
    size_t h264_send_size;
    volatile int h264_clients;    /* HTTP H.264 viewer count */

    /* Raw frame double buffers (YUV420 from ISP) */
    uint8_t *raw_buf[NUM_BUFS];
    size_t raw_buf_size;

    /* ISR → task signaling for frame capture */
    SemaphoreHandle_t frame_captured;
    volatile int captured_buf_idx;

    /* Frame statistics */
    volatile float fps;
    uint32_t frame_count;
    int64_t stats_start_time;

    /* On-demand streaming */
    volatile int mjpeg_clients;

    /* Camera task handle */
    TaskHandle_t cam_task_handle;

    /* RTP state */
    int rtp_sock;
    struct sockaddr_in rtp_dest_addr;
    uint16_t rtp_seq;
    uint32_t rtp_ts;
    uint32_t rtp_ssrc;

    /* RTP diagnostics */
    uint32_t rtp_pkts_sent;
    uint32_t rtp_pkts_dropped;
    uint32_t rtp_frames_sent;
    uint32_t rtp_frames_skipped;    /* Frames skipped due to congestion or no GCS */
    uint32_t rtp_backoff_count;

    /* Adaptive streaming state */
    uint32_t last_pause_count;      /* TX pause count at last check */
    int skip_frames;                /* Number of frames to skip (congestion backoff) */
    bool need_idr;                  /* Request IDR after skip */

#if HAS_CAMERA_PIPELINE
    esp_cam_ctlr_handle_t cam_handle;
#endif
#if HAS_HW_JPEG
    jpeg_encoder_handle_t jpeg_handle;
#endif
#if HAS_HW_H264
    esp_h264_enc_handle_t h264_handle;
#endif
} s_cam = {0};

/* Forward declarations */
static void camera_capture_task(void *arg);
static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t status_handler(httpd_req_t *req);

/* ========== RTP / UDP Video Sender ========== */

static void rtp_header_serialize(uint8_t *buf, uint16_t seq, uint32_t ts, uint32_t ssrc, bool marker)
{
    buf[0] = 0x80; // V=2, P=0, X=0, CC=0
    buf[1] = (marker ? 0x80 : 0x00) | (RTP_PAYLOAD_TYPE & 0x7F);
    buf[2] = (seq >> 8) & 0xFF;
    buf[3] = seq & 0xFF;
    buf[4] = (ts >> 24) & 0xFF;
    buf[5] = (ts >> 16) & 0xFF;
    buf[6] = (ts >> 8) & 0xFF;
    buf[7] = ts & 0xFF;
    buf[8] = (ssrc >> 24) & 0xFF;
    buf[9] = (ssrc >> 16) & 0xFF;
    buf[10] = (ssrc >> 8) & 0xFF;
    buf[11] = ssrc & 0xFF;
}

static void rtp_send_packet(const uint8_t *data, size_t len, bool marker)
{
    if (s_cam.rtp_sock < 0) return;

    /* Check HaLow TX flow control — skip if pool is saturated */
    if (app_wlan_tx_is_paused()) {
        s_cam.rtp_pkts_dropped++;
        s_cam.rtp_backoff_count++;
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    uint8_t pkt[RTP_PKT_MAX_SIZE + RTP_HEADER_SIZE + 2];
    rtp_header_serialize(pkt, s_cam.rtp_seq++, s_cam.rtp_ts, s_cam.rtp_ssrc, marker);
    memcpy(pkt + RTP_HEADER_SIZE, data, len);

    int ret = sendto(s_cam.rtp_sock, pkt, len + RTP_HEADER_SIZE, 0,
                     (struct sockaddr *)&s_cam.rtp_dest_addr, sizeof(s_cam.rtp_dest_addr));
    if (ret < 0) {
        s_cam.rtp_pkts_dropped++;
        if (errno == ENOMEM || errno == EAGAIN || errno == EWOULDBLOCK) {
            s_cam.rtp_backoff_count++;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        return;
    }

    s_cam.rtp_pkts_sent++;

    /* Pacing: delay between packets to prevent Morse Micro TX queue overflow */
    vTaskDelay(pdMS_TO_TICKS(RTP_PACING_MS));
}

static void rtp_send_frame(const uint8_t *buf, size_t len)
{
    if (s_cam.rtp_sock < 0) return;

    /* On-demand: only send RTP when GCS is actively connected */
    if (!gcs_bridge_is_active()) {
        s_cam.rtp_frames_skipped++;
        return;
    }

    /* Adaptive frame skip: back off when TX pool is congested */
    if (s_cam.skip_frames > 0) {
        s_cam.skip_frames--;
        s_cam.rtp_frames_skipped++;
        return;
    }

    /* Check if TX pool congestion increased — trigger adaptive backoff */
    uint32_t cur_pause = app_wlan_tx_pause_count();
    if (cur_pause > s_cam.last_pause_count) {
        uint32_t delta = cur_pause - s_cam.last_pause_count;
        /* Skip 2 frames per new pause event (gives HaLow time to drain) */
        s_cam.skip_frames = delta * 2;
        s_cam.need_idr = true;
        ESP_LOGW(TAG, "[rtp] congestion: %lu new pauses, skipping %d frames",
                 (unsigned long)delta, s_cam.skip_frames);
    }
    s_cam.last_pause_count = cur_pause;

    /* Update RTP destination from GCS bridge if MAVLink heartbeat detected */
    uint32_t gcs_ip = gcs_bridge_get_ip();
    if (gcs_ip != 0 && gcs_ip != s_cam.rtp_dest_addr.sin_addr.s_addr) {
        s_cam.rtp_dest_addr.sin_addr.s_addr = gcs_ip;
        ESP_LOGI(TAG, "RTP destination updated to GCS: %s",
                 inet_ntoa(s_cam.rtp_dest_addr.sin_addr));
    }

    s_cam.rtp_frames_sent++;

    /* Debug: log first few frames to verify Annex-B format */
    if (s_cam.rtp_frames_sent <= 3) {
        ESP_LOGI(TAG, "[rtp-debug] frame #%lu len=%u bytes[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x",
                 (unsigned long)s_cam.rtp_frames_sent, (unsigned)len,
                 len > 0 ? buf[0] : 0, len > 1 ? buf[1] : 0,
                 len > 2 ? buf[2] : 0, len > 3 ? buf[3] : 0,
                 len > 4 ? buf[4] : 0, len > 5 ? buf[5] : 0,
                 len > 6 ? buf[6] : 0, len > 7 ? buf[7] : 0);
    }

    const uint8_t *p = buf;
    const uint8_t *end = buf + len;

    /* Standard RTP timestamp: 90kHz clock for H.264 */
    s_cam.rtp_ts += (90000 / H264_FPS);

    while (p < end) {
        /* Find Annex-B start code (00 00 00 01 or 00 00 01) */
        if (p + 4 < end && p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1) {
            p += 4;
        } else if (p + 3 < end && p[0] == 0 && p[1] == 0 && p[2] == 1) {
            p += 3;
        } else {
            p++;
            continue;
        }

        /* Found NAL unit start. Find next start code to get NAL length. */
        const uint8_t *nal_start = p;
        const uint8_t *next = p;
        while (next + 3 < end) {
            if (next[0] == 0 && next[1] == 0 && (next[2] == 1 || (next[2] == 0 && next[3] == 1))) {
                break;
            }
            next++;
        }
        if (next + 3 >= end) next = end;
        size_t nal_len = next - nal_start;

        if (nal_len == 0) continue;

        uint8_t nal_type = nal_start[0] & 0x1F;

        if (nal_len <= RTP_PKT_MAX_SIZE) {
            /* Single NAL unit packet */
            rtp_send_packet(nal_start, nal_len, (next == end));
        } else {
            /* FU-A Fragmentation (RFC 6184) */
            uint8_t nal_header = nal_start[0];
            const uint8_t *payload = nal_start + 1;
            size_t payload_len = nal_len - 1;

            while (payload_len > 0) {
                /* Check TX flow control for each fragment.
                 * If paused mid-NAL, skip remaining fragments AND remaining NALs
                 * to avoid sending partial FU-A sequences that corrupt the decoder. */
                if (app_wlan_tx_is_paused()) {
                    s_cam.rtp_pkts_dropped++;
                    s_cam.rtp_backoff_count++;
                    s_cam.skip_frames = 2; /* Skip next 2 frames to let TX drain */
                    s_cam.need_idr = true;
                    vTaskDelay(pdMS_TO_TICKS(20));
                    return; /* Abort entire frame — partial FU-A is undecodable */
                }

                size_t chunk = (payload_len > (RTP_PKT_MAX_SIZE - 2)) ? (RTP_PKT_MAX_SIZE - 2) : payload_len;
                bool first = (payload == nal_start + 1);
                bool last = (chunk == payload_len);

                uint8_t fu_indicator = (nal_header & 0xE0) | 28; // FU-A type 28
                uint8_t fu_header = (first ? 0x80 : 0) | (last ? 0x40 : 0) | nal_type;

                uint8_t pkt[RTP_PKT_MAX_SIZE + RTP_HEADER_SIZE + 2];
                rtp_header_serialize(pkt, s_cam.rtp_seq++, s_cam.rtp_ts, s_cam.rtp_ssrc, (last && next == end));
                pkt[RTP_HEADER_SIZE] = fu_indicator;
                pkt[RTP_HEADER_SIZE + 1] = fu_header;
                memcpy(pkt + RTP_HEADER_SIZE + 2, payload, chunk);

                int ret = sendto(s_cam.rtp_sock, pkt, chunk + RTP_HEADER_SIZE + 2, 0,
                                 (struct sockaddr *)&s_cam.rtp_dest_addr, sizeof(s_cam.rtp_dest_addr));
                if (ret < 0) {
                    s_cam.rtp_pkts_dropped++;
                    if (errno == ENOMEM || errno == EAGAIN || errno == EWOULDBLOCK) {
                        s_cam.rtp_backoff_count++;
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                } else {
                    s_cam.rtp_pkts_sent++;
                }

                vTaskDelay(pdMS_TO_TICKS(RTP_PACING_MS));

                payload += chunk;
                payload_len -= chunk;
            }
        }
        p = next;
    }
}

#if HAS_CAMERA_PIPELINE

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans,
                                        void *user_data)
{
    int next_idx = (s_cam.captured_buf_idx + 1) % NUM_BUFS;
    trans->buffer = s_cam.raw_buf[next_idx];
    trans->buflen = s_cam.raw_buf_size;
    return false;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle,
                                         esp_cam_ctlr_trans_t *trans,
                                         void *user_data)
{
    for (int i = 0; i < NUM_BUFS; i++) {
        if (trans->buffer == s_cam.raw_buf[i]) {
            s_cam.captured_buf_idx = i;
            break;
        }
    }

    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_cam.frame_captured, &higher_prio_woken);
    return higher_prio_woken == pdTRUE;
}

static esp_err_t sensor_init(void)
{
    /* Try to get existing I2C bus first (sensor_agent may have already created it),
     * fall back to creating a new one if not yet initialized */
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    esp_err_t bus_err = i2c_master_get_bus_handle(I2C_NUM_0, &i2c_bus_handle);
    if (bus_err != ESP_OK || i2c_bus_handle == NULL) {
        i2c_master_bus_config_t i2c_bus_conf = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .sda_io_num = CAM_SCCB_SDA_IO,
            .scl_io_num = CAM_SCCB_SCL_IO,
            .i2c_port = I2C_NUM_0,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_bus_conf, &i2c_bus_handle),
                            TAG, "I2C bus init failed");
        ESP_LOGI(TAG, "SCCB I2C bus created (SCL=%d, SDA=%d)", CAM_SCCB_SCL_IO, CAM_SCCB_SDA_IO);
    } else {
        ESP_LOGI(TAG, "Reusing existing I2C bus for camera SCCB");
    }

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

    esp_cam_sensor_format_array_t fmt_array = {0};
    esp_cam_sensor_query_format(cam, &fmt_array);
    const esp_cam_sensor_format_t *formats = fmt_array.format_array;
    for (int i = 0; i < fmt_array.count; i++) {
        ESP_LOGI(TAG, "  Sensor format[%d]: %s", i, formats[i].name);
    }

    esp_cam_sensor_format_t *target_fmt = NULL;
    for (int i = 0; i < fmt_array.count; i++) {
        if (!strcmp(formats[i].name, CAM_FORMAT)) {
            target_fmt = (esp_cam_sensor_format_t *)&formats[i];
            break;
        }
    }

    if (!target_fmt) {
        ESP_LOGE(TAG, "Camera format '%s' not supported", CAM_FORMAT);
        target_fmt = (esp_cam_sensor_format_t *)&formats[0];
    }

    ESP_RETURN_ON_ERROR(esp_cam_sensor_set_format(cam, target_fmt),
                        TAG, "Set camera format failed");
    ESP_LOGI(TAG, "Camera format: %s", target_fmt->name);

    int enable = 1;
    ESP_RETURN_ON_ERROR(esp_cam_sensor_ioctl(cam, ESP_CAM_SENSOR_IOC_S_STREAM, &enable),
                        TAG, "Start sensor stream failed");

    ESP_LOGI(TAG, "Camera sensor initialized and streaming");
    return ESP_OK;
}
#endif /* HAS_CAMERA_PIPELINE */

/* ========== Initialization ========== */

esp_err_t camera_h264_init(void)
{
    esp_err_t ret;

    if (s_cam.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing MIPI-CSI camera pipeline");
    ESP_LOGI(TAG, "Target: %dx%d, format: %s", CAM_WIDTH, CAM_HEIGHT, CAM_FORMAT);

    s_cam.frame_ready = xSemaphoreCreateBinary();
    s_cam.jpeg_mutex = xSemaphoreCreateMutex();
    s_cam.frame_captured = xSemaphoreCreateBinary();
    s_cam.h264_ready = xSemaphoreCreateBinary();
    s_cam.h264_mutex = xSemaphoreCreateMutex();

    /* Allocate buffers */
    s_cam.raw_buf_size = CAM_WIDTH * CAM_HEIGHT * 3 / 2;  /* YUV420 = 1.5 bytes/pixel */
    s_cam.raw_buf_size = (s_cam.raw_buf_size + 63) & ~63;  /* Cache line align */

    for (int i = 0; i < NUM_BUFS; i++) {
        /* Raw buffers: 64-byte aligned for CSI DMA */
        s_cam.raw_buf[i] = heap_caps_aligned_calloc(64, 1, s_cam.raw_buf_size,
                                                     MALLOC_CAP_SPIRAM);
        if (!s_cam.raw_buf[i]) {
            ESP_LOGE(TAG, "Failed to allocate raw frame buffer %d", i);
            return ESP_ERR_NO_MEM;
        }

#if ENABLE_MJPEG
        /* JPEG output buffers */
#if HAS_HW_JPEG
        size_t jpg_alloc_size = 0;
        jpeg_encode_memory_alloc_cfg_t jpg_mem_cfg = {
            .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
        };
        s_cam.jpeg_buf[i] = (uint8_t *)jpeg_alloc_encoder_mem(
            JPEG_BUF_SIZE, &jpg_mem_cfg, &jpg_alloc_size);
#else
        s_cam.jpeg_buf[i] = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
#endif
        if (!s_cam.jpeg_buf[i]) {
            ESP_LOGE(TAG, "Failed to allocate JPEG buffer %d", i);
            return ESP_ERR_NO_MEM;
        }
#endif /* ENABLE_MJPEG */
    }

    /* H.264 output buffer: 64-byte aligned for HW encoder DMA */
    s_cam.h264_buf = heap_caps_aligned_calloc(64, 1, H264_BUF_SIZE, MALLOC_CAP_SPIRAM);
    s_cam.h264_send_buf = heap_caps_malloc(H264_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_cam.h264_buf || !s_cam.h264_send_buf) {
        ESP_LOGE(TAG, "Failed to allocate H.264 buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Buffers allocated: %d x raw=%uKB + h264=%uKB",
             NUM_BUFS, (unsigned)(s_cam.raw_buf_size/1024),
             (unsigned)(H264_BUF_SIZE/1024));

#if HAS_CAMERA_PIPELINE
    /* LDO for MIPI PHY */
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

    /* Wait for sensor_agent I2C init to complete before using SCCB */
    ESP_LOGI(TAG, "Waiting for sensor I2C init to complete...");
    i2c_sync_wait_sensors();
    ESP_LOGI(TAG, "Sensor init done, starting camera SCCB");

    /* Camera sensor init (SCCB/I2C) */
    ret = sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera sensor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* CSI controller */
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
        .lane_bit_rate_mbps = CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_YUV420,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
    };

    ret = esp_cam_new_csi_ctlr(&csi_config, &s_cam.cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_register_event_callbacks(s_cam.cam_handle, &cbs, NULL),
                        TAG, "CSI callback registration failed");
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_enable(s_cam.cam_handle), TAG, "CSI enable failed");
    ESP_LOGI(TAG, "MIPI-CSI controller initialized (2-lane, %dx%d)", CAM_WIDTH, CAM_HEIGHT);

    /* ISP pipeline */
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_YUV420,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
    };
    ret = esp_isp_new_processor(&isp_config, &isp_proc);
    if (ret == ESP_OK) {
        esp_isp_enable(isp_proc);
        ESP_LOGI(TAG, "ISP pipeline enabled (RAW8 → YUV420)");
    }

    /* Start CSI capture */
    s_cam.captured_buf_idx = 0;
    ret = esp_cam_ctlr_start(s_cam.cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "CSI capture started");
#else
    ESP_LOGW(TAG, "MIPI-CSI not available - stub mode");
#endif

#if ENABLE_MJPEG && HAS_HW_JPEG
    /* HW JPEG encoder (only when MJPEG streaming is enabled) */
    jpeg_encode_engine_cfg_t jpeg_enc_cfg = { .timeout_ms = 100 };
    ret = jpeg_new_encoder_engine(&jpeg_enc_cfg, &s_cam.jpeg_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HW JPEG encoder initialized");
    } else {
        ESP_LOGW(TAG, "HW JPEG init failed: %s", esp_err_to_name(ret));
    }
#endif

    /* HW H.264 encoder */
#if HAS_HW_H264
    esp_h264_enc_cfg_hw_t h264_cfg = {
        .gop = H264_GOP,
        .fps = H264_FPS,
        .res = { .width = CAM_WIDTH, .height = CAM_HEIGHT },
        .rc = {
            .bitrate = H264_BITRATE,
            .qp_min = H264_QP_MIN,
            .qp_max = H264_QP_MAX,
        },
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
    };
    esp_h264_err_t h264_ret = esp_h264_enc_hw_new(&h264_cfg, &s_cam.h264_handle);
    if (h264_ret == ESP_H264_ERR_OK) {
        h264_ret = esp_h264_enc_open(s_cam.h264_handle);
        if (h264_ret == ESP_H264_ERR_OK) {
            ESP_LOGI(TAG, "HW H.264 encoder initialized (%dx%d, GOP=%d, %d Kbps)",
                     CAM_WIDTH, CAM_HEIGHT, H264_GOP, H264_BITRATE / 1000);
        } else {
            ESP_LOGW(TAG, "H.264 enc open failed: %d", h264_ret);
            s_cam.h264_handle = NULL;
        }
    } else {
        ESP_LOGW(TAG, "H.264 enc create failed: %d", h264_ret);
        s_cam.h264_handle = NULL;
    }
#else
    ESP_LOGW(TAG, "HW H.264 encoder not available (esp_h264_enc_single_hw.h missing)");
    ESP_LOGW(TAG, "Add espressif/esp_h264 to idf_component.yml");
#endif

    /* H.264 delivered via UDP RTP + HTTP/TCP backup */
    s_cam.rtp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_cam.rtp_sock >= 0) {
        memset(&s_cam.rtp_dest_addr, 0, sizeof(s_cam.rtp_dest_addr));
        s_cam.rtp_dest_addr.sin_family = AF_INET;
        s_cam.rtp_dest_addr.sin_port = htons(RTP_PORT);
        inet_aton(RTP_DEFAULT_DEST_IP, &s_cam.rtp_dest_addr.sin_addr);

        s_cam.rtp_seq = (uint16_t)esp_random();
        s_cam.rtp_ts = esp_random();
        s_cam.rtp_ssrc = esp_random();
        ESP_LOGI(TAG, "RTP UDP socket initialized → %s:%d (auto-updates from MAVLink GCS)",
                 RTP_DEFAULT_DEST_IP, RTP_PORT);
    } else {
        ESP_LOGE(TAG, "Failed to create RTP UDP socket");
    }

    /* Start capture task */
    xTaskCreatePinnedToCore(camera_capture_task, "cam_task", 8192, NULL, 5,
                            &s_cam.cam_task_handle, 1);

    s_cam.initialized = true;
    s_cam.stats_start_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Camera pipeline initialized successfully");
    return ESP_OK;
}

/* ========== Camera Capture + Encode Task ========== */

static void camera_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Capture task started");

#if HAS_CAMERA_PIPELINE
    int64_t last_encode_us = 0;
    const int64_t frame_interval_us = 1000000 / STREAM_TARGET_FPS;
    int64_t last_log_us = 0;

    /* Accumulated timing stats (reset every log interval) */
    uint32_t stat_frames = 0;
    int64_t stat_wait_us = 0, stat_h264_us = 0;
    size_t stat_h264_bytes = 0;

    while (1) {
        int64_t t0 = esp_timer_get_time();

        if (xSemaphoreTake(s_cam.frame_captured, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "Frame capture timeout - check camera ribbon cable");
            continue;
        }

        int64_t t1 = esp_timer_get_time();

        /* Time-based frame skip (consistent interval regardless of encode time) */
        if ((t1 - last_encode_us) < frame_interval_us) {
            continue;
        }
        last_encode_us = t1;

        int buf_idx = s_cam.captured_buf_idx;
        uint8_t *frame_data = s_cam.raw_buf[buf_idx];

        /* Sync cache: DMA wrote to PSRAM, CPU needs to read it */
        esp_cache_msync(frame_data, s_cam.raw_buf_size,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);

        /* === JPEG encode (for MJPEG HTTP stream, disabled by default) === */
        int64_t t2 = t1;
#if ENABLE_MJPEG && HAS_HW_JPEG
        {
            size_t jpg_size_out = 0;
            int wr_idx = s_cam.jpeg_write_idx;
            jpeg_encode_cfg_t jpeg_cfg = {
                .src_type = JPEG_ENCODE_IN_FORMAT_YUV422,
                .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
                .image_quality = JPEG_QUALITY,
                .width = CAM_WIDTH,
                .height = CAM_HEIGHT,
            };

            uint32_t jpg_size = 0;
            esp_err_t ret = jpeg_encoder_process(s_cam.jpeg_handle, &jpeg_cfg,
                                                  frame_data, s_cam.raw_buf_size,
                                                  s_cam.jpeg_buf[wr_idx], JPEG_BUF_SIZE,
                                                  &jpg_size);
            if (ret == ESP_OK && jpg_size > 0) {
                if (xSemaphoreTake(s_cam.jpeg_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    s_cam.jpeg_size[wr_idx] = jpg_size;
                    s_cam.jpeg_write_idx = (wr_idx + 1) % NUM_BUFS;
                    s_cam.jpeg_read_idx = wr_idx;
                    xSemaphoreGive(s_cam.jpeg_mutex);
                }
                xSemaphoreGive(s_cam.frame_ready);
                jpg_size_out = jpg_size;
            }
        }
        t2 = esp_timer_get_time();
#endif

        /* === H.264 encode → send via UDP RTP === */
        int64_t t4 = t2;
        size_t h264_size_out = 0;
#if HAS_HW_H264
        if (s_cam.h264_handle) {
            /* ISP outputs YUV420 (O_UYY_E_VYY) directly — no conversion needed */

            /* HW H.264 encode */
            esp_h264_enc_in_frame_t in_frame = {
                .raw_data = { .buffer = frame_data },
            };
            in_frame.raw_data.len = CAM_WIDTH * CAM_HEIGHT * 3 / 2;

            esp_h264_enc_out_frame_t out_frame = {
                .raw_data = {
                    .buffer = s_cam.h264_buf,
                    .len = H264_BUF_SIZE,
                },
            };

            esp_h264_err_t h264_ret = esp_h264_enc_process(s_cam.h264_handle,
                                                            &in_frame, &out_frame);
            t4 = esp_timer_get_time();

            if (h264_ret == ESP_H264_ERR_OK && out_frame.length > 0) {
                h264_size_out = out_frame.length;

                /* Sync cache: H.264 encoder wrote to PSRAM, CPU needs to read */
                esp_cache_msync(s_cam.h264_buf,
                                (out_frame.length + 63) & ~63,
                                ESP_CACHE_MSYNC_FLAG_DIR_M2C);

                /* 1. Send via UDP RTP (Primary, Low Latency) */
                rtp_send_frame(s_cam.h264_buf, out_frame.length);

                /* 2. Copy H.264 frame for HTTP client (Backup/Debug) */
                if (xSemaphoreTake(s_cam.h264_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    memcpy(s_cam.h264_send_buf, s_cam.h264_buf, out_frame.length);
                    s_cam.h264_send_size = out_frame.length;
                    xSemaphoreGive(s_cam.h264_mutex);
                    xSemaphoreGive(s_cam.h264_ready);
                }
            }
        }
#endif

        /* Update FPS stats */
        s_cam.frame_count++;
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - s_cam.stats_start_time;
        if (elapsed > 1000000) {
            s_cam.fps = (float)s_cam.frame_count * 1000000.0f / (float)elapsed;
            s_cam.frame_count = 0;
            s_cam.stats_start_time = now;
        }

        /* Timing debug log (every 5 seconds) */
        stat_frames++;
        stat_wait_us += (t1 - t0);
        stat_h264_us += (t4 - t2);
        stat_h264_bytes += h264_size_out;

        if ((now - last_log_us) > 5000000) {
            if (stat_frames > 0) {
                ESP_LOGI(TAG, "[perf] %ld frames: wait=%ldms h264=%ldms total=%ldms | h264=%luKB | http=%d",
                         (long)stat_frames,
                         (long)(stat_wait_us / stat_frames / 1000),
                         (long)(stat_h264_us / stat_frames / 1000),
                         (long)((stat_wait_us + stat_h264_us) / stat_frames / 1000),
                         (unsigned long)(stat_h264_bytes / 1024),
                         s_cam.h264_clients);
            }
            ESP_LOGI(TAG, "[rtp] sent=%lu drop=%lu frames=%lu skip=%lu backoff=%lu | tx_paused=%s pause_cnt=%lu | gcs=%s dest=%s",
                     (unsigned long)s_cam.rtp_pkts_sent,
                     (unsigned long)s_cam.rtp_pkts_dropped,
                     (unsigned long)s_cam.rtp_frames_sent,
                     (unsigned long)s_cam.rtp_frames_skipped,
                     (unsigned long)s_cam.rtp_backoff_count,
                     app_wlan_tx_is_paused() ? "YES" : "no",
                     (unsigned long)app_wlan_tx_pause_count(),
                     gcs_bridge_is_active() ? "active" : "INACTIVE",
                     inet_ntoa(s_cam.rtp_dest_addr.sin_addr));
            stat_frames = 0;
            stat_wait_us = stat_h264_us = 0;
            stat_h264_bytes = 0;
            last_log_us = now;
        }
    }

#else
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
    /* MJPEG disabled (ISP outputs YUV420, HW JPEG encoder only accepts YUV422) */
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req,
        "<html><body style='background:#111;color:#eee;font-family:monospace;text-align:center;padding:40px'>"
        "<h2>ESP32-P4 Helicopter</h2>"
        "<p><a href='/video' style='color:#0af;font-size:20px'>H.264 Video (HTTP backup)</a></p>"
        "<p style='color:#888;font-size:14px'>Primary: UDP RTP on port 5600 (auto-detect GCS IP)</p>"
        "<p><a href='/stream.sdp' style='color:#0af;font-size:16px'>stream.sdp</a> — open in VLC for H.264 RTP</p>"
        "<p><a href='/thermal' style='color:#0af;font-size:20px'>Thermal Camera</a></p>"
        "<p><a href='/status' style='color:#0af;font-size:20px'>System Status</a></p>"
        "</body></html>", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[896];
    snprintf(buf, sizeof(buf),
        "{\"initialized\":%s,\"resolution\":\"%dx%d\",\"fps\":%.1f,"
        "\"jpeg_encoder\":\"%s\",\"h264_encoder\":\"%s\","
        "\"h264_transport\":\"udp_rtp+http_tcp\",\"h264_port\":%d,"
        "\"rtp_dest\":\"%s\","
        "\"rtp_pkts_sent\":%lu,\"rtp_pkts_dropped\":%lu,"
        "\"rtp_frames\":%lu,\"rtp_frames_skipped\":%lu,\"rtp_backoffs\":%lu,"
        "\"tx_paused\":%s,\"tx_pause_count\":%lu,\"gcs_active\":%s,"
        "\"pipeline\":\"csi_isp\","
        "\"vlc\":\"rtp://@:%d\"}",
        s_cam.initialized ? "true" : "false",
        CAM_WIDTH, CAM_HEIGHT, s_cam.fps,
#if HAS_HW_JPEG
        "hw",
#else
        "none",
#endif
#if HAS_HW_H264
        s_cam.h264_handle ? "hw" : "failed",
#else
        "none",
#endif
        RTP_PORT,
        inet_ntoa(s_cam.rtp_dest_addr.sin_addr),
        (unsigned long)s_cam.rtp_pkts_sent,
        (unsigned long)s_cam.rtp_pkts_dropped,
        (unsigned long)s_cam.rtp_frames_sent,
        (unsigned long)s_cam.rtp_frames_skipped,
        (unsigned long)s_cam.rtp_backoff_count,
        app_wlan_tx_is_paused() ? "true" : "false",
        (unsigned long)app_wlan_tx_pause_count(),
        gcs_bridge_is_active() ? "true" : "false",
        RTP_PORT);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, strlen(buf));
}

/* ========== SDP for VLC/ffplay RTP playback ========== */

/*
 * Serve SDP so VLC can open: http://<ip>/stream.sdp
 * or: vlc http://192.168.1.2/stream.sdp
 * or: ffplay -protocol_whitelist file,udp,rtp -i http://192.168.1.2/stream.sdp
 */
static esp_err_t sdp_handler(httpd_req_t *req)
{
    /* Get the client's IP so we can put the correct connection address */
    char sdp[512];
    snprintf(sdp, sizeof(sdp),
        "v=0\r\n"
        "o=- 0 0 IN IP4 0.0.0.0\r\n"
        "s=ESP32-P4 H.264\r\n"
        "c=IN IP4 0.0.0.0\r\n"
        "t=0 0\r\n"
        "m=video %d RTP/AVP %d\r\n"
        "a=rtpmap:%d H264/90000\r\n"
        "a=fmtp:%d packetization-mode=1\r\n"
        "a=framerate:%d\r\n",
        RTP_PORT, RTP_PAYLOAD_TYPE,
        RTP_PAYLOAD_TYPE,
        RTP_PAYLOAD_TYPE,
        H264_FPS);

    httpd_resp_set_type(req, "application/sdp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"stream.sdp\"");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, sdp, strlen(sdp));
}

/* ========== H.264 HTTP Stream (TCP — replaces UDP RTP) ========== */

/*
 * Serve H.264 Annex B bytestream over HTTP chunked transfer.
 * VLC: open http://<ip>/video  (auto-detects H.264)
 * ffplay: ffplay -f h264 http://<ip>/video
 */
static esp_err_t h264_stream_handler(httpd_req_t *req)
{
    if (!s_cam.initialized) {
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "Camera not connected", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Type", "video/h264");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    s_cam.h264_clients++;
    ESP_LOGI(TAG, "H.264 HTTP client connected (%d active)", s_cam.h264_clients);

    esp_err_t res = ESP_OK;
    while (res == ESP_OK) {
        if (xSemaphoreTake(s_cam.h264_ready, pdMS_TO_TICKS(5000)) != pdTRUE) {
            continue;  /* Timeout, try again */
        }

        if (xSemaphoreTake(s_cam.h264_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (s_cam.h264_send_size > 0) {
                res = httpd_resp_send_chunk(req,
                    (const char *)s_cam.h264_send_buf, s_cam.h264_send_size);
            }
            xSemaphoreGive(s_cam.h264_mutex);
        }
    }

    s_cam.h264_clients--;
    ESP_LOGI(TAG, "H.264 HTTP client disconnected (%d active)", s_cam.h264_clients);
    return res;
}

/* ========== Thermal Camera HTML Viewer & Raw Endpoint ========== */

/*
 * Self-contained HTML page that fetches raw Y16 data from /thermal/raw
 * and renders it on a canvas with iron colormap.  No external dependencies.
 */
static const char THERMAL_HTML[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<title>Thermal Camera</title>"
"<style>"
"body{background:#111;color:#eee;font-family:monospace;margin:0;display:flex;"
"flex-direction:column;align-items:center;justify-content:center;height:100vh}"
"canvas{image-rendering:pixelated;border:1px solid #444}"
"#info{margin-top:8px;font-size:14px}"
"</style></head><body>"
"<canvas id='c'></canvas>"
"<div id='info'>Connecting...</div>"
"<script>"
"const canvas=document.getElementById('c');"
"const ctx=canvas.getContext('2d');"
"const info=document.getElementById('info');"
"let frames=0,lastT=performance.now(),pollMs=111,errCnt=0,tempStr='';"
/* Iron colormap LUT (256 entries) */
"const lut=new Uint8Array(256*3);"
"for(let i=0;i<256;i++){"
"  let r,g,b;"
"  if(i<64){r=0;g=0;b=i*4;}"
"  else if(i<128){let t=(i-64)*4;r=t;g=0;b=255-t;}"
"  else if(i<192){let t=(i-128)*4;r=255;g=t;b=0;}"
"  else{let t=(i-192)*4;r=255;g=255;b=t;}"
"  lut[i*3]=r;lut[i*3+1]=g;lut[i*3+2]=b;"
"}"
"const SCALE=4;"
"let imgData=null,w=0,h=0;"
"async function poll(){"
"  try{"
"    const resp=await fetch('/thermal/raw');"
"    if(resp.status===204||!resp.ok){"
"      errCnt++;pollMs=resp.status===204?1000:Math.min(5000,111*Math.pow(2,errCnt));"
"      info.textContent=resp.status===204?'Waiting for thermal camera...':'Error '+resp.status;return;"
"    }"
"    errCnt=0;pollMs=111;"
"    const tw=parseInt(resp.headers.get('X-Thermal-Width'))||80;"
"    const th=parseInt(resp.headers.get('X-Thermal-Height'))||60;"
"    if(tw!==w||th!==h){"
"      w=tw;h=th;canvas.width=w;canvas.height=h;"
"      canvas.style.width=(w*SCALE)+'px';canvas.style.height=(h*SCALE)+'px';"
"      imgData=ctx.createImageData(w,h);"
"    }"
"    const buf=await resp.arrayBuffer();"
"    const raw=new Uint8Array(buf);"
/* Compact format from server: 4-byte header (vmin_LE16, vmax_LE16) + npix bytes (8-bit normalized).
 * ~19KB instead of 38KB raw Y16. Temperature calculated from vmin/vmax in header. */
"    const npix=w*h;"
"    let y=new Uint8Array(npix);"
"    let isY16=false;"
"    if(raw.length===npix+4){"
"      isY16=true;"
"      const vmin=raw[0]|(raw[1]<<8);"
"      const vmax=raw[2]|(raw[3]<<8);"
"      for(let i=0;i<npix;i++)y[i]=raw[4+i];"
"      const cx=(w>>1),cy=(h>>1);"
"      const spotN=raw[4+cy*w+cx];"
"      const rng=vmax>vmin?vmax-vmin:1;"
"      const spotRaw=vmin+spotN*rng/255;"
"      const spotC=(spotRaw/100-273.15).toFixed(1);"
"      const tminC=(vmin/100-273.15).toFixed(1);"
"      const tmaxC=(vmax/100-273.15).toFixed(1);"
"      tempStr=' | '+tminC+'~'+tmaxC+'C  center:'+spotC+'C';"
"    }else if(raw.length>=npix){"
"      for(let i=0;i<npix;i++)y[i]=raw[i];"
"      tempStr='';"
"    }"
"    const d=imgData.data;"
"    for(let i=0;i<npix;i++){"
"      const idx=y[i];"
"      d[i*4]=lut[idx*3];d[i*4+1]=lut[idx*3+1];d[i*4+2]=lut[idx*3+2];d[i*4+3]=255;"
"    }"
"    ctx.putImageData(imgData,0,0);"
/* Draw center crosshair (spotmeter) */
"    if(isY16){"
"      ctx.strokeStyle='rgba(255,255,255,0.7)';ctx.lineWidth=1;"
"      const cx=w/2,cy=h/2;"
"      ctx.beginPath();ctx.moveTo(cx-4,cy);ctx.lineTo(cx+4,cy);"
"      ctx.moveTo(cx,cy-4);ctx.lineTo(cx,cy+4);ctx.stroke();"
"    }"
"    frames++;"
"    const now=performance.now();"
"    if(now-lastT>=1000){"
"      const fps=(frames*1000/(now-lastT)).toFixed(1);"
"      const fmt=isY16?'Y16':'YUY2';"
"      info.textContent=w+'x'+h+' '+fmt+' | '+fps+' fps'+tempStr;"
"      frames=0;lastT=now;"
"    }"
"  }catch(e){info.textContent='Error: '+e.message;}"
"}"
"function loop(){poll().finally(()=>setTimeout(loop,pollMs));}loop();"
"</script></body></html>";

static esp_err_t thermal_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, THERMAL_HTML, strlen(THERMAL_HTML));
}

static esp_err_t thermal_raw_handler(httpd_req_t *req)
{
    if (!thermal_camera_is_active()) {
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, NULL, 0);
    }

    unsigned tw = thermal_camera_width();
    unsigned th = thermal_camera_height();
    size_t frame_sz = thermal_camera_frame_size();
    unsigned npix = tw * th;

    uint16_t *y16_buf = heap_caps_malloc(frame_sz, MALLOC_CAP_SPIRAM);
    if (!y16_buf) {
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, NULL, 0);
    }

    if (!thermal_camera_get_frame(y16_buf)) {
        free(y16_buf);
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, NULL, 0);
    }

    /* Compact format: 4-byte header (vmin_u16 LE, vmax_u16 LE) + npix bytes (8-bit normalized)
     * Reduces 38KB Y16 → ~19KB, halving bandwidth for HaLow. */
    uint16_t vmin = 65535, vmax = 0;
    for (unsigned i = 0; i < npix; i++) {
        if (y16_buf[i] < vmin) vmin = y16_buf[i];
        if (y16_buf[i] > vmax) vmax = y16_buf[i];
    }
    uint16_t rng = (vmax > vmin) ? (vmax - vmin) : 1;

    size_t out_sz = 4 + npix;
    uint8_t *out = heap_caps_malloc(out_sz, MALLOC_CAP_SPIRAM);
    if (!out) { free(y16_buf); return httpd_resp_send(req, NULL, 0); }

    /* Header: vmin(LE16) + vmax(LE16) */
    out[0] = vmin & 0xFF; out[1] = (vmin >> 8) & 0xFF;
    out[2] = vmax & 0xFF; out[3] = (vmax >> 8) & 0xFF;

    /* Normalize to 8-bit */
    for (unsigned i = 0; i < npix; i++) {
        out[4 + i] = (uint8_t)(((uint32_t)(y16_buf[i] - vmin) * 255) / rng);
    }
    free(y16_buf);

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Expose-Headers",
                       "X-Thermal-Width,X-Thermal-Height");

    char w_str[12], h_str[12];
    snprintf(w_str, sizeof(w_str), "%u", tw);
    snprintf(h_str, sizeof(h_str), "%u", th);
    httpd_resp_set_hdr(req, "X-Thermal-Width", w_str);
    httpd_resp_set_hdr(req, "X-Thermal-Height", h_str);

    esp_err_t res = httpd_resp_send(req, (const char *)out, out_sz);
    free(out);
    return res;
}

/* ========== HTTP Server Setup ========== */

static const httpd_uri_t uri_stream = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = stream_handler,
};

static const httpd_uri_t uri_status = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
};

static const httpd_uri_t uri_thermal = {
    .uri = "/thermal",
    .method = HTTP_GET,
    .handler = thermal_page_handler,
};

static const httpd_uri_t uri_thermal_raw = {
    .uri = "/thermal/raw",
    .method = HTTP_GET,
    .handler = thermal_raw_handler,
};

static const httpd_uri_t uri_video = {
    .uri = "/video",
    .method = HTTP_GET,
    .handler = h264_stream_handler,
};

static const httpd_uri_t uri_sdp = {
    .uri = "/stream.sdp",
    .method = HTTP_GET,
    .handler = sdp_handler,
};

httpd_handle_t camera_stream_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.max_open_sockets = 4;
    config.stack_size = 8192;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_stream);
        httpd_register_uri_handler(server, &uri_status);
        httpd_register_uri_handler(server, &uri_video);
        httpd_register_uri_handler(server, &uri_sdp);
        httpd_register_uri_handler(server, &uri_thermal);
        httpd_register_uri_handler(server, &uri_thermal_raw);
        ESP_LOGI(TAG, "HTTP server started");
        ESP_LOGI(TAG, "  H.264:   rtp://@:5600 (primary) + http://<ip>/video (backup, %d fps)", STREAM_TARGET_FPS);
        ESP_LOGI(TAG, "  SDP:     http://<ip>/stream.sdp (open in VLC)");
        ESP_LOGI(TAG, "  Thermal: http://<ip>/thermal  (browser)");
        ESP_LOGI(TAG, "  Status:  http://<ip>/status");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }

    return server;
}

float camera_get_fps(void)
{
    return s_cam.fps;
}
