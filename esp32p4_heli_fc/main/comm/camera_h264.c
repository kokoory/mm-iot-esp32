/*
 * MIPI-CSI Camera + HW JPEG/H.264 Encoder for ESP32-P4
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pipelines:
 *   MJPEG: OV5647 → MIPI-CSI → ISP (RAW8→YUV422) → 2x downscale → HW JPEG → HTTP/TCP /mjpeg
 *   H.264: (set ENABLE_MJPEG=0 to use) OV5647 → ISP (RAW8→YUV420) → HW H.264 → UDP RTP :5600
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

/* Camera capture format (sensor → CSI → ISP) */
#define CAM_FORMAT          "MIPI_2lane_24Minput_RAW8_800x640_50fps"
#define CAM_CAPTURE_W       800
#define CAM_CAPTURE_H       640

/* Stream resolution (software 2x downscale before JPEG encode).
 * 400x320 at Q=40 ≈ 3-5KB per frame → 3-4 RTP packets → less SPI pressure,
 * higher probability all packets arrive (less flickering). */
#define CAM_WIDTH           400
#define CAM_HEIGHT          320

/* MIPI CSI lane bitrate */
#define CSI_LANE_BITRATE_MBPS  200

/* JPEG quality (1-100, higher = better quality, larger file) */
#define JPEG_QUALITY        30           /* Low quality for HaLow bandwidth */
#define JPEG_BUF_SIZE       (100 * 1024) /* 100KB for low-quality 800x640 */

/* H.264 encoder settings (primary) */
#define H264_GOP            35
#define H264_FPS            1
#define H264_QP_MIN         28
#define H264_QP_MAX         42
#define H264_BITRATE        350000
#define H264_BUF_SIZE       (100 * 1024)

/* MJPEG encoder settings (primary — no I-frame burst, smooth SPI traffic) */
#define MJPEG_QUALITY       40           /* JPEG quality 1-100 (40 at 400x320 ≈ 3-5KB) */
#define MJPEG_FPS           2            /* 2 FPS — minimal SPI pressure */

/* H.264 delivered via UDP RTP + HTTP/TCP backup */
#define RTP_PORT            5600
#define RTP_PKT_MAX_SIZE    1200         /* Small packets for HaLow stability */
#define RTP_HEADER_SIZE     12
#define RTP_PAYLOAD_TYPE    96           /* Dynamic PT for H.264 */
#define RTP_PACING_MS       10           /* Base pacing — prevents SPI 16-page overflow */
#define RTP_PACING_I_MS     12           /* I-frame pacing: must be >=12ms to avoid SPI page overflow */
#define RTP_MAX_P_FRAME     8000         /* Skip P-frames larger than 8KB */
#define RTP_I_WAIT_TIMEOUT_MS 300        /* Max wait for TX drain during I-frame (prevents infinite stall) */
#define RTP_DEFAULT_DEST_IP "192.168.1.143"  /* Default GCS IP, updated by MAVLink heartbeat */

/* Set to 1 to enable MJPEG, 0 for H.264 (primary) */
#define ENABLE_MJPEG        1

/* Stream frame rate limit (camera captures at 50fps, we stream fewer) */
#if ENABLE_MJPEG
#define STREAM_TARGET_FPS   MJPEG_FPS
#else
#define STREAM_TARGET_FPS   H264_FPS
#endif

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

    /* Raw frame double buffers (from ISP at capture resolution) */
    uint8_t *raw_buf[NUM_BUFS];
    size_t raw_buf_size;

    /* Downscale buffer (capture → stream resolution) */
    uint8_t *scale_buf;
    size_t scale_buf_size;

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

    /* TX byte rate tracking (prevent SPI overflow) */
    uint32_t tx_bytes_this_sec;     /* Bytes sent in current 1-second window */
    uint32_t tx_window_start;       /* Tick count at window start */

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

/* Dynamic TX byte rate budget based on link quality.
 * Reserve 30% of usable throughput for MAVLink telemetry.
 * Update every 2 seconds from MCS/loss stats. */
#define RTP_TX_BUDGET_MIN        15000   /* 15 KB/s floor (MCS0 / very lossy) */
#define RTP_TX_BUDGET_MAX        40000   /* 40 KB/s ceiling — must share SPI with MAVLink */
#define RTP_TX_BUDGET_DEFAULT    25000   /* Default before first measurement */
#define RTP_BUDGET_WINDOW_MS       200   /* Budget window: 200ms (prevents burst within 1s) */
#define RTP_BUDGET_UPDATE_MS      2000   /* Re-evaluate link quality every 2s */
#define RTP_MAVLINK_RESERVE_PCT     30   /* Reserve 30% of throughput for MAVLink */

static uint32_t s_rtp_tx_budget = RTP_TX_BUDGET_DEFAULT;
static uint32_t s_rtp_budget_update_ms = 0;

static void rtp_update_tx_budget(void)
{
    uint32_t now = xTaskGetTickCount();
    if ((now - s_rtp_budget_update_ms) < pdMS_TO_TICKS(RTP_BUDGET_UPDATE_MS)) {
        return;
    }
    s_rtp_budget_update_ms = now;

    app_wlan_link_quality_t lq;
    app_wlan_get_link_quality(&lq);

    /* Video gets (100 - RESERVE)% of usable throughput */
    uint32_t budget = lq.throughput_bps * (100 - RTP_MAVLINK_RESERVE_PCT) / 100;

    /* Clamp */
    if (budget < RTP_TX_BUDGET_MIN) budget = RTP_TX_BUDGET_MIN;
    if (budget > RTP_TX_BUDGET_MAX) budget = RTP_TX_BUDGET_MAX;

    if (budget != s_rtp_tx_budget) {
        ESP_LOGI(TAG, "[rtp] TX budget: %lu→%lu B/s (MCS%d %dMHz %s loss=%d%% throughput=%lu B/s)",
                 (unsigned long)s_rtp_tx_budget, (unsigned long)budget,
                 lq.mcs, lq.bw_mhz, lq.sgi ? "SGI" : "LGI",
                 lq.loss_pct, (unsigned long)lq.throughput_bps);
        s_rtp_tx_budget = budget;
    }
}

static bool rtp_tx_budget_check(size_t bytes)
{
    uint32_t now = xTaskGetTickCount();
    if ((now - s_cam.tx_window_start) >= pdMS_TO_TICKS(RTP_BUDGET_WINDOW_MS)) {
        s_cam.tx_bytes_this_sec = 0;
        s_cam.tx_window_start = now;
    }
    /* Scale budget to window size (budget is per-second, window is 200ms) */
    uint32_t window_budget = s_rtp_tx_budget * RTP_BUDGET_WINDOW_MS / 1000;
    if (s_cam.tx_bytes_this_sec + bytes > window_budget) {
        return false; /* over budget for this window */
    }
    s_cam.tx_bytes_this_sec += bytes;
    return true;
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
}

/* ========== MJPEG RTP Sender (RFC 2435) ========== */

/*
 * Send complete JPEG data (minus SOI/EOI) over RTP with RFC 2435 framing.
 *
 * RFC 2435 JPEG header (8 bytes, every packet):
 *   0:     Type-specific (0)
 *   1-3:   Fragment offset (24-bit big-endian)
 *   4:     Type (1 = YUV422)
 *   5:     Q (quality factor, 1-99 = standard tables)
 *   6:     Width / 8
 *   7:     Height / 8
 *
 * With Q < 128, VLC/gstreamer generate standard quantization tables from Q
 * and prepend SOI + DQT + SOF + DHT + SOS headers. Our data (full JPEG
 * minus SOI/EOI) starts with the JPEG's own markers, and the decoder's
 * parser overwrites the generated headers when it encounters the real ones.
 */
#define JPEG_RTP_HEADER_SIZE  8
#define JPEG_RTP_MAX_PAYLOAD  (RTP_PKT_MAX_SIZE - JPEG_RTP_HEADER_SIZE)

/* SPI stress detection: when pause_count jumps rapidly, completely pause
 * video sending to prevent morse_pageset_tx overflow → health task crash */
#define SPI_STRESS_THRESHOLD     5   /* pauses in one check → emergency stop */
#define SPI_STRESS_COOLDOWN_MS  500  /* pause this long after SPI stress */

static uint32_t s_spi_cooldown_until = 0;  /* tick count: don't send until */

static void rtp_send_jpeg_frame(const uint8_t *jpeg_data, size_t jpeg_len)
{
    if (s_cam.rtp_sock < 0) return;

    /* Periodically update TX budget based on link quality */
    rtp_update_tx_budget();

    /* On-demand: only send RTP when GCS is actively connected */
    if (!gcs_bridge_is_active()) {
        s_cam.rtp_frames_skipped++;
        return;
    }

    /* SPI stress cooldown: after detecting rapid page failures, stop all
     * video TX for SPI_STRESS_COOLDOWN_MS to let the buffer drain.
     * This prevents the morse_pageset error cascade → health task crash. */
    uint32_t now_tick = xTaskGetTickCount();
    if (s_spi_cooldown_until != 0 && (int32_t)(now_tick - s_spi_cooldown_until) < 0) {
        s_cam.rtp_frames_skipped++;
        return;
    }
    s_spi_cooldown_until = 0;

    /* Skip if TX pool is currently saturated */
    if (app_wlan_tx_is_paused()) {
        s_cam.rtp_frames_skipped++;
        return;
    }

    /* Adaptive frame skip */
    if (s_cam.skip_frames > 0) {
        s_cam.skip_frames--;
        s_cam.rtp_frames_skipped++;
        return;
    }

    uint32_t cur_pause = app_wlan_tx_pause_count();
    if (cur_pause > s_cam.last_pause_count) {
        uint32_t delta = cur_pause - s_cam.last_pause_count;
        if (delta >= SPI_STRESS_THRESHOLD) {
            /* Emergency: SPI page buffer under heavy stress.
             * Stop all video TX for cooldown period to prevent crash. */
            s_spi_cooldown_until = now_tick + pdMS_TO_TICKS(SPI_STRESS_COOLDOWN_MS);
            s_cam.last_pause_count = cur_pause;
            s_cam.rtp_frames_skipped++;
            ESP_LOGW(TAG, "[rtp] SPI stress: %lu pauses, cooling down %dms",
                     (unsigned long)delta, SPI_STRESS_COOLDOWN_MS);
            return;
        } else if (delta >= 3) {
            s_cam.skip_frames = delta / 3;
            ESP_LOGW(TAG, "[rtp] congestion: %lu pauses, skip %d frames",
                     (unsigned long)delta, s_cam.skip_frames);
        }
    }
    s_cam.last_pause_count = cur_pause;

    /* Update RTP destination from GCS bridge */
    uint32_t gcs_ip = gcs_bridge_get_ip();
    if (gcs_ip != 0 && gcs_ip != s_cam.rtp_dest_addr.sin_addr.s_addr) {
        s_cam.rtp_dest_addr.sin_addr.s_addr = gcs_ip;
        ESP_LOGI(TAG, "RTP destination updated to GCS: %s",
                 inet_ntoa(s_cam.rtp_dest_addr.sin_addr));
    }

    s_cam.rtp_frames_sent++;

    /* Strip SOI (FF D8) and EOI (FF D9) — VLC reconstructs them */
    const uint8_t *data = jpeg_data;
    size_t data_len = jpeg_len;
    if (data_len >= 2 && data[0] == 0xFF && data[1] == 0xD8) {
        data += 2;
        data_len -= 2;
    }
    if (data_len >= 2 && data[data_len - 2] == 0xFF && data[data_len - 1] == 0xD9) {
        data_len -= 2;
    }

    /* RTP timestamp: 90kHz clock */
    s_cam.rtp_ts += (90000 / MJPEG_FPS);

    /* Determine JPEG type from ISP configuration */
#if ENABLE_MJPEG && !defined(JPEG_ENCODE_IN_FORMAT_YUV420)
    uint8_t jpeg_type = 1;  /* YUV422 */
#else
    uint8_t jpeg_type = 0;  /* YUV420 */
#endif

    uint32_t offset = 0;
    while (offset < data_len) {
        /* Check TX flow control */
        if (app_wlan_tx_is_paused()) {
            s_cam.rtp_pkts_dropped++;
            s_cam.rtp_backoff_count++;
            vTaskDelay(pdMS_TO_TICKS(20));
            return; /* Abort frame — MJPEG frames are independent */
        }

        size_t chunk = data_len - offset;
        if (chunk > JPEG_RTP_MAX_PAYLOAD) chunk = JPEG_RTP_MAX_PAYLOAD;
        bool last = (offset + chunk >= data_len);

        size_t pkt_total = RTP_HEADER_SIZE + JPEG_RTP_HEADER_SIZE + chunk;

        /* Byte rate budget check */
        if (!rtp_tx_budget_check(pkt_total)) {
            s_cam.rtp_pkts_dropped++;
            return; /* Over budget — drop rest of frame */
        }

        uint8_t pkt[RTP_PKT_MAX_SIZE + RTP_HEADER_SIZE + JPEG_RTP_HEADER_SIZE];

        /* RTP header — payload type 26 (JPEG) */
        rtp_header_serialize(pkt, s_cam.rtp_seq++, s_cam.rtp_ts, s_cam.rtp_ssrc, last);
        pkt[1] = (last ? 0x80 : 0x00) | 26;

        /* RFC 2435 JPEG header */
        uint8_t *jhdr = pkt + RTP_HEADER_SIZE;
        jhdr[0] = 0;                             /* Type-specific */
        jhdr[1] = (offset >> 16) & 0xFF;         /* Fragment offset (MSB) */
        jhdr[2] = (offset >> 8) & 0xFF;
        jhdr[3] = offset & 0xFF;                 /* Fragment offset (LSB) */
        jhdr[4] = jpeg_type;                     /* 0=YUV420, 1=YUV422 */
        jhdr[5] = MJPEG_QUALITY;                 /* Q < 128: standard tables */
        jhdr[6] = CAM_WIDTH / 8;                 /* Width / 8 */
        jhdr[7] = CAM_HEIGHT / 8;                /* Height / 8 */

        /* JPEG data payload */
        memcpy(pkt + RTP_HEADER_SIZE + JPEG_RTP_HEADER_SIZE, data + offset, chunk);

        int ret = sendto(s_cam.rtp_sock, pkt, pkt_total, 0,
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

        offset += chunk;
        vTaskDelay(pdMS_TO_TICKS(RTP_PACING_MS));
    }
}

/* Legacy H.264 RTP sender (kept for reference, MJPEG is now primary) */
static void rtp_send_h264_frame(const uint8_t *buf, size_t len);

static void rtp_send_frame(const uint8_t *buf, size_t len)
{
    if (s_cam.rtp_sock < 0) return;

    /* Periodically update TX budget based on link quality */
    rtp_update_tx_budget();

    /* On-demand: only send RTP when GCS is actively connected */
    if (!gcs_bridge_is_active()) {
        s_cam.rtp_frames_skipped++;
        return;
    }

    /* Detect if this frame contains an I-frame (IDR NAL type 5 or SPS type 7) */
    bool has_idr = false;
    for (size_t i = 0; i + 4 < len; i++) {
        if (buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 0 && buf[i+3] == 1) {
            uint8_t nal_type = buf[i+4] & 0x1F;
            if (nal_type == 5 || nal_type == 7) { /* IDR slice or SPS */
                has_idr = true;
                break;
            }
        }
    }

    /* Adaptive frame skip: back off when TX pool is congested.
     * NEVER skip I-frames — without them the decoder can't recover. */
    if (s_cam.skip_frames > 0 && !has_idr) {
        s_cam.skip_frames--;
        s_cam.rtp_frames_skipped++;
        return;
    }
    s_cam.skip_frames = 0; /* Reset skip counter when I-frame arrives or skip exhausted */

    /* Size limit: only skip oversized P-frames. I-frames are ALWAYS sent. */
    if (!has_idr && len > RTP_MAX_P_FRAME) {
        s_cam.rtp_frames_skipped++;
        return;
    }

    /* Check if TX pool congestion increased — trigger adaptive backoff (P-frames only) */
    uint32_t cur_pause = app_wlan_tx_pause_count();
    if (cur_pause > s_cam.last_pause_count) {
        uint32_t delta = cur_pause - s_cam.last_pause_count;
        if (delta >= 3) {
            /* Significant burst — skip 1 P-frame per 3 pauses */
            s_cam.skip_frames = delta / 3;
            ESP_LOGW(TAG, "[rtp] congestion: %lu pauses, skip %d P-frames",
                     (unsigned long)delta, s_cam.skip_frames);
        }
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
        ESP_LOGI(TAG, "[rtp-debug] frame #%lu len=%u %s bytes[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x",
                 (unsigned long)s_cam.rtp_frames_sent, (unsigned)len,
                 has_idr ? "IDR" : "P",
                 len > 0 ? buf[0] : 0, len > 1 ? buf[1] : 0,
                 len > 2 ? buf[2] : 0, len > 3 ? buf[3] : 0,
                 len > 4 ? buf[4] : 0, len > 5 ? buf[5] : 0,
                 len > 6 ? buf[6] : 0, len > 7 ? buf[7] : 0);
    }

    const uint8_t *p = buf;
    const uint8_t *end = buf + len;

    /* NAL-type aware pacing: I-frames get more spacing to avoid TX burst */
    int pacing_ms = has_idr ? RTP_PACING_I_MS : RTP_PACING_MS;

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
            vTaskDelay(pdMS_TO_TICKS(pacing_ms));
        } else {
            /* FU-A Fragmentation (RFC 6184) */
            uint8_t nal_header = nal_start[0];
            const uint8_t *payload = nal_start + 1;
            size_t payload_len = nal_len - 1;

            while (payload_len > 0) {
                /* Check TX flow control for each fragment.
                 * For I-frames: WAIT until TX unpauses (never abort I-frame)
                 * For P-frames: abort to avoid partial FU-A corruption */
                if (app_wlan_tx_is_paused()) {
                    if (has_idr) {
                        /* I-frame: wait for TX pool to drain, with timeout to
                         * prevent infinite stall if MM6108 enters error state */
                        uint32_t wait_start = xTaskGetTickCount();
                        while (app_wlan_tx_is_paused()) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                            if ((xTaskGetTickCount() - wait_start) > pdMS_TO_TICKS(RTP_I_WAIT_TIMEOUT_MS)) {
                                ESP_LOGW(TAG, "[rtp] I-frame TX wait timeout (%dms), aborting frame",
                                         RTP_I_WAIT_TIMEOUT_MS);
                                s_cam.rtp_pkts_dropped++;
                                s_cam.rtp_backoff_count++;
                                s_cam.skip_frames = 4; /* Skip more after timeout */
                                vTaskDelay(pdMS_TO_TICKS(50));
                                return;
                            }
                        }
                    } else {
                        /* P-frame: abort — partial FU-A is undecodable */
                        s_cam.rtp_pkts_dropped++;
                        s_cam.rtp_backoff_count++;
                        s_cam.skip_frames = 2;
                        vTaskDelay(pdMS_TO_TICKS(20));
                        return;
                    }
                }

                size_t chunk = (payload_len > (RTP_PKT_MAX_SIZE - 2)) ? (RTP_PKT_MAX_SIZE - 2) : payload_len;
                size_t pkt_total = chunk + RTP_HEADER_SIZE + 2;
                bool first = (payload == nal_start + 1);
                bool last = (chunk == payload_len);

                /* Byte rate budget check: wait if over budget (prevents SPI overflow) */
                if (!rtp_tx_budget_check(pkt_total)) {
                    /* Over byte rate budget — wait for next window.
                     * For P-frames, abort; for I-frames, wait briefly. */
                    if (!has_idr) {
                        s_cam.rtp_pkts_dropped++;
                        s_cam.skip_frames = 2;
                        return;
                    }
                    /* I-frame: wait 50ms then recheck */
                    vTaskDelay(pdMS_TO_TICKS(50));
                    rtp_tx_budget_check(pkt_total); /* retry (window may have reset) */
                }

                uint8_t fu_indicator = (nal_header & 0xE0) | 28; // FU-A type 28
                uint8_t fu_header = (first ? 0x80 : 0) | (last ? 0x40 : 0) | nal_type;

                uint8_t pkt[RTP_PKT_MAX_SIZE + RTP_HEADER_SIZE + 2];
                rtp_header_serialize(pkt, s_cam.rtp_seq++, s_cam.rtp_ts, s_cam.rtp_ssrc, (last && next == end));
                pkt[RTP_HEADER_SIZE] = fu_indicator;
                pkt[RTP_HEADER_SIZE + 1] = fu_header;
                memcpy(pkt + RTP_HEADER_SIZE + 2, payload, chunk);

                int ret = sendto(s_cam.rtp_sock, pkt, pkt_total, 0,
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

                vTaskDelay(pdMS_TO_TICKS(pacing_ms));

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

    int num_sensors = &__esp_cam_sensor_detect_fn_array_end - &__esp_cam_sensor_detect_fn_array_start;
    ESP_LOGI(TAG, "Scanning %d registered sensor driver(s)...", num_sensors);

    esp_cam_sensor_device_t *cam = NULL;
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
        ESP_LOGI(TAG, "  Probing SCCB addr 0x%02X ...", p->sccb_addr);
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
        ESP_LOGI(TAG, "  No sensor at 0x%02X", p->sccb_addr);
        ESP_ERROR_CHECK(esp_sccb_del_i2c_io(cam_config.sccb_handle));
    }

    if (!cam) {
        ESP_LOGE(TAG, "No camera sensor detected! Check ribbon cable connection.");
        return ESP_ERR_NOT_FOUND;
    }

    /* Log detected sensor identity */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Detected camera : %s", cam->name ? cam->name : "unknown");
    ESP_LOGI(TAG, "  Product ID (PID): 0x%04X", (unsigned)cam->id.pid);
    ESP_LOGI(TAG, "  Manufacturer ID : 0x%02X%02X", cam->id.midh, cam->id.midl);
    ESP_LOGI(TAG, "  Version         : 0x%02X", cam->id.ver);
    ESP_LOGI(TAG, "========================================");

    esp_cam_sensor_format_array_t fmt_array = {0};
    esp_cam_sensor_query_format(cam, &fmt_array);
    const esp_cam_sensor_format_t *formats = fmt_array.format_array;
    ESP_LOGI(TAG, "Supported formats (%d):", fmt_array.count);
    for (int i = 0; i < fmt_array.count; i++) {
        ESP_LOGI(TAG, "  [%d] %s (%dx%d @ %dfps)", i, formats[i].name,
                 formats[i].width, formats[i].height, formats[i].fps);
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
    ESP_LOGI(TAG, "Capture: %dx%d → Stream: %dx%d, format: %s",
             CAM_CAPTURE_W, CAM_CAPTURE_H, CAM_WIDTH, CAM_HEIGHT, CAM_FORMAT);

    s_cam.frame_ready = xSemaphoreCreateBinary();
    s_cam.jpeg_mutex = xSemaphoreCreateMutex();
    s_cam.frame_captured = xSemaphoreCreateBinary();
    s_cam.h264_ready = xSemaphoreCreateBinary();
    s_cam.h264_mutex = xSemaphoreCreateMutex();

    /* Allocate buffers at capture resolution (sensor → CSI → ISP output).
     * YUV420 = 1.5 bytes/pixel, YUV422 = 2 bytes/pixel.
     * When MJPEG is enabled and SDK lacks YUV420 JPEG input, ISP must output YUV422. */
#if ENABLE_MJPEG && !defined(JPEG_ENCODE_IN_FORMAT_YUV420)
    s_cam.raw_buf_size = CAM_CAPTURE_W * CAM_CAPTURE_H * 2;     /* YUV422 = 2 bytes/pixel */
#else
    s_cam.raw_buf_size = CAM_CAPTURE_W * CAM_CAPTURE_H * 3 / 2;  /* YUV420 = 1.5 bytes/pixel */
#endif
    s_cam.raw_buf_size = (s_cam.raw_buf_size + 127) & ~127;  /* 128-byte align for H.264 HW encoder */

    for (int i = 0; i < NUM_BUFS; i++) {
        /* Raw buffers: 128-byte aligned for CSI DMA + H.264 HW encoder */
        s_cam.raw_buf[i] = heap_caps_aligned_calloc(128, 1, s_cam.raw_buf_size,
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

    /* Downscale buffer: stream resolution (software 2x downscale before JPEG encode) */
#if ENABLE_MJPEG && !defined(JPEG_ENCODE_IN_FORMAT_YUV420)
    s_cam.scale_buf_size = CAM_WIDTH * CAM_HEIGHT * 2;   /* YUV422 */
#else
    s_cam.scale_buf_size = CAM_WIDTH * CAM_HEIGHT * 3 / 2; /* YUV420 */
#endif
    s_cam.scale_buf_size = (s_cam.scale_buf_size + 127) & ~127;
    s_cam.scale_buf = heap_caps_aligned_calloc(128, 1, s_cam.scale_buf_size,
                                                MALLOC_CAP_SPIRAM);
    if (!s_cam.scale_buf) {
        ESP_LOGE(TAG, "Failed to allocate downscale buffer");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Downscale buffer: %uKB (%dx%d → %dx%d)",
             (unsigned)(s_cam.scale_buf_size / 1024),
             CAM_CAPTURE_W, CAM_CAPTURE_H, CAM_WIDTH, CAM_HEIGHT);

    /* H.264 output buffer: 128-byte aligned for HW encoder DMA */
    s_cam.h264_buf = heap_caps_aligned_calloc(128, 1, H264_BUF_SIZE, MALLOC_CAP_SPIRAM);
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

    /* CSI controller — capture at full sensor resolution */
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CAM_CAPTURE_W,
        .v_res = CAM_CAPTURE_H,
        .lane_bit_rate_mbps = CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
#if ENABLE_MJPEG && !defined(JPEG_ENCODE_IN_FORMAT_YUV420)
        .output_data_color_type = CAM_CTLR_COLOR_YUV422,
#else
        .output_data_color_type = CAM_CTLR_COLOR_YUV420,
#endif
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
    ESP_LOGI(TAG, "MIPI-CSI controller initialized (2-lane, %dx%d)", CAM_CAPTURE_W, CAM_CAPTURE_H);

    /* ISP pipeline */
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
#if ENABLE_MJPEG && !defined(JPEG_ENCODE_IN_FORMAT_YUV420)
        .output_data_color_type = ISP_COLOR_YUV422,
#else
        .output_data_color_type = ISP_COLOR_YUV420,
#endif
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CAM_CAPTURE_W,
        .v_res = CAM_CAPTURE_H,
    };
    ret = esp_isp_new_processor(&isp_config, &isp_proc);
    if (ret == ESP_OK) {
        esp_isp_enable(isp_proc);
#if ENABLE_MJPEG && !defined(JPEG_ENCODE_IN_FORMAT_YUV420)
        ESP_LOGI(TAG, "ISP pipeline enabled (RAW8 → YUV422 for MJPEG)");
#else
        ESP_LOGI(TAG, "ISP pipeline enabled (RAW8 → YUV420)");
#endif
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
    /* HW JPEG encoder — primary encoder for MJPEG streaming */
    jpeg_encode_engine_cfg_t jpeg_enc_cfg = { .timeout_ms = 200 };
    ret = jpeg_new_encoder_engine(&jpeg_enc_cfg, &s_cam.jpeg_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HW JPEG encoder initialized (quality=%d, MJPEG primary)", MJPEG_QUALITY);
    } else {
        ESP_LOGW(TAG, "HW JPEG init failed: %s — falling back to H.264", esp_err_to_name(ret));
        s_cam.jpeg_handle = NULL;
    }
#endif

    /* HW H.264 encoder (fallback when MJPEG unavailable) */
#if HAS_HW_H264
    esp_h264_enc_cfg_hw_t h264_cfg = {
        .gop = H264_GOP,
        .fps = H264_FPS,
        .res = { .width = CAM_CAPTURE_W, .height = CAM_CAPTURE_H },
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
                     CAM_CAPTURE_W, CAM_CAPTURE_H, H264_GOP, H264_BITRATE / 1000);
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

#if !ENABLE_MJPEG
    /* H.264: UDP RTP + HTTP/TCP backup */
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
#else
    /* MJPEG: HTTP/TCP only — no UDP socket needed.
     * TCP backpressure prevents SPI overflow naturally. */
    s_cam.rtp_sock = -1;
    ESP_LOGI(TAG, "MJPEG mode: HTTP/TCP streaming only (no RTP/UDP)");
#endif

    /* Start capture task */
    xTaskCreatePinnedToCore(camera_capture_task, "cam_task", 8192, NULL, 5,
                            &s_cam.cam_task_handle, 1);

    s_cam.initialized = true;
    s_cam.stats_start_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Camera pipeline initialized successfully");
    return ESP_OK;
}

/* ========== Software 2x Downscale ========== */

/*
 * Nearest-neighbor 2x downscale for YUV422 packed (YUYV).
 * Every YUYV macro pixel covers 2 horizontal pixels.
 * Downscale: take every other macro pixel from every other row.
 * src: CAM_CAPTURE_W x CAM_CAPTURE_H  →  dst: CAM_WIDTH x CAM_HEIGHT
 */
static void downscale_2x_yuv422(const uint8_t *src, uint8_t *dst,
                                  int src_w, int src_h)
{
    int dst_w = src_w / 2;
    /* src stride: 2 bytes per pixel (YUYV = 4 bytes per 2 pixels) */
    int src_stride = src_w * 2;
    /* dst stride: same ratio */
    int dst_stride = dst_w * 2;

    for (int y = 0; y < src_h; y += 2) {
        const uint8_t *srow = src + y * src_stride;
        uint8_t *drow = dst + (y / 2) * dst_stride;
        /* Each YUYV macro pixel = 4 bytes (Y0 U Y1 V) covers 2 src pixels.
         * Skip every other macro pixel (stride of 8 bytes = 4 src pixels). */
        for (int x = 0; x < src_w; x += 4) {
            /* Copy one macro pixel (4 bytes), skip the next */
            const uint8_t *sp = srow + x * 2;
            *drow++ = sp[0]; /* Y0 */
            *drow++ = sp[1]; /* U  */
            *drow++ = sp[2]; /* Y1 */
            *drow++ = sp[3]; /* V  */
        }
    }
}

/*
 * Nearest-neighbor 2x downscale for YUV420 planar (I420).
 * Y plane: subsample both axes by 2.
 * U, V planes: already half-res in both axes, subsample by 2 again.
 */
static void downscale_2x_yuv420(const uint8_t *src, uint8_t *dst,
                                  int src_w, int src_h)
{
    int dst_w = src_w / 2;
    int dst_h = src_h / 2;

    /* Y plane */
    const uint8_t *sy = src;
    uint8_t *dy = dst;
    for (int y = 0; y < src_h; y += 2) {
        const uint8_t *srow = sy + y * src_w;
        for (int x = 0; x < src_w; x += 2) {
            *dy++ = srow[x];
        }
    }

    /* U plane (src U is src_w/2 x src_h/2) */
    int src_uv_w = src_w / 2;
    int src_uv_h = src_h / 2;
    const uint8_t *su = src + src_w * src_h;
    uint8_t *du = dst + dst_w * dst_h;
    for (int y = 0; y < src_uv_h; y += 2) {
        const uint8_t *srow = su + y * src_uv_w;
        for (int x = 0; x < src_uv_w; x += 2) {
            *du++ = srow[x];
        }
    }

    /* V plane */
    const uint8_t *sv = su + src_uv_w * src_uv_h;
    uint8_t *dv = dst + dst_w * dst_h + (dst_w / 2) * (dst_h / 2);
    for (int y = 0; y < src_uv_h; y += 2) {
        const uint8_t *srow = sv + y * src_uv_w;
        for (int x = 0; x < src_uv_w; x += 2) {
            *dv++ = srow[x];
        }
    }
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

        /* Software 2x downscale for MJPEG: capture res → stream res.
         * H.264 uses full capture resolution directly. */
#if ENABLE_MJPEG
#if !defined(JPEG_ENCODE_IN_FORMAT_YUV420)
        downscale_2x_yuv422(frame_data, s_cam.scale_buf,
                            CAM_CAPTURE_W, CAM_CAPTURE_H);
#else
        downscale_2x_yuv420(frame_data, s_cam.scale_buf,
                            CAM_CAPTURE_W, CAM_CAPTURE_H);
#endif
        /* Flush downscaled buffer to PSRAM for HW encoder DMA access */
        esp_cache_msync(s_cam.scale_buf, s_cam.scale_buf_size,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        uint8_t *encode_data = s_cam.scale_buf;
#endif /* ENABLE_MJPEG */

        /* === MJPEG encode → send via UDP RTP (Primary path) === */
        int64_t t2 = t1;
        int64_t t4 = t1;
        size_t h264_size_out = 0;
#if ENABLE_MJPEG && HAS_HW_JPEG
        if (s_cam.jpeg_handle) {
            int wr_idx = s_cam.jpeg_write_idx;

            /* Try YUV420 first (ESP-IDF v5.4+), fall back to YUV422 */
#ifdef JPEG_ENCODE_IN_FORMAT_YUV420
            jpeg_encode_cfg_t jpeg_cfg = {
                .src_type = JPEG_ENCODE_IN_FORMAT_YUV420,
                .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
                .image_quality = MJPEG_QUALITY,
                .width = CAM_WIDTH,
                .height = CAM_HEIGHT,
            };
#else
            jpeg_encode_cfg_t jpeg_cfg = {
                .src_type = JPEG_ENCODE_IN_FORMAT_YUV422,
                .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
                .image_quality = MJPEG_QUALITY,
                .width = CAM_WIDTH,
                .height = CAM_HEIGHT,
            };
#endif

            uint32_t jpg_size = 0;
            esp_err_t ret = jpeg_encoder_process(s_cam.jpeg_handle, &jpeg_cfg,
                                                  encode_data, s_cam.scale_buf_size,
                                                  s_cam.jpeg_buf[wr_idx], JPEG_BUF_SIZE,
                                                  &jpg_size);
            t4 = esp_timer_get_time();

            if (ret == ESP_OK && jpg_size > 0) {
                h264_size_out = jpg_size; /* Reuse stat counter for encoded bytes */

                /* Sync cache: JPEG encoder wrote to PSRAM */
                esp_cache_msync(s_cam.jpeg_buf[wr_idx],
                                (jpg_size + 63) & ~63,
                                ESP_CACHE_MSYNC_FLAG_DIR_M2C);

                /* HTTP/TCP MJPEG: update double buffer for HTTP clients.
                 * No RTP/UDP — TCP backpressure handles flow control. */
                if (xSemaphoreTake(s_cam.jpeg_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    s_cam.jpeg_size[wr_idx] = jpg_size;
                    s_cam.jpeg_write_idx = (wr_idx + 1) % NUM_BUFS;
                    s_cam.jpeg_read_idx = wr_idx;
                    xSemaphoreGive(s_cam.jpeg_mutex);
                }
                xSemaphoreGive(s_cam.frame_ready);
            } else if (ret != ESP_OK) {
                ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(ret));
            }
        }
        /* Fallback: H.264 encode if MJPEG handle not available */
        else
#elif HAS_HW_H264
        /* H.264 only path (when MJPEG disabled) */
#endif
        {
#if HAS_HW_H264
            if (s_cam.h264_handle) {
                esp_h264_enc_in_frame_t in_frame = {
                    .raw_data = { .buffer = frame_data },
                };
                in_frame.raw_data.len = CAM_CAPTURE_W * CAM_CAPTURE_H * 3 / 2;

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
                    esp_cache_msync(s_cam.h264_buf,
                                    (out_frame.length + 63) & ~63,
                                    ESP_CACHE_MSYNC_FLAG_DIR_M2C);
                    rtp_send_frame(s_cam.h264_buf, out_frame.length);

                    if (xSemaphoreTake(s_cam.h264_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        memcpy(s_cam.h264_send_buf, s_cam.h264_buf, out_frame.length);
                        s_cam.h264_send_size = out_frame.length;
                        xSemaphoreGive(s_cam.h264_mutex);
                        xSemaphoreGive(s_cam.h264_ready);
                    }
                }
            }
#endif
        }

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
                ESP_LOGI(TAG, "[perf] %ld frames: wait=%ldms enc=%ldms total=%ldms | enc=%luKB | http=%d",
                         (long)stat_frames,
                         (long)(stat_wait_us / stat_frames / 1000),
                         (long)(stat_h264_us / stat_frames / 1000),
                         (long)((stat_wait_us + stat_h264_us) / stat_frames / 1000),
                         (unsigned long)(stat_h264_bytes / 1024),
#if ENABLE_MJPEG
                         s_cam.mjpeg_clients);
#else
                         s_cam.h264_clients);
#endif
            }
#if ENABLE_MJPEG
            ESP_LOGI(TAG, "[mjpeg] http_clients=%d | tx_paused=%s pause_cnt=%lu",
                     s_cam.mjpeg_clients,
                     app_wlan_tx_is_paused() ? "YES" : "no",
                     (unsigned long)app_wlan_tx_pause_count());
#else
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
#endif
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
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req,
        "<html><body style='background:#111;color:#eee;font-family:monospace;text-align:center;padding:40px'>"
        "<h2>ESP32-P4 Helicopter</h2>"
#if ENABLE_MJPEG
        "<p style='color:#0f0;font-size:14px'>Primary: HTTP/TCP MJPEG (400x320)</p>"
        "<p><a href='/mjpeg' style='color:#0af;font-size:20px'>Live Video (MJPEG)</a></p>"
#else
        "<p style='color:#888;font-size:14px'>Primary: H.264 RTP on port 5600</p>"
        "<p><a href='/stream.sdp' style='color:#0af;font-size:16px'>stream.sdp</a> — open in VLC</p>"
        "<p><a href='/video' style='color:#0af;font-size:20px'>H.264 Stream</a></p>"
#endif
        "<p><a href='/thermal' style='color:#0af;font-size:20px'>Thermal Camera</a></p>"
        "<p><a href='/status' style='color:#0af;font-size:20px'>System Status</a></p>"
        "</body></html>", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[896];
#if ENABLE_MJPEG
    snprintf(buf, sizeof(buf),
        "{\"initialized\":%s,\"resolution\":\"%dx%d\",\"fps\":%.1f,"
        "\"encoder\":\"mjpeg_hw\",\"transport\":\"http_tcp\","
        "\"mjpeg_quality\":%d,\"mjpeg_clients\":%d,"
        "\"tx_paused\":%s,\"tx_pause_count\":%lu,"
        "\"pipeline\":\"csi_isp_mjpeg\","
        "\"stream_url\":\"/mjpeg\"}",
        s_cam.initialized ? "true" : "false",
        CAM_WIDTH, CAM_HEIGHT, s_cam.fps,
        MJPEG_QUALITY, s_cam.mjpeg_clients,
        app_wlan_tx_is_paused() ? "true" : "false",
        (unsigned long)app_wlan_tx_pause_count());
#else
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
        CAM_CAPTURE_W, CAM_CAPTURE_H, s_cam.fps,
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
#endif

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
    char sdp[512];
#if ENABLE_MJPEG
    /* MJPEG RTP: payload type 26 (JPEG), RFC 2435 */
    snprintf(sdp, sizeof(sdp),
        "v=0\r\n"
        "o=- 0 0 IN IP4 0.0.0.0\r\n"
        "s=ESP32-P4 MJPEG\r\n"
        "c=IN IP4 0.0.0.0\r\n"
        "t=0 0\r\n"
        "m=video %d RTP/AVP 26\r\n"
        "a=rtpmap:26 JPEG/90000\r\n"
        "a=framerate:%d\r\n",
        RTP_PORT, MJPEG_FPS);
#else
    /* H.264 RTP: dynamic payload type 96 */
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
#endif

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

/* ========== MJPEG HTTP Multipart Stream (TCP — primary video) ========== */

/*
 * Serve MJPEG as multipart/x-mixed-replace over HTTP/TCP.
 * Browser/VLC: open http://<ip>/mjpeg
 * TCP backpressure naturally prevents SPI overflow — no RTP congestion logic needed.
 */
static esp_err_t mjpeg_stream_handler(httpd_req_t *req)
{
    if (!s_cam.initialized) {
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "Camera not initialized", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

    s_cam.mjpeg_clients++;
    ESP_LOGI(TAG, "MJPEG HTTP client connected (%d active)", s_cam.mjpeg_clients);

    esp_err_t res = ESP_OK;
    char part_hdr[128];

    while (res == ESP_OK) {
        /* Wait for a new JPEG frame from the capture task */
        if (xSemaphoreTake(s_cam.frame_ready, pdMS_TO_TICKS(5000)) != pdTRUE) {
            continue;  /* Timeout — keep connection alive, try again */
        }

        /* Read the latest JPEG frame under mutex */
        int rd_idx;
        size_t jpg_len = 0;
        uint8_t *jpg_data = NULL;

        if (xSemaphoreTake(s_cam.jpeg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            rd_idx = s_cam.jpeg_read_idx;
            jpg_len = s_cam.jpeg_size[rd_idx];
            jpg_data = s_cam.jpeg_buf[rd_idx];
            xSemaphoreGive(s_cam.jpeg_mutex);
        }

        if (!jpg_data || jpg_len == 0) continue;

        /* Send multipart boundary + JPEG content-type header */
        int hdr_len = snprintf(part_hdr, sizeof(part_hdr),
            "%sContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            STREAM_BOUNDARY, (unsigned)jpg_len);

        res = httpd_resp_send_chunk(req, part_hdr, hdr_len);
        if (res != ESP_OK) break;

        /* Send JPEG data — TCP backpressure handles flow control */
        res = httpd_resp_send_chunk(req, (const char *)jpg_data, jpg_len);
    }

    s_cam.mjpeg_clients--;
    ESP_LOGI(TAG, "MJPEG HTTP client disconnected (%d active)", s_cam.mjpeg_clients);
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
"    const tw=parseInt(resp.headers.get('X-Thermal-Width'))||160;"
"    const th=parseInt(resp.headers.get('X-Thermal-Height'))||120;"
"    if(tw!==w||th!==h){"
"      w=tw;h=th;canvas.width=w;canvas.height=h;"
"      canvas.style.width=(w*SCALE)+'px';canvas.style.height=(h*SCALE)+'px';"
"      imgData=ctx.createImageData(w,h);"
"    }"
"    const buf=await resp.arrayBuffer();"
"    const raw=new Uint8Array(buf);"
"    const npix=w*h;"
"    let y=new Uint8Array(npix);"
"    if(raw.length===npix+4){"
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
"    ctx.strokeStyle='rgba(255,255,255,0.7)';ctx.lineWidth=1;"
"    const cx2=w/2,cy2=h/2;"
"    ctx.beginPath();ctx.moveTo(cx2-4,cy2);ctx.lineTo(cx2+4,cy2);"
"    ctx.moveTo(cx2,cy2-4);ctx.lineTo(cx2,cy2+4);ctx.stroke();"
"    frames++;"
"    const now=performance.now();"
"    if(now-lastT>=1000){"
"      const fps=(frames*1000/(now-lastT)).toFixed(1);"
"      info.textContent=w+'x'+h+' Y16 | '+fps+' fps'+tempStr;"
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

    /* Compact format: 4-byte header (vmin_LE16, vmax_LE16) + npix bytes (8-bit normalized) */
    uint16_t vmin = 65535, vmax = 0;
    for (unsigned i = 0; i < npix; i++) {
        if (y16_buf[i] < vmin) vmin = y16_buf[i];
        if (y16_buf[i] > vmax) vmax = y16_buf[i];
    }
    uint16_t rng = (vmax > vmin) ? (vmax - vmin) : 1;

    size_t out_sz = 4 + npix;
    uint8_t *out = heap_caps_malloc(out_sz, MALLOC_CAP_SPIRAM);
    if (!out) { free(y16_buf); return httpd_resp_send(req, NULL, 0); }

    out[0] = vmin & 0xFF; out[1] = (vmin >> 8) & 0xFF;
    out[2] = vmax & 0xFF; out[3] = (vmax >> 8) & 0xFF;

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

static const httpd_uri_t uri_mjpeg = {
    .uri = "/mjpeg",
    .method = HTTP_GET,
    .handler = mjpeg_stream_handler,
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
        httpd_register_uri_handler(server, &uri_mjpeg);
        httpd_register_uri_handler(server, &uri_sdp);
        httpd_register_uri_handler(server, &uri_thermal);
        httpd_register_uri_handler(server, &uri_thermal_raw);
        ESP_LOGI(TAG, "HTTP server started");
#if ENABLE_MJPEG
        ESP_LOGI(TAG, "  Video:   http://<ip>/mjpeg  (HTTP/TCP MJPEG, %dx%d, Q=%d, %dfps)",
                 CAM_WIDTH, CAM_HEIGHT, MJPEG_QUALITY, MJPEG_FPS);
#else
        ESP_LOGI(TAG, "  Video:   H.264 RTP on port %d + http://<ip>/video", RTP_PORT);
        ESP_LOGI(TAG, "  SDP:     http://<ip>/stream.sdp (open in VLC)");
#endif
        ESP_LOGI(TAG, "  Thermal: http://<ip>/thermal  (browser, iron colormap)");
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
