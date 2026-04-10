/*
 * FLIR Lepton 3.x Thermal Camera Driver for ESP32-P4
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * VoSPI (Video over SPI) protocol:
 *   - Lepton is SPI slave, ESP32-P4 is master
 *   - Output-only: master clocks out dummy bytes, reads MISO
 *   - Packet: 2B ID + 2B CRC + 160B payload (80 pixels x 16-bit)
 *   - Lepton 3.x: 4 segments x 60 packets = 1 frame (160x120)
 *   - Segment number encoded in packet 20's ID word bits [6:4]
 *   - Discard packet: ID word = 0x_F__ (bits [11:8] = 0xF)
 *
 * CCI (Command & Control Interface):
 *   - I2C at 7-bit address 0x2A
 *   - Used for: AGC config, radiometry, FFC, telemetry
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "lepton_thermal.h"

static const char *TAG = "lepton";

/* ── VoSPI packet layout ───────────────────────────────── */
#define VOSPI_HEADER_SIZE   4
#define VOSPI_ID_DISCARD    0x0F00  /* Discard packet mask: bits [11:8] */
#define VOSPI_SEG_PACKET    20      /* Segment number in packet 20 */

/* ── Driver state ──────────────────────────────────────── */
static struct {
    /* SPI */
    spi_device_handle_t spi_dev;

    /* I2C */
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_dev;

    /* Frame double buffer */
    uint16_t *frame_buf[2];
    int write_idx;              /* Buffer being written to */
    int read_idx;               /* Buffer available for reading */
    SemaphoreHandle_t frame_ready;

    /* VoSPI packet buffer (DMA-capable) */
    uint8_t *pkt_buf;

    /* Assembly buffer: one full frame being built */
    uint16_t *assembly_buf;

    /* Stats */
    lepton_stats_t stats;
    int64_t last_fps_time;
    uint32_t fps_frame_count;

    bool initialized;
    bool running;
} s_lep;

/* ── SPI Init ──────────────────────────────────────────── */
static esp_err_t lepton_spi_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = -1,              /* Not used (output-only from Lepton) */
        .miso_io_num = LEPTON_SPI_MISO,
        .sclk_io_num = LEPTON_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LEPTON_PACKET_SIZE,
    };

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = LEPTON_SPI_FREQ_HZ,
        .mode = 3,                      /* CPOL=1, CPHA=1 (Lepton requirement) */
        .spics_io_num = LEPTON_SPI_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX, /* Read-only from Lepton */
    };

    ret = spi_bus_add_device(SPI3_HOST, &dev_cfg, &s_lep.spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI3_HOST);
        return ret;
    }

    ESP_LOGI(TAG, "SPI3 initialized: CLK=%d MISO=%d CS=%d @ %dMHz",
             LEPTON_SPI_CLK, LEPTON_SPI_MISO, LEPTON_SPI_CS,
             LEPTON_SPI_FREQ_HZ / 1000000);
    return ESP_OK;
}

/* ── I2C (CCI) Init ───────────────────────────────────── */
static esp_err_t lepton_i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_1,         /* I2C_NUM_0 may be used by camera SCCB */
        .sda_io_num = LEPTON_I2C_SDA,
        .scl_io_num = LEPTON_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_lep.i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LEPTON_I2C_ADDR,
        .scl_speed_hz = LEPTON_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(s_lep.i2c_bus, &dev_cfg, &s_lep.i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C1 (CCI) initialized: SCL=%d SDA=%d @ %dkHz, addr=0x%02X",
             LEPTON_I2C_SCL, LEPTON_I2C_SDA,
             LEPTON_I2C_FREQ_HZ / 1000, LEPTON_I2C_ADDR);
    return ESP_OK;
}

/* ── CCI Register Access ──────────────────────────────── */

/* CCI register addresses */
#define CCI_REG_STATUS      0x0002
#define CCI_REG_COMMAND     0x0004
#define CCI_REG_DATA_LEN    0x0006
#define CCI_REG_DATA_0      0x0008

/* CCI status bits */
#define CCI_STATUS_BUSY     (1 << 0)
#define CCI_STATUS_BOOT_OK  (1 << 2)
#define CCI_STATUS_ERROR    (0xFF00)

static esp_err_t cci_read_reg(uint16_t reg, uint16_t *value)
{
    uint8_t tx[2] = { (reg >> 8) & 0xFF, reg & 0xFF };
    uint8_t rx[2] = {0};

    esp_err_t ret = i2c_master_transmit_receive(s_lep.i2c_dev,
                                                 tx, sizeof(tx),
                                                 rx, sizeof(rx),
                                                 100);
    if (ret == ESP_OK) {
        *value = (rx[0] << 8) | rx[1];
    }
    return ret;
}

static esp_err_t cci_write_reg(uint16_t reg, uint16_t value)
{
    uint8_t tx[4] = {
        (reg >> 8) & 0xFF, reg & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF,
    };
    return i2c_master_transmit(s_lep.i2c_dev, tx, sizeof(tx), 100);
}

static esp_err_t cci_wait_idle(int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        uint16_t status = 0;
        if (cci_read_reg(CCI_REG_STATUS, &status) == ESP_OK) {
            if (!(status & CCI_STATUS_BUSY)) {
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t cci_run_command(uint16_t cmd_id)
{
    esp_err_t ret = cci_wait_idle(1000);
    if (ret != ESP_OK) return ret;

    ret = cci_write_reg(CCI_REG_COMMAND, cmd_id);
    if (ret != ESP_OK) return ret;

    return cci_wait_idle(2000);
}

/* ── CCI Commands ─────────────────────────────────────── */

/* Module IDs */
#define CCI_MOD_AGC     0x0100
#define CCI_MOD_SYS     0x0200
#define CCI_MOD_RAD     0x0E00

/* Command types */
#define CCI_CMD_GET     0x0000
#define CCI_CMD_SET     0x0001
#define CCI_CMD_RUN     0x0002

static esp_err_t lepton_enable_agc(bool enable)
{
    /* AGC Enable: module 0x01, command 0x00, SET */
    esp_err_t ret = cci_wait_idle(1000);
    if (ret != ESP_OK) return ret;

    /* Write data: 1 word */
    cci_write_reg(CCI_REG_DATA_LEN, 2);
    cci_write_reg(CCI_REG_DATA_0, enable ? 1 : 0);

    return cci_run_command(CCI_MOD_AGC | 0x0000 | CCI_CMD_SET);
}

static esp_err_t lepton_enable_radiometry(bool enable)
{
    /* RAD Enable: module 0x0E, command 0x10, SET */
    esp_err_t ret = cci_wait_idle(1000);
    if (ret != ESP_OK) return ret;

    cci_write_reg(CCI_REG_DATA_LEN, 2);
    cci_write_reg(CCI_REG_DATA_0, enable ? 1 : 0);

    return cci_run_command(CCI_MOD_RAD | 0x0010 | CCI_CMD_SET);
}

/* ── VoSPI Frame Capture ──────────────────────────────── */

/**
 * Read one VoSPI packet from the Lepton.
 * Returns the packet ID word (or -1 on error).
 */
static int vospi_read_packet(uint8_t *pkt_buf)
{
    spi_transaction_t trans = {
        .rxlength = LEPTON_PACKET_SIZE * 8,
        .rx_buffer = pkt_buf,
        .flags = 0,
    };

    esp_err_t ret = spi_device_polling_transmit(s_lep.spi_dev, &trans);
    if (ret != ESP_OK) {
        return -1;
    }

    /* ID word: first 2 bytes (big-endian) */
    uint16_t id = (pkt_buf[0] << 8) | pkt_buf[1];
    return (int)id;
}

/**
 * Check if a packet is a discard packet.
 * Discard: ID bits [11:8] == 0xF
 */
static inline bool vospi_is_discard(int id)
{
    return ((id & 0x0F00) == 0x0F00);
}

/**
 * Extract packet number from ID word (bits [5:0])
 */
static inline int vospi_pkt_num(int id)
{
    return id & 0x003F;
}

/**
 * Extract segment number from ID word (bits [6:4] of high byte)
 * Only valid for packet 20.
 */
static inline int vospi_seg_num(int id)
{
    return (id >> 12) & 0x07;
}

/**
 * Capture one complete frame (4 segments x 60 packets).
 * Returns ESP_OK on success, frame data in assembly_buf.
 */
static esp_err_t vospi_capture_frame(void)
{
    int segments_done = 0;
    bool seg_valid[5] = {false};    /* segments 1-4 */
    int sync_retries = 0;
    const int MAX_SYNC_RETRIES = 200;

    while (segments_done < LEPTON_SEGMENTS) {
        /* Read packets for one segment */
        int expected_pkt = 0;
        bool segment_ok = true;
        int current_seg = 0;

        for (int pkt = 0; pkt < LEPTON_PACKETS_PER_SEG; pkt++) {
            int id = vospi_read_packet(s_lep.pkt_buf);
            if (id < 0) {
                segment_ok = false;
                break;
            }

            /* Discard packet: wait and retry */
            if (vospi_is_discard(id)) {
                s_lep.stats.discard_packets++;
                /* Too many discards = need resync */
                if (++sync_retries > MAX_SYNC_RETRIES) {
                    /* De-assert CS for >185ms to force resync */
                    ESP_LOGW(TAG, "VoSPI sync lost, resetting...");
                    s_lep.stats.sync_errors++;
                    vTaskDelay(pdMS_TO_TICKS(200));
                    sync_retries = 0;
                    segments_done = 0;
                    memset(seg_valid, 0, sizeof(seg_valid));
                }
                pkt--;  /* Retry this packet slot */
                continue;
            }
            sync_retries = 0;

            int pkt_num = vospi_pkt_num(id);
            if (pkt_num != expected_pkt) {
                /* Out of order - segment is invalid */
                segment_ok = false;
                break;
            }

            /* Packet 20 carries the segment number */
            if (pkt_num == VOSPI_SEG_PACKET) {
                current_seg = vospi_seg_num(id);
                if (current_seg < 1 || current_seg > 4) {
                    segment_ok = false;
                    break;
                }
            }

            /* Copy pixel data to assembly buffer */
            /* Each packet has 80 pixels (160 bytes payload) */
            /* Segment N covers rows (N-1)*30 to N*30-1 for Lepton 3.x */
            if (current_seg >= 1 && current_seg <= 4) {
                int row_in_seg = pkt_num / 2;   /* 2 packets per row (160 pixels) */
                int half = pkt_num % 2;         /* first or second half of row */
                int global_row = (current_seg - 1) * 30 + row_in_seg;

                if (global_row < LEPTON_HEIGHT) {
                    int dst_offset = global_row * LEPTON_WIDTH + half * 80;
                    const uint8_t *payload = s_lep.pkt_buf + VOSPI_HEADER_SIZE;

                    /* Convert big-endian 16-bit pixels to native */
                    uint16_t *dst = s_lep.assembly_buf + dst_offset;
                    for (int p = 0; p < 80; p++) {
                        dst[p] = (payload[p * 2] << 8) | payload[p * 2 + 1];
                    }
                }
            }

            expected_pkt++;
        }

        if (segment_ok && current_seg >= 1 && current_seg <= 4 && !seg_valid[current_seg]) {
            seg_valid[current_seg] = true;
            segments_done++;
        }
    }

    return ESP_OK;
}

/* ── Capture Task ──────────────────────────────────────── */

static void lepton_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Capture task started on core %d", xPortGetCoreID());

    int64_t t_start, t_end;
    s_lep.last_fps_time = esp_timer_get_time();
    s_lep.fps_frame_count = 0;

    while (s_lep.running) {
        t_start = esp_timer_get_time();

        /* Capture one frame into assembly buffer */
        esp_err_t ret = vospi_capture_frame();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Frame capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Swap double buffer */
        memcpy(s_lep.frame_buf[s_lep.write_idx], s_lep.assembly_buf, LEPTON_FRAME_SIZE);
        s_lep.read_idx = s_lep.write_idx;
        s_lep.write_idx = 1 - s_lep.write_idx;
        xSemaphoreGive(s_lep.frame_ready);

        s_lep.stats.frames_captured++;
        s_lep.fps_frame_count++;

        t_end = esp_timer_get_time();
        float capture_ms = (t_end - t_start) / 1000.0f;
        s_lep.stats.capture_time_ms = capture_ms;

        /* Update FPS every 5 seconds */
        int64_t elapsed = t_end - s_lep.last_fps_time;
        if (elapsed > 5000000) {
            s_lep.stats.fps = s_lep.fps_frame_count * 1000000.0f / elapsed;
            ESP_LOGI(TAG, "Thermal: %.1f FPS, capture=%.0fms, frames=%lu",
                     s_lep.stats.fps, capture_ms,
                     (unsigned long)s_lep.stats.frames_captured);
            s_lep.fps_frame_count = 0;
            s_lep.last_fps_time = t_end;
        }
    }

    ESP_LOGI(TAG, "Capture task stopped");
    vTaskDelete(NULL);
}

/* ── Public API ────────────────────────────────────────── */

esp_err_t lepton_init(void)
{
    if (s_lep.initialized) {
        return ESP_OK;
    }

    memset(&s_lep, 0, sizeof(s_lep));

    ESP_LOGI(TAG, "Initializing FLIR Lepton 3.x thermal camera...");

    /* EN pin: power on Lepton */
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << LEPTON_EN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&en_cfg);
    gpio_set_level(LEPTON_EN_PIN, 1);
    ESP_LOGI(TAG, "EN pin %d set HIGH", LEPTON_EN_PIN);

    /* Wait for Lepton boot (~950ms from power-on) */
    ESP_LOGI(TAG, "Waiting for Lepton boot (1.5s)...");
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* Initialize SPI (VoSPI) */
    esp_err_t ret = lepton_spi_init();
    if (ret != ESP_OK) return ret;

    /* Initialize I2C (CCI) */
    ret = lepton_i2c_init();
    if (ret != ESP_OK) return ret;

    /* Check Lepton booted OK via CCI */
    uint16_t status = 0;
    ret = cci_read_reg(CCI_REG_STATUS, &status);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CCI status: 0x%04X (boot_ok=%d, busy=%d)",
                 status,
                 (status & CCI_STATUS_BOOT_OK) ? 1 : 0,
                 (status & CCI_STATUS_BUSY) ? 1 : 0);
    } else {
        ESP_LOGW(TAG, "CCI status read failed (Lepton may still be booting)");
    }

    /* Enable AGC (for better visual output) */
    lepton_enable_agc(true);

    /* Enable radiometry (14-bit calibrated output) */
    lepton_enable_radiometry(true);

    /* Allocate frame buffers in PSRAM */
    for (int i = 0; i < 2; i++) {
        s_lep.frame_buf[i] = heap_caps_calloc(LEPTON_PIXELS, sizeof(uint16_t),
                                               MALLOC_CAP_SPIRAM);
        if (!s_lep.frame_buf[i]) {
            ESP_LOGE(TAG, "Frame buffer %d alloc failed", i);
            return ESP_ERR_NO_MEM;
        }
    }

    s_lep.assembly_buf = heap_caps_calloc(LEPTON_PIXELS, sizeof(uint16_t),
                                           MALLOC_CAP_SPIRAM);
    if (!s_lep.assembly_buf) {
        ESP_LOGE(TAG, "Assembly buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* VoSPI packet buffer - must be DMA-capable (internal RAM) */
    s_lep.pkt_buf = heap_caps_calloc(1, LEPTON_PACKET_SIZE, MALLOC_CAP_DMA);
    if (!s_lep.pkt_buf) {
        ESP_LOGE(TAG, "Packet buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    s_lep.frame_ready = xSemaphoreCreateBinary();
    s_lep.write_idx = 0;
    s_lep.read_idx = -1;
    s_lep.initialized = true;

    ESP_LOGI(TAG, "Lepton initialized (frame=%dx%d, %d bytes/frame)",
             LEPTON_WIDTH, LEPTON_HEIGHT, LEPTON_FRAME_SIZE);
    return ESP_OK;
}

esp_err_t lepton_start(void)
{
    if (!s_lep.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_lep.running) {
        return ESP_OK;
    }

    s_lep.running = true;

    BaseType_t ret = xTaskCreatePinnedToCore(
        lepton_capture_task,
        "lepton_cap",
        4096,
        NULL,
        5,                  /* Priority */
        NULL,
        0                   /* Core 0 (core 1 reserved for future use) */
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Capture task creation failed");
        s_lep.running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Lepton capture started");
    return ESP_OK;
}

esp_err_t lepton_get_frame(const uint16_t **frame_out, size_t *frame_len)
{
    if (!s_lep.initialized || s_lep.read_idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Wait for next frame with timeout */
    if (xSemaphoreTake(s_lep.frame_ready, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *frame_out = s_lep.frame_buf[s_lep.read_idx];
    *frame_len = LEPTON_FRAME_SIZE;
    return ESP_OK;
}

void lepton_get_stats(lepton_stats_t *stats)
{
    if (stats) {
        *stats = s_lep.stats;
    }
}

float lepton_get_fps(void)
{
    return s_lep.stats.fps;
}

void lepton_agc_linear(const uint16_t *raw16, uint8_t *gray8, size_t len)
{
    /* Find min/max for linear stretch */
    uint16_t vmin = UINT16_MAX, vmax = 0;
    for (size_t i = 0; i < len; i++) {
        if (raw16[i] < vmin) vmin = raw16[i];
        if (raw16[i] > vmax) vmax = raw16[i];
    }

    uint16_t range = vmax - vmin;
    if (range == 0) range = 1;

    for (size_t i = 0; i < len; i++) {
        gray8[i] = (uint8_t)(((uint32_t)(raw16[i] - vmin) * 255) / range);
    }
}
