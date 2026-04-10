/*
 * Thermal Camera — FLIR Lepton 3.5 via SPI (VoSPI) + I2C (CCI)
 *
 * PureThermal Breakout Board (GroupGets) connected to ESP32-P4:
 *   VoSPI: SPI3_HOST dedicated (SCLK=22, MISO=30, CS=31) — 20MHz, Mode 3
 *   CCI:   I2C0 shared with IMU/MAG (address 0x2A)
 *   EN:    GPIO 28 — Active HIGH, controls Lepton module power
 *
 * Lepton 3.5: 160x120 @ ~9fps, Grey14 (2 bytes/pixel)
 * VoSPI frame = 4 segments x 60 packets x 164 bytes = 39,360 bytes
 *
 * Architecture:
 *   - sensor_agent (Core 0) creates I2C bus, signals ready via i2c_sync
 *   - thermal_camera (Core 1) creates dedicated SPI bus, reuses I2C for CCI
 *   - No SPI bus sharing — Lepton has exclusive SPI3_HOST access
 *
 * References:
 *   - GroupGets PureThermal Breakout Board Datasheet v1.3
 *   - FLIR Lepton Engineering Datasheet (VoSPI spec)
 *   - Lepton max SPI clock: 20MHz, MOSI not used (ground on breakout)
 */

#include "thermal_camera.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "../common/board_config.h"
#include "../common/i2c_sync.h"

static const char *TAG = "lepton";

/* ── Lepton VoSPI constants ──────────────────────────────────────── */
#define LEP_WIDTH           160
#define LEP_HEIGHT          120
#define LEP_BPP             2       /* Grey14: 2 bytes/pixel */
#define LEP_PKT_HEADER      4       /* 2 bytes ID + 2 bytes CRC */
#define LEP_PKT_DATA       160      /* 80 pixels x 2 bytes (Grey14) */
#define LEP_PKT_SIZE        (LEP_PKT_HEADER + LEP_PKT_DATA)  /* 164 */
#define LEP_PKTS_PER_SEG    60
#define LEP_SEGS_PER_FRAME   4
#define LEP_FRAME_PKTS      (LEP_PKTS_PER_SEG * LEP_SEGS_PER_FRAME)  /* 240 */
#define LEP_FRAME_SIZE      (LEP_WIDTH * LEP_HEIGHT * LEP_BPP)  /* 38400 */

#define LEP_RESYNC_MS       185     /* CS HIGH duration for resync */
#define LEP_BOOT_WAIT_MS    950     /* Wait after EN assertion before I2C */

/* Lepton I2C (CCI) registers */
#define LEP_REG_STATUS      0x0002
#define LEP_REG_COMMAND_ID  0x0004
#define LEP_REG_DATA_LEN    0x0006
#define LEP_REG_DATA0       0x0008

/* ── Module state ────────────────────────────────────────────────── */
static struct {
    bool initialized;
    bool active;

    spi_device_handle_t spi_dev;

    /* Double buffer in PSRAM */
    uint16_t *frame_buf;        /* Latest complete frame (Y16) */
    SemaphoreHandle_t frame_mutex;
    bool frame_valid;

    /* VoSPI read buffer (full frame assembly) */
    uint8_t *vospi_buf;

    /* Stats */
    uint32_t frame_count;
    uint32_t discard_count;
    uint32_t resync_count;

    /* Task handle */
    TaskHandle_t task_handle;
} s_lep = {0};

/* ── EN pin control ──────────────────────────────────────────────── */

static void lepton_en_init(void)
{
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << PIN_LEPTON_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&en_cfg);
    gpio_set_level(PIN_LEPTON_EN, 0);  /* Start with Lepton off */
}

static void lepton_en_set(bool enable)
{
    gpio_set_level(PIN_LEPTON_EN, enable ? 1 : 0);
}

static void lepton_hard_reset(void)
{
    ESP_LOGI(TAG, "Hard reset via EN pin");
    lepton_en_set(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    lepton_en_set(true);
    vTaskDelay(pdMS_TO_TICKS(LEP_BOOT_WAIT_MS));
}

/* ── I2C CCI helpers ─────────────────────────────────────────────── */

static esp_err_t lep_i2c_write_reg16(uint16_t reg, uint16_t val)
{
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_master_get_bus_handle(I2C_PORT, &bus);
    if (err != ESP_OK || bus == NULL) return ESP_FAIL;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LEPTON_I2C_ADDR,
        .scl_speed_hz = I2C_LEPTON_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev = NULL;
    err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) return err;

    uint8_t buf[4] = {
        (reg >> 8) & 0xFF, reg & 0xFF,
        (val >> 8) & 0xFF, val & 0xFF,
    };
    err = i2c_master_transmit(dev, buf, 4, 100);
    i2c_master_bus_rm_device(dev);
    return err;
}

static esp_err_t lep_i2c_read_reg16(uint16_t reg, uint16_t *val)
{
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_master_get_bus_handle(I2C_PORT, &bus);
    if (err != ESP_OK || bus == NULL) return ESP_FAIL;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LEPTON_I2C_ADDR,
        .scl_speed_hz = I2C_LEPTON_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev = NULL;
    err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) return err;

    uint8_t reg_buf[2] = { (reg >> 8) & 0xFF, reg & 0xFF };
    uint8_t data_buf[2] = {0};
    err = i2c_master_transmit_receive(dev, reg_buf, 2, data_buf, 2, 100);
    if (err == ESP_OK) {
        *val = ((uint16_t)data_buf[0] << 8) | data_buf[1];
    }
    i2c_master_bus_rm_device(dev);
    return err;
}

static bool lep_wait_busy(void)
{
    for (int i = 0; i < 50; i++) {
        uint16_t status = 0;
        if (lep_i2c_read_reg16(LEP_REG_STATUS, &status) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (!(status & 0x0001)) {  /* Not busy */
            return (status >> 8) == 0;  /* No error */
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "CCI busy timeout");
    return false;
}

static bool lep_cci_command(uint16_t command_id)
{
    if (lep_i2c_write_reg16(LEP_REG_COMMAND_ID, command_id) != ESP_OK) return false;
    return lep_wait_busy();
}

static bool lep_check_boot(void)
{
    uint16_t status = 0;
    if (lep_i2c_read_reg16(LEP_REG_STATUS, &status) != ESP_OK) return false;

    /* Bit 2: boot status, Bit 1: boot mode, Bit 0: busy */
    if ((status & 0x0004) && !(status & 0x0001)) {
        ESP_LOGI(TAG, "Lepton booted (status=0x%04X)", status);
        return true;
    }
    return false;
}

/* ── SPI bus initialization (dedicated for Lepton VoSPI) ──────────── */

static esp_err_t init_lepton_spi_bus(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = -1,              /* VoSPI is read-only — MOSI not used */
        .miso_io_num = PIN_LEPTON_SPI_MISO,
        .sclk_io_num = PIN_LEPTON_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LEP_PKT_SIZE,  /* 164 bytes per VoSPI packet */
    };

    esp_err_t err = spi_bus_initialize(LEPTON_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Lepton SPI bus initialized (SCLK=%d, MISO=%d, dedicated)",
             PIN_LEPTON_SPI_SCLK, PIN_LEPTON_SPI_MISO);
    return ESP_OK;
}

/* ── VoSPI frame reader task ─────────────────────────────────────── */

static void lepton_vospi_task(void *arg)
{
    /* Working buffer for one VoSPI packet (DMA-capable, internal RAM) */
    uint8_t *pkt = heap_caps_malloc(LEP_PKT_SIZE, MALLOC_CAP_DMA);
    if (!pkt) {
        ESP_LOGE(TAG, "Failed to allocate VoSPI packet buffer");
        vTaskDelete(NULL);
        return;
    }
    spi_transaction_t trans = {
        .length = LEP_PKT_SIZE * 8,
        .rxlength = 0,              /* 0 = same as length */
        .rx_buffer = pkt,
        .tx_buffer = NULL,
    };

    bool synced = false;
    int64_t last_log_us = 0;

    ESP_LOGI(TAG, "VoSPI task started — reading 160x120 Grey14 @~9fps");

    while (1) {
        /* Acquire SPI bus for the entire frame.
         * VoSPI requires uninterrupted CS-low during all 240 packets.
         * With dedicated bus this always succeeds immediately, but we keep
         * the acquire/release pattern for spi_device_polling_transmit perf. */
        esp_err_t acq = spi_device_acquire_bus(s_lep.spi_dev, portMAX_DELAY);
        if (acq != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Assemble one frame: 4 segments x 60 packets */
        bool frame_ok = true;

        for (int seg = 1; seg <= LEP_SEGS_PER_FRAME && frame_ok; seg++) {
            bool discard_seg = false;

            for (int pkt_num = 0; pkt_num < LEP_PKTS_PER_SEG && frame_ok; ) {
                esp_err_t err = spi_device_polling_transmit(s_lep.spi_dev, &trans);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(err));
                    frame_ok = false;
                    break;
                }

                uint16_t id = ((uint16_t)pkt[0] << 8) | pkt[1];

                /* Discard packet: (id >> 8) & 0x0f == 0x0f */
                if (((id >> 8) & 0x0F) == 0x0F) {
                    s_lep.discard_count++;
                    if (pkt_num == 0 && seg == 1) {
                        /* No frame in progress — wait and retry */
                        frame_ok = false;
                    }
                    continue;
                }

                uint16_t packet_num = id & 0x0FFF;
                uint8_t ttt = (id >> 12) & 0x07;

                if (packet_num != pkt_num) {
                    ESP_LOGD(TAG, "Packet mismatch: got %d expect %d (seg %d)", packet_num, pkt_num, seg);
                    frame_ok = false;
                    synced = false;
                    break;
                }

                if (pkt_num == 20) {
                    if (ttt == 0) {
                        discard_seg = true;
                    } else if (ttt != seg) {
                        ESP_LOGD(TAG, "Segment mismatch: ttt=%d expect=%d", ttt, seg);
                        frame_ok = false;
                        synced = false;
                        break;
                    }
                }

                size_t offset = ((seg - 1) * LEP_PKTS_PER_SEG + pkt_num) * LEP_PKT_DATA;
                memcpy(s_lep.vospi_buf + offset, pkt + LEP_PKT_HEADER, LEP_PKT_DATA);

                pkt_num++;
            }

            if (discard_seg) {
                seg--;
            }
        }

        /* Release SPI bus */
        spi_device_release_bus(s_lep.spi_dev);

        if (!frame_ok) {
            if (!synced) {
                /* Resync: CS is already HIGH (bus released).
                 * Wait >=185ms per VoSPI spec for Lepton to reset sync. */
                s_lep.resync_count++;
                ESP_LOGD(TAG, "VoSPI resync #%lu", (unsigned long)s_lep.resync_count);
                vTaskDelay(pdMS_TO_TICKS(LEP_RESYNC_MS));
                synced = true;
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            continue;
        }

        /* Frame complete — copy to double buffer */
        synced = true;
        s_lep.frame_count++;

        if (xSemaphoreTake(s_lep.frame_mutex, 0) == pdTRUE) {
            memcpy(s_lep.frame_buf, s_lep.vospi_buf, LEP_FRAME_SIZE);
            s_lep.frame_valid = true;
            xSemaphoreGive(s_lep.frame_mutex);
        }

        /* Brief yield between frames */
        vTaskDelay(pdMS_TO_TICKS(2));

        /* Periodic stats */
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_log_us > 10000000) {
            ESP_LOGI(TAG, "Lepton: %lu frames, %lu discards, %lu resyncs",
                     (unsigned long)s_lep.frame_count,
                     (unsigned long)s_lep.discard_count,
                     (unsigned long)s_lep.resync_count);
            last_log_us = now_us;
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────── */

esp_err_t thermal_camera_init(thermal_frame_cb_t frame_cb, void *user_ctx)
{
    if (s_lep.initialized) return ESP_OK;

    ESP_LOGI(TAG, "Initializing FLIR Lepton 3.5 (PureThermal Breakout Board)");
    ESP_LOGI(TAG, "  SPI: SCLK=%d, MISO=%d, CS=%d (%dMHz)",
             PIN_LEPTON_SPI_SCLK, PIN_LEPTON_SPI_MISO, PIN_LEPTON_CS,
             LEPTON_SPI_FREQ / 1000000);
    ESP_LOGI(TAG, "  I2C: addr=0x%02X (%dkHz)", LEPTON_I2C_ADDR,
             I2C_LEPTON_FREQ_HZ / 1000);
    ESP_LOGI(TAG, "  EN: GPIO%d", PIN_LEPTON_EN);

    /* Initialize EN pin and power on Lepton */
    lepton_en_init();
    lepton_en_set(true);
    ESP_LOGI(TAG, "EN pin HIGH — waiting %dms for Lepton boot...", LEP_BOOT_WAIT_MS);
    vTaskDelay(pdMS_TO_TICKS(LEP_BOOT_WAIT_MS));

    /* Wait for sensor_agent to finish I2C bus creation.
     * We share I2C0 for Lepton CCI commands. */
    i2c_sync_wait_sensors();

    /* Initialize dedicated SPI bus for VoSPI (no MOSI needed) */
    esp_err_t ret = init_lepton_spi_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed");
        return ret;
    }

    /* Add Lepton as SPI device — Mode 3 per VoSPI spec */
    spi_device_interface_config_t spi_cfg = {
        .clock_speed_hz = LEPTON_SPI_FREQ,
        .mode = 3,  /* CPOL=1, CPHA=1 */
        .spics_io_num = PIN_LEPTON_CS,
        .queue_size = 1,
        .flags = 0,  /* Full-duplex: Lepton ignores MOSI, reads MISO */
    };
    ret = spi_bus_add_device(LEPTON_SPI_HOST, &spi_cfg, &s_lep.spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPI device added (%dMHz, Mode 3, dedicated bus)",
             LEPTON_SPI_FREQ / 1000000);

    /* Check Lepton boot status via I2C CCI */
    bool booted = false;
    for (int i = 0; i < 10; i++) {
        if (lep_check_boot()) {
            booted = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (!booted) {
        ESP_LOGW(TAG, "Lepton did not report boot — trying hard reset...");
        lepton_hard_reset();
        for (int i = 0; i < 10; i++) {
            if (lep_check_boot()) {
                booted = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!booted) {
            ESP_LOGW(TAG, "Lepton still not booted — continuing (may resync via VoSPI)");
        }
    }

    /* Allocate buffers */
    s_lep.frame_mutex = xSemaphoreCreateMutex();
    s_lep.frame_buf = heap_caps_calloc(1, LEP_FRAME_SIZE, MALLOC_CAP_SPIRAM);
    s_lep.vospi_buf = heap_caps_calloc(1, LEP_FRAME_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_lep.frame_buf || !s_lep.vospi_buf || !s_lep.frame_mutex) {
        ESP_LOGE(TAG, "Failed to allocate frame buffers");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Frame buffers: 2 x %dKB (PSRAM)", LEP_FRAME_SIZE / 1024);

    /* Start VoSPI reader task — high priority to maintain sync */
    xTaskCreatePinnedToCore(lepton_vospi_task, "lepton", 4096, NULL, 6, &s_lep.task_handle, 1);

    s_lep.initialized = true;
    ESP_LOGI(TAG, "Lepton 3.5 initialized (160x120 Grey14, ~9fps)");
    return ESP_OK;
}

esp_err_t thermal_camera_start(void)
{
    if (!s_lep.initialized) return ESP_ERR_INVALID_STATE;
    s_lep.active = true;
    ESP_LOGI(TAG, "Thermal streaming started");
    return ESP_OK;
}

esp_err_t thermal_camera_stop(void)
{
    if (!s_lep.initialized) return ESP_ERR_INVALID_STATE;
    s_lep.active = false;
    return ESP_OK;
}

bool thermal_camera_get_frame(uint16_t *buf)
{
    if (!s_lep.frame_valid || !s_lep.frame_mutex) return false;

    if (xSemaphoreTake(s_lep.frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_lep.frame_valid) {
            memcpy(buf, s_lep.frame_buf, LEP_FRAME_SIZE);
            xSemaphoreGive(s_lep.frame_mutex);
            return true;
        }
        xSemaphoreGive(s_lep.frame_mutex);
    }
    return false;
}

bool thermal_camera_get_jpeg(uint8_t *buf, size_t buf_size, size_t *out_len)
{
    /* Lepton outputs raw Y16, not JPEG — this API not applicable */
    (void)buf; (void)buf_size; (void)out_len;
    return false;
}

bool thermal_camera_is_active(void)
{
    return s_lep.active;
}

unsigned thermal_camera_width(void)
{
    return LEP_WIDTH;
}

unsigned thermal_camera_height(void)
{
    return LEP_HEIGHT;
}

size_t thermal_camera_frame_size(void)
{
    return LEP_FRAME_SIZE;
}
