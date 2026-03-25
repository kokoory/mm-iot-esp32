/*
 * SBUS RC Receiver Driver Implementation
 *
 * SBUS uses inverted UART at 100kbps, 8E2 (8 data bits, even parity, 2 stop bits).
 * ESP32-P4 UART supports signal inversion in hardware via uart_set_line_inverse().
 *
 * Frame format (25 bytes):
 *   Byte 0:     Header (0x0F)
 *   Bytes 1-22: 16 channels x 11 bits = 176 bits packed LSB-first
 *   Byte 23:    Flags (bit2=frame_lost, bit3=failsafe)
 *   Byte 24:    Footer (0x00)
 */

#include "sbus.h"
#include "../common/board_config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "sbus";

#define SBUS_HEADER     0x0F
#define SBUS_FOOTER     0x00
#define SBUS_RX_BUF     256

/* Latest decoded data */
static sbus_data_t s_data;
static bool s_new_data = false;
static TaskHandle_t s_task_handle = NULL;

/* ── Frame decoder ───────────────────────────────────────────── */

static void sbus_decode_frame(const uint8_t *frame, sbus_data_t *out)
{
    /* 16 channels x 11 bits, packed LSB-first starting at byte 1 */
    out->channels[0]  = ((uint16_t)frame[1]       | (uint16_t)frame[2]  << 8) & 0x07FF;
    out->channels[1]  = ((uint16_t)frame[2]  >> 3  | (uint16_t)frame[3]  << 5) & 0x07FF;
    out->channels[2]  = ((uint16_t)frame[3]  >> 6  | (uint16_t)frame[4]  << 2
                                                    | (uint16_t)frame[5]  << 10) & 0x07FF;
    out->channels[3]  = ((uint16_t)frame[5]  >> 1  | (uint16_t)frame[6]  << 7) & 0x07FF;
    out->channels[4]  = ((uint16_t)frame[6]  >> 4  | (uint16_t)frame[7]  << 4) & 0x07FF;
    out->channels[5]  = ((uint16_t)frame[7]  >> 7  | (uint16_t)frame[8]  << 1
                                                    | (uint16_t)frame[9]  << 9) & 0x07FF;
    out->channels[6]  = ((uint16_t)frame[9]  >> 2  | (uint16_t)frame[10] << 6) & 0x07FF;
    out->channels[7]  = ((uint16_t)frame[10] >> 5  | (uint16_t)frame[11] << 3) & 0x07FF;
    out->channels[8]  = ((uint16_t)frame[12]       | (uint16_t)frame[13] << 8) & 0x07FF;
    out->channels[9]  = ((uint16_t)frame[13] >> 3  | (uint16_t)frame[14] << 5) & 0x07FF;
    out->channels[10] = ((uint16_t)frame[14] >> 6  | (uint16_t)frame[15] << 2
                                                    | (uint16_t)frame[16] << 10) & 0x07FF;
    out->channels[11] = ((uint16_t)frame[16] >> 1  | (uint16_t)frame[17] << 7) & 0x07FF;
    out->channels[12] = ((uint16_t)frame[17] >> 4  | (uint16_t)frame[18] << 4) & 0x07FF;
    out->channels[13] = ((uint16_t)frame[18] >> 7  | (uint16_t)frame[19] << 1
                                                    | (uint16_t)frame[20] << 9) & 0x07FF;
    out->channels[14] = ((uint16_t)frame[20] >> 2  | (uint16_t)frame[21] << 6) & 0x07FF;
    out->channels[15] = ((uint16_t)frame[21] >> 5  | (uint16_t)frame[22] << 3) & 0x07FF;

    out->frame_lost = (frame[23] & 0x04) != 0;
    out->failsafe   = (frame[23] & 0x08) != 0;
    out->last_frame_us = (uint64_t)esp_timer_get_time();
}

/* ── UART reader task ────────────────────────────────────────── */

static void sbus_reader_task(void *param)
{
    (void)param;
    uint8_t buf[SBUS_RX_BUF];
    uint8_t frame[SBUS_FRAME_SIZE];
    int frame_idx = 0;
    bool synced = false;

    ESP_LOGI(TAG, "SBUS reader task started");

    while (1) {
        int len = uart_read_bytes(SBUS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            if (!synced) {
                /* Look for header byte after a gap */
                if (buf[i] == SBUS_HEADER) {
                    frame[0] = buf[i];
                    frame_idx = 1;
                    synced = true;
                }
                continue;
            }

            frame[frame_idx++] = buf[i];

            if (frame_idx >= SBUS_FRAME_SIZE) {
                /* Validate: header = 0x0F, footer = 0x00 */
                if (frame[0] == SBUS_HEADER && frame[24] == SBUS_FOOTER) {
                    sbus_data_t decoded;
                    sbus_decode_frame(frame, &decoded);
                    s_data = decoded;
                    s_new_data = true;
                } else {
                    /* Lost sync, re-sync */
                    synced = false;
                }
                frame_idx = 0;
                synced = false;
            }
        }
    }
}

/* ── Public API ──────────────────────────────────────────────── */

int sbus_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = SBUS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(SBUS_UART_NUM, SBUS_RX_BUF * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return -1;
    }

    err = uart_param_config(SBUS_UART_NUM, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* SBUS uses RX only, TX pin = -1 */
    err = uart_set_pin(SBUS_UART_NUM, UART_PIN_NO_CHANGE, PIN_SBUS_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* SBUS signal is inverted */
    err = uart_set_line_inverse(SBUS_UART_NUM, UART_SIGNAL_RXD_INV);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set inverse failed: %s", esp_err_to_name(err));
        return -1;
    }

    memset(&s_data, 0, sizeof(s_data));

    /* Start reader task on FC core */
    xTaskCreatePinnedToCore(sbus_reader_task, "sbus_rx", 4096, NULL,
                            SENSOR_TASK_PRIORITY - 1, &s_task_handle, FC_CORE);

    ESP_LOGI(TAG, "SBUS initialized on GPIO %d (UART%d, inverted)",
             PIN_SBUS_RX, SBUS_UART_NUM);
    return 0;
}

int sbus_read(sbus_data_t *data)
{
    if (!s_new_data) return -1;
    *data = s_data;
    s_new_data = false;
    return 0;
}

float sbus_channel_to_float(uint16_t raw)
{
    /* SBUS range: 172-1811, center ~992 */
    float normalized = ((float)raw - 992.0f) / 820.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    if (normalized >  1.0f) normalized =  1.0f;
    return normalized;
}
