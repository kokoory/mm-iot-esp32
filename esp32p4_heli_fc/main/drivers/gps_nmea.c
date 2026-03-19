/*
 * GPS NMEA-0183 UART Parser
 *
 * Runs a dedicated FreeRTOS task that reads UART byte-by-byte,
 * assembles complete sentences, validates checksums, and parses
 * GPRMC / GPGGA fields into a sensor_gps_t structure.
 */

#include "gps_nmea.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "gps_nmea";

/* ------------------------------------------------------------------ */
/* NMEA helper functions                                               */
/* ------------------------------------------------------------------ */

/**
 * Advance *p past the next comma in the string.
 * Returns pointer to field start (after comma), or NULL if end of string.
 */
static const char *nmea_next_field(const char *p)
{
    if (!p) return NULL;
    const char *c = strchr(p, ',');
    return c ? c + 1 : NULL;
}

/**
 * Parse a floating-point number from the current field position.
 * Returns 0.0 if field is empty.
 */
static float nmea_parse_float(const char *p)
{
    if (!p || *p == ',' || *p == '*' || *p == '\0') {
        return 0.0f;
    }
    return strtof(p, NULL);
}

/**
 * Parse a double from the current field position.
 */
static double nmea_parse_double(const char *p)
{
    if (!p || *p == ',' || *p == '*' || *p == '\0') {
        return 0.0;
    }
    return strtod(p, NULL);
}

/**
 * Parse an integer from the current field position.
 */
static int nmea_parse_int(const char *p)
{
    if (!p || *p == ',' || *p == '*' || *p == '\0') {
        return 0;
    }
    return (int)strtol(p, NULL, 10);
}

/**
 * Parse NMEA coordinate (DDMM.MMMMM or DDDMM.MMMMM) into decimal degrees.
 *   raw    = the numeric value (e.g. 4807.038)
 *   dir    = 'N'/'S' or 'E'/'W'
 * Returns decimal degrees (negative for S/W).
 */
static double nmea_parse_coord(double raw, char dir)
{
    if (raw == 0.0) return 0.0;

    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = (double)degrees + minutes / 60.0;

    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

/**
 * Validate NMEA checksum.
 * The sentence must start with '$' and contain '*XX' before the end.
 * Returns true if valid.
 */
static bool nmea_validate_checksum(const char *sentence)
{
    if (!sentence || sentence[0] != '$') {
        return false;
    }

    const char *p = sentence + 1;   /* skip '$' */
    uint8_t calc_cksum = 0;

    while (*p && *p != '*') {
        calc_cksum ^= (uint8_t)*p;
        p++;
    }

    if (*p != '*') {
        return false;   /* no checksum field found */
    }

    /* Parse the two hex digits after '*' */
    uint8_t msg_cksum = (uint8_t)strtol(p + 1, NULL, 16);
    return (calc_cksum == msg_cksum);
}

/* ------------------------------------------------------------------ */
/* Sentence parsers                                                    */
/* ------------------------------------------------------------------ */

/**
 * Parse $GPRMC / $GNRMC sentence:
 *   $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,spd,crs,ddmmyy,mv,mvd,mode*cs
 *
 * Fields:
 *   1: UTC time
 *   2: Status A=active V=void
 *   3: Latitude
 *   4: N/S
 *   5: Longitude
 *   6: E/W
 *   7: Speed over ground (knots)
 *   8: Course over ground (degrees true)
 *   9: Date
 */
static void parse_gprmc(const char *sentence, sensor_gps_t *data)
{
    const char *p = sentence;

    /* Skip sentence ID ("$GPRMC," or "$GNRMC,") */
    p = nmea_next_field(p);     /* -> UTC time */
    if (!p) return;
    /* float utc = nmea_parse_float(p); -- not stored */

    p = nmea_next_field(p);     /* -> Status */
    if (!p) return;
    char status = *p;

    p = nmea_next_field(p);     /* -> Latitude */
    if (!p) return;
    double raw_lat = nmea_parse_double(p);

    p = nmea_next_field(p);     /* -> N/S */
    if (!p) return;
    char ns = *p;

    p = nmea_next_field(p);     /* -> Longitude */
    if (!p) return;
    double raw_lon = nmea_parse_double(p);

    p = nmea_next_field(p);     /* -> E/W */
    if (!p) return;
    char ew = *p;

    p = nmea_next_field(p);     /* -> Speed (knots) */
    if (!p) return;
    float speed_kts = nmea_parse_float(p);

    p = nmea_next_field(p);     /* -> Course */
    if (!p) return;
    float course = nmea_parse_float(p);

    /* Only update if status is Active */
    if (status == 'A') {
        data->latitude    = nmea_parse_coord(raw_lat, ns);
        data->longitude   = nmea_parse_coord(raw_lon, ew);
        data->ground_speed = speed_kts * 0.514444f;  /* knots -> m/s */
        data->course      = course;
    }
}

/**
 * Parse $GPGGA / $GNGGA sentence:
 *   $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,q,nn,hdop,alt,M,sep,M,age,refid*cs
 *
 * Fields:
 *   1: UTC time
 *   2: Latitude
 *   3: N/S
 *   4: Longitude
 *   5: E/W
 *   6: Fix quality (0=invalid, 1=GPS, 2=DGPS, ...)
 *   7: Number of satellites
 *   8: HDOP
 *   9: Altitude above MSL (meters)
 *   10: M
 *   11: Geoid separation
 */
static void parse_gpgga(const char *sentence, sensor_gps_t *data)
{
    const char *p = sentence;

    p = nmea_next_field(p);     /* -> UTC time */
    if (!p) return;

    p = nmea_next_field(p);     /* -> Latitude */
    if (!p) return;
    double raw_lat = nmea_parse_double(p);

    p = nmea_next_field(p);     /* -> N/S */
    if (!p) return;
    char ns = *p;

    p = nmea_next_field(p);     /* -> Longitude */
    if (!p) return;
    double raw_lon = nmea_parse_double(p);

    p = nmea_next_field(p);     /* -> E/W */
    if (!p) return;
    char ew = *p;

    p = nmea_next_field(p);     /* -> Fix quality */
    if (!p) return;
    int fix_quality = nmea_parse_int(p);

    p = nmea_next_field(p);     /* -> Satellites */
    if (!p) return;
    int sats = nmea_parse_int(p);

    p = nmea_next_field(p);     /* -> HDOP */
    if (!p) return;
    float hdop = nmea_parse_float(p);

    p = nmea_next_field(p);     /* -> Altitude MSL */
    if (!p) return;
    float alt_msl = nmea_parse_float(p);

    /* Map fix quality to fix_type */
    if (fix_quality >= 1) {
        data->fix_type = 3;     /* 3D fix (simplification: GPS fix = 3D) */
    } else {
        data->fix_type = 0;
    }

    data->satellites   = (uint8_t)sats;
    data->hdop         = hdop;
    data->altitude_msl = alt_msl;

    /* Also update lat/lon from GGA if fix is valid */
    if (fix_quality >= 1 && raw_lat != 0.0) {
        data->latitude  = nmea_parse_coord(raw_lat, ns);
        data->longitude = nmea_parse_coord(raw_lon, ew);
    }
}

/* ------------------------------------------------------------------ */
/* NMEA line dispatcher                                                */
/* ------------------------------------------------------------------ */

static void nmea_process_sentence(const char *sentence, sensor_gps_t *data)
{
    if (!nmea_validate_checksum(sentence)) {
        return;     /* silently discard bad checksum */
    }

    data->timestamp_us = (uint64_t)esp_timer_get_time();

    if (strncmp(sentence, "$GPRMC,", 7) == 0 ||
        strncmp(sentence, "$GNRMC,", 7) == 0) {
        parse_gprmc(sentence, data);
    } else if (strncmp(sentence, "$GPGGA,", 7) == 0 ||
               strncmp(sentence, "$GNGGA,", 7) == 0) {
        parse_gpgga(sentence, data);
    }
    /* Other sentence types are silently ignored */
}

/* ------------------------------------------------------------------ */
/* UART reader task                                                    */
/* ------------------------------------------------------------------ */

static void gps_task(void *param)
{
    gps_handle_t *gps = (gps_handle_t *)param;
    char line[NMEA_MAX_LEN];
    int  idx = 0;

    ESP_LOGI(TAG, "GPS parser task started on UART%d", gps->uart_num);

    for (;;) {
        uint8_t byte;
        int len = uart_read_bytes(gps->uart_num, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;   /* timeout – loop back */
        }

        if (byte == '$') {
            /* Start of new sentence */
            idx = 0;
            line[idx++] = '$';
        } else if (byte == '\r' || byte == '\n') {
            if (idx > 5) {
                /* End of sentence – null-terminate and process */
                line[idx] = '\0';

                sensor_gps_t tmp;
                xSemaphoreTake(gps->mutex, portMAX_DELAY);
                memcpy(&tmp, &gps->data, sizeof(tmp));
                xSemaphoreGive(gps->mutex);

                nmea_process_sentence(line, &tmp);

                xSemaphoreTake(gps->mutex, portMAX_DELAY);
                memcpy(&gps->data, &tmp, sizeof(tmp));
                gps->data_valid = true;
                xSemaphoreGive(gps->mutex);
            }
            idx = 0;
        } else {
            if (idx < (NMEA_MAX_LEN - 1)) {
                line[idx++] = (char)byte;
            }
            /* else: overflow – will be discarded at next '$' */
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int gps_init(gps_handle_t *gps, uart_port_t uart_num,
             int tx_pin, int rx_pin, uint32_t baud)
{
    memset(gps, 0, sizeof(*gps));
    gps->uart_num = uart_num;
    gps->data_valid = false;

    gps->mutex = xSemaphoreCreateMutex();
    configASSERT(gps->mutex);

    /* Configure UART */
    uart_config_t uart_cfg = {
        .baud_rate  = (int)baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = uart_param_config(uart_num, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = uart_set_pin(uart_num, tx_pin, rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* Launch parser task */
    BaseType_t ok = xTaskCreate(gps_task, "gps_parser", 4096, gps, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create GPS parser task");
        return -1;
    }

    ESP_LOGI(TAG, "GPS UART%d initialised at %lu baud (TX=%d, RX=%d)",
             uart_num, (unsigned long)baud, tx_pin, rx_pin);
    return 0;
}

int gps_get_data(gps_handle_t *gps, sensor_gps_t *out)
{
    if (!gps || !out) {
        return -1;
    }

    xSemaphoreTake(gps->mutex, portMAX_DELAY);
    bool valid = gps->data_valid;
    if (valid) {
        memcpy(out, &gps->data, sizeof(sensor_gps_t));
    }
    xSemaphoreGive(gps->mutex);

    return valid ? 0 : -1;
}
