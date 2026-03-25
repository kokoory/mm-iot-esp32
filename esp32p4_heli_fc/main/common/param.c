/*
 * Parameter System Implementation
 *
 * Uses ESP-IDF NVS (Non-Volatile Storage) for persistence.
 * Parameters are stored as floats with a 16-char key name.
 */

#include "param.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "param";
#define NVS_NAMESPACE "fc_params"

/* Parameter table - order must match param_id_t enum */
static const param_meta_t s_param_table[PARAM_COUNT] = {
    /* Rate controller PID */
    [PARAM_ROLL_RATE_KP]     = { "MC_ROLLRATE_P",   0.15f,  0.0f,  2.0f },
    [PARAM_ROLL_RATE_KI]     = { "MC_ROLLRATE_I",   0.05f,  0.0f,  1.0f },
    [PARAM_ROLL_RATE_KD]     = { "MC_ROLLRATE_D",   0.003f, 0.0f,  0.1f },
    [PARAM_PITCH_RATE_KP]    = { "MC_PITCHRATE_P",  0.15f,  0.0f,  2.0f },
    [PARAM_PITCH_RATE_KI]    = { "MC_PITCHRATE_I",  0.05f,  0.0f,  1.0f },
    [PARAM_PITCH_RATE_KD]    = { "MC_PITCHRATE_D",  0.003f, 0.0f,  0.1f },
    [PARAM_YAW_RATE_KP]      = { "MC_YAWRATE_P",    0.3f,   0.0f,  2.0f },
    [PARAM_YAW_RATE_KI]      = { "MC_YAWRATE_I",    0.1f,   0.0f,  1.0f },
    [PARAM_YAW_RATE_KD]      = { "MC_YAWRATE_D",    0.0f,   0.0f,  0.1f },

    /* Attitude controller */
    [PARAM_ATT_ROLL_KP]      = { "MC_ROLL_P",       4.5f,   0.5f,  12.0f },
    [PARAM_ATT_PITCH_KP]     = { "MC_PITCH_P",      4.5f,   0.5f,  12.0f },
    [PARAM_ATT_YAW_KP]       = { "MC_YAW_P",        2.0f,   0.5f,  8.0f  },

    /* Altitude / position */
    [PARAM_ALT_KP]            = { "MPC_Z_P",         1.0f,   0.1f,  5.0f },
    [PARAM_CLIMB_RATE_KP]     = { "MPC_Z_VEL_P",     0.5f,   0.1f,  2.0f },
    [PARAM_CLIMB_RATE_KI]     = { "MPC_Z_VEL_I",     0.1f,   0.0f,  1.0f },
    [PARAM_CLIMB_RATE_KD]     = { "MPC_Z_VEL_D",     0.0f,   0.0f,  0.5f },
    [PARAM_MAX_CLIMB_RATE]    = { "MPC_Z_VEL_MAX",   2.0f,   0.5f,  10.0f },

    /* Attitude limits */
    [PARAM_MAX_ROLL_DEG]      = { "MC_ROLL_LIM",     45.0f,  10.0f, 80.0f },
    [PARAM_MAX_PITCH_DEG]     = { "MC_PITCH_LIM",    45.0f,  10.0f, 80.0f },
    [PARAM_MAX_YAW_RATE_DEG]  = { "MC_YAWRATE_LIM",  180.0f, 30.0f, 360.0f },

    /* AHRS / estimator */
    [PARAM_AHRS_BETA]         = { "EKF2_BETA",       0.1f,   0.01f, 1.0f },
    [PARAM_ALT_CF_ALPHA]      = { "EKF2_BARO_CF",    0.98f,  0.5f,  0.999f },

    /* Helicopter mixer */
    [PARAM_CCPM_ANGLE_OFFSET] = { "H_CCPM_OFFSET",   0.0f,   -180.0f, 180.0f },
    [PARAM_COLLECTIVE_RANGE]  = { "H_COLL_RANGE",    1.0f,   0.1f,  2.0f },
    [PARAM_CYCLIC_RANGE]      = { "H_CYC_RANGE",     1.0f,   0.1f,  2.0f },
    [PARAM_SERVO_CENTER_US]   = { "H_SERVO_CTR",     1500.0f, 1000.0f, 2000.0f },
    [PARAM_SERVO_RANGE_US]    = { "H_SERVO_RNG",     500.0f, 100.0f, 600.0f },
    [PARAM_ESC_MIN_US]        = { "H_ESC_MIN",       1000.0f, 800.0f, 1200.0f },
    [PARAM_ESC_MAX_US]        = { "H_ESC_MAX",       2000.0f, 1800.0f, 2200.0f },
    [PARAM_TAIL_COLL_FF]      = { "H_TAIL_FF",       0.3f,   0.0f,  1.0f },

    /* Battery thresholds */
    [PARAM_BATT_LOW_V]        = { "BAT_V_LOW",       10.5f,  6.0f,  50.0f },
    [PARAM_BATT_CRIT_V]       = { "BAT_V_CRIT",      9.6f,   5.0f,  48.0f },
    [PARAM_BATT_VDIV_RATIO]   = { "BAT_V_DIV",       11.0f,  1.0f,  50.0f },

    /* System */
    [PARAM_SENSOR_TIMEOUT_MS] = { "SYS_SENS_TMO",    500.0f, 100.0f, 5000.0f },
};

/* Runtime parameter values */
static float s_values[PARAM_COUNT];
static bool s_modified[PARAM_COUNT];
static bool s_initialized = false;

/* ── Implementation ─────────────────────────────────────────── */

void param_init(void)
{
    if (s_initialized) return;

    /* Set all to defaults first */
    for (int i = 0; i < PARAM_COUNT; i++) {
        s_values[i] = s_param_table[i].default_val;
        s_modified[i] = false;
    }

    /* Load saved values from NVS */
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        int loaded = 0;
        for (int i = 0; i < PARAM_COUNT; i++) {
            uint32_t raw;
            err = nvs_get_u32(nvs, s_param_table[i].name, &raw);
            if (err == ESP_OK) {
                float val;
                memcpy(&val, &raw, sizeof(float));
                /* Validate range */
                if (val >= s_param_table[i].min_val && val <= s_param_table[i].max_val) {
                    s_values[i] = val;
                    loaded++;
                }
            }
        }
        nvs_close(nvs);
        ESP_LOGI(TAG, "Loaded %d/%d parameters from NVS", loaded, PARAM_COUNT);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved parameters found, using defaults");
    } else {
        ESP_LOGW(TAG, "NVS open failed: %s, using defaults", esp_err_to_name(err));
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Parameter system initialized (%d params)", PARAM_COUNT);
}

float param_get(param_id_t id)
{
    if (id >= PARAM_COUNT) return 0.0f;
    return s_values[id];
}

bool param_set(param_id_t id, float value)
{
    if (id >= PARAM_COUNT) return false;

    /* Clamp to valid range */
    if (value < s_param_table[id].min_val) value = s_param_table[id].min_val;
    if (value > s_param_table[id].max_val) value = s_param_table[id].max_val;

    if (s_values[id] == value) return false;

    s_values[id] = value;
    s_modified[id] = true;

    ESP_LOGI(TAG, "PARAM SET: %s = %.4f", s_param_table[id].name, value);
    return true;
}

void param_reset(param_id_t id)
{
    if (id >= PARAM_COUNT) return;
    s_values[id] = s_param_table[id].default_val;
    s_modified[id] = true;
}

void param_reset_all(void)
{
    for (int i = 0; i < PARAM_COUNT; i++) {
        s_values[i] = s_param_table[i].default_val;
        s_modified[i] = true;
    }
    ESP_LOGI(TAG, "All parameters reset to defaults");
}

int param_save_all(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open for write failed: %s", esp_err_to_name(err));
        return -1;
    }

    int saved = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        uint32_t raw;
        memcpy(&raw, &s_values[i], sizeof(float));
        err = nvs_set_u32(nvs, s_param_table[i].name, raw);
        if (err == ESP_OK) {
            saved++;
        } else {
            ESP_LOGW(TAG, "Failed to save %s: %s", s_param_table[i].name, esp_err_to_name(err));
        }
        s_modified[i] = false;
    }

    err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
        return -1;
    }

    ESP_LOGI(TAG, "Saved %d/%d parameters to NVS", saved, PARAM_COUNT);
    return 0;
}

uint16_t param_count(void)
{
    return PARAM_COUNT;
}

const param_meta_t *param_get_meta(param_id_t id)
{
    if (id >= PARAM_COUNT) return NULL;
    return &s_param_table[id];
}

param_id_t param_find(const char *name)
{
    if (!name) return PARAM_COUNT;
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(s_param_table[i].name, name, 16) == 0) {
            return (param_id_t)i;
        }
    }
    return PARAM_COUNT;
}
