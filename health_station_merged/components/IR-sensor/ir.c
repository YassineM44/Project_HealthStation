/**
 * @file ir.c
 * @brief IR obstacle sensor driver implementation.
 *
 * Simple digital GPIO polling with software debounce.
 * The FC-51 module handles all analog IR processing on-board;
 * we only read the digital OUT pin.
 */

#include "ir.h"

#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ir";

/* ── ir_init ─────────────────────────────────────────────────────────── */

esp_err_t ir_init(ir_sensor_t *sensor, const ir_config_t *config) {
    if (sensor == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!GPIO_IS_VALID_GPIO(config->out_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->debounce_reads == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->config = *config;

    /* Configure the IR module's digital output as an input.
     * Enable internal pull-up so the line stays HIGH when the module
     * is not actively driving LOW (open-collector output on some
     * variants). */
    gpio_config_t io_conf = {
        .pin_bit_mask  = 1ULL << config->out_gpio,
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    sensor->pill_present   = false;
    sensor->confirm_streak = 0;
    sensor->initialized    = true;

    ESP_LOGI(TAG, "ir_init OK — GPIO=%d, active_low=%s, debounce=%u",
             config->out_gpio,
             config->active_low ? "true" : "false",
             config->debounce_reads);

    return ESP_OK;
}

/* ── ir_update ───────────────────────────────────────────────────────── */

esp_err_t ir_update(ir_sensor_t *sensor, ir_status_t *status) {
    if (sensor == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Read the digital output from the IR module */
    int level = gpio_get_level(sensor->config.out_gpio);

    /* Apply active-low / active-high logic:
     *   active_low  = true  → detected when level == 0
     *   active_low  = false → detected when level == 1 */
    bool detected;
    if (sensor->config.active_low) {
        detected = (level == 0);
    } else {
        detected = (level != 0);
    }

    status->raw_detected = detected;

    /* ── Software debounce ──
     *
     * The debounced state only flips after debounce_reads consecutive
     * readings that disagree with the current confirmed state.
     * This filters out:
     *   - electrical bounce / noise spikes
     *   - brief reflections from passing objects
     *   - sensor oscillation at detection boundary
     */
    if (detected != sensor->pill_present) {
        /* Reading differs from confirmed state — count it */
        sensor->confirm_streak++;

        if (sensor->confirm_streak >= sensor->config.debounce_reads) {
            /* Enough consecutive confirming reads — commit state change */
            sensor->pill_present   = detected;
            sensor->confirm_streak = 0;
        }
    } else {
        /* Reading matches current state — reset streak */
        sensor->confirm_streak = 0;
    }

    status->pill_present = sensor->pill_present;

    return ESP_OK;
}

/* ── ir_calibrate ────────────────────────────────────────────────────── */

/**
 * Helper: read the sensor once and return the logical detection state
 * (accounting for active_low inversion).
 */
static bool ir_read_detected(const ir_sensor_t *sensor) {
    int level = gpio_get_level(sensor->config.out_gpio);
    if (sensor->config.active_low) {
        return (level == 0);
    }
    return (level != 0);
}

esp_err_t ir_calibrate(ir_sensor_t *sensor, uint8_t num_reads,
                       uint32_t interval_ms) {
    if (sensor == NULL || !sensor->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    if (num_reads == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t detected_count = 0;

    ESP_LOGI(TAG, "Calibrating — keep compartment EMPTY (%u reads, %"
             PRIu32 " ms apart)", num_reads, interval_ms);

    for (uint8_t i = 0; i < num_reads; i++) {
        if (ir_read_detected(sensor)) {
            detected_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    if (detected_count > num_reads / 2) {
        /* More than half the reads saw an obstacle — the compartment
         * wall is being detected.  The potentiometer must be adjusted. */
        ESP_LOGE(TAG, "╔═══════════════════════════════════════════════════════╗");
        ESP_LOGE(TAG, "║         CALIBRATION FAILED — WALL DETECTED           ║");
        ESP_LOGE(TAG, "╠═══════════════════════════════════════════════════════╣");
        ESP_LOGE(TAG, "║  Sensor fired %u/%u reads with EMPTY compartment.    ║",
                 detected_count, num_reads);
        ESP_LOGE(TAG, "║                                                       ║");
        ESP_LOGE(TAG, "║  The on-board potentiometer is set TOO SENSITIVE.     ║");
        ESP_LOGE(TAG, "║  The sensor detects the compartment wall as a pill.   ║");
        ESP_LOGE(TAG, "║                                                       ║");
        ESP_LOGE(TAG, "║  FIX:                                                 ║");
        ESP_LOGE(TAG, "║    1. Turn the potentiometer COUNTERCLOCKWISE         ║");
        ESP_LOGE(TAG, "║    2. Keep turning until the on-board LED turns OFF   ║");
        ESP_LOGE(TAG, "║       with the compartment EMPTY                      ║");
        ESP_LOGE(TAG, "║    3. Place a pill — LED should turn ON               ║");
        ESP_LOGE(TAG, "║    4. Restart the ESP32                               ║");
        ESP_LOGE(TAG, "╚═══════════════════════════════════════════════════════╝");

        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Calibration PASSED — wall not detected (%u/%u clear)",
             (uint8_t)(num_reads - detected_count), num_reads);

    return ESP_OK;
}
