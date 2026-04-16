/**
 * @file main.c
 * @brief IR Pill Compartment — Standalone Test Application
 *
 * Single FreeRTOS task that polls the IR obstacle sensor and logs
 * pill state changes.  Event-driven output: only prints when the
 * debounced state transitions (PLACED ↔ REMOVED).
 *
 * At startup, runs a calibration check to verify the compartment
 * wall is NOT being detected.  If it is, the potentiometer on the
 * IR module must be adjusted before monitoring can begin.
 */

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ir.h"

static const char *TAG = "IR";

/* ── Pin & Tuning ────────────────────────────────────────────────────── */

#define IR_OUT_GPIO             GPIO_NUM_17
#define IR_ACTIVE_LOW           true
#define IR_DEBOUNCE_READS       IR_DEFAULT_DEBOUNCE_READS     /* 3 */
#define IR_POLL_INTERVAL_MS     IR_DEFAULT_POLL_INTERVAL_MS   /* 200 */

/* ── IR Sensor Task ──────────────────────────────────────────────────── */

static void ir_task(void *arg) {
    (void)arg;

    ir_sensor_t sensor;
    ir_status_t status = {0};

    ir_config_t config = {
        .out_gpio        = IR_OUT_GPIO,
        .active_low      = IR_ACTIVE_LOW,
        .debounce_reads  = IR_DEBOUNCE_READS,
        .poll_interval_ms = IR_POLL_INTERVAL_MS,
    };

    esp_err_t err = ir_init(&sensor, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ir_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    /* ── Startup calibration ──
     * Let the sensor settle for 2 seconds, then take 10 readings.
     * If the compartment wall is being detected, stop and tell the
     * user to adjust the potentiometer. */
    ESP_LOGI(TAG, "Waiting %u ms for sensor to settle...",
             (unsigned)IR_CALIBRATION_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(IR_CALIBRATION_SETTLE_MS));

    err = ir_calibrate(&sensor, IR_CALIBRATION_READS,
                       IR_CALIBRATION_INTERVAL_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Calibration failed — adjust potentiometer and restart.");
        ESP_LOGE(TAG, "Monitoring will NOT start.");
        vTaskDelete(NULL);
        return;
    }

    /* ── Normal monitoring loop ── */
    bool prev_pill_present = false;

    while (1) {
        err = ir_update(&sensor, &status);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ir_update failed: %s", esp_err_to_name(err));
        } else if (status.pill_present != prev_pill_present) {
            if (status.pill_present) {
                ESP_LOGI(TAG, "Pill PLACED");
            } else {
                ESP_LOGW(TAG, "Pill REMOVED");
            }
            prev_pill_present = status.pill_present;
        }

        vTaskDelay(pdMS_TO_TICKS(config.poll_interval_ms));
    }
}

/* ── app_main ────────────────────────────────────────────────────────── */

void app_main(void) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   IR Pill Compartment — Standalone   ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

    xTaskCreate(ir_task, "ir_task", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "IR sensor task launched.");
}
