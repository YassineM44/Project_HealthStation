/**
 * @file ir.h
 * @brief IR obstacle sensor driver for pill compartment detection.
 *
 * Uses a digital-output IR obstacle module (e.g. FC-51).  The module
 * outputs LOW when an obstacle (pill) is detected and HIGH when clear.
 * Software debouncing prevents false triggers from vibration or
 * ambient IR noise.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

/* ── Defaults ────────────────────────────────────────────────────────── */

#define IR_DEFAULT_DEBOUNCE_READS      3U    /* consecutive matching reads  */
#define IR_DEFAULT_POLL_INTERVAL_MS    200U  /* polling period (ms)         */
#define IR_CALIBRATION_READS           10U   /* reads during calibration    */
#define IR_CALIBRATION_INTERVAL_MS     200U  /* delay between cal reads (ms)*/
#define IR_CALIBRATION_SETTLE_MS       2000U /* settle time before cal      */

/* ── Configuration ───────────────────────────────────────────────────── */

typedef struct {
    gpio_num_t out_gpio;           /**< Digital output pin from IR module   */
    bool       active_low;         /**< true = LOW means object detected    */
    uint8_t    debounce_reads;     /**< Consecutive reads to confirm change */
    uint32_t   poll_interval_ms;   /**< Polling cadence (ms)                */
} ir_config_t;

/* ── Runtime status (returned by ir_update) ──────────────────────────── */

typedef struct {
    bool pill_present;             /**< Debounced state: pill detected?     */
    bool raw_detected;             /**< Instantaneous reading (after logic) */
} ir_status_t;

/* ── Sensor handle ───────────────────────────────────────────────────── */

typedef struct {
    ir_config_t config;
    bool        initialized;
    bool        pill_present;      /**< Confirmed (debounced) state         */
    uint8_t     confirm_streak;    /**< Consecutive reads matching new state*/
} ir_sensor_t;

/* ── Public API ──────────────────────────────────────────────────────── */

/**
 * @brief Initialize the IR sensor GPIO.
 *
 * Configures the output pin as a digital input with internal pull-up.
 * The sensor starts in the "no pill" state.
 *
 * @param sensor  Pointer to sensor handle (caller-allocated).
 * @param config  Pointer to configuration.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on bad parameters.
 */
esp_err_t ir_init(ir_sensor_t *sensor, const ir_config_t *config);

/**
 * @brief Read the sensor and update the debounced pill-present state.
 *
 * Call this at the configured poll_interval_ms cadence.  The debounced
 * state only changes after debounce_reads consecutive matching reads.
 *
 * @param sensor  Initialized sensor handle.
 * @param status  Output: current raw + debounced state.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t ir_update(ir_sensor_t *sensor, ir_status_t *status);

/**
 * @brief Run startup calibration (call with EMPTY compartment).
 *
 * Reads the sensor multiple times to verify the compartment wall is
 * NOT being detected.  If the sensor fires during calibration, the
 * potentiometer on the FC-51 module is set too sensitive — the user
 * must turn it counterclockwise until the on-board LED turns off
 * with the compartment empty.
 *
 * @param sensor       Initialized sensor handle.
 * @param num_reads    Number of reads to take during calibration.
 * @param interval_ms  Delay between reads (ms).
 * @return ESP_OK            Calibration passed (wall not detected).
 * @return ESP_ERR_INVALID_STATE  Wall detected; potentiometer needs adjustment.
 */
esp_err_t ir_calibrate(ir_sensor_t *sensor, uint8_t num_reads,
                       uint32_t interval_ms);
