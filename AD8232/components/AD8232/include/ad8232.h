/**
 * @file ad8232.h
 * @brief Clinical-grade AD8232 ECG driver for ESP32 (ESP-IDF 5.2.6)
 *
 * Provides hardware-calibrated ADC acquisition at 500 Hz with a 4-stage
 * IIR digital signal processing pipeline:
 *   1. 0.5 Hz high-pass  — baseline wander removal
 *   2. 50 Hz notch       — power-line interference (EU)
 *   3. 60 Hz notch       — power-line interference (US)
 *   4. 40 Hz low-pass    — high-frequency / EMG noise suppression
 *
 * Pin mapping (AD8232 module → ESP32):
 *   OUTPUT → GPIO 34  (ADC1_CH6, input-only)
 *   LO+    → GPIO 32  (digital input, HIGH = lead off)
 *   LO−    → GPIO 33  (digital input, HIGH = lead off)
 *   VCC    → 3.3 V
 *   GND    → GND
 */

#ifndef AD8232_H
#define AD8232_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ────────────────────────── Hardware Configuration ────────────────────────── */

/** ECG analog output pin (input-only) */
#define AD8232_ECG_GPIO         GPIO_NUM_34
/** ADC unit for GPIO 34 */
#define AD8232_ADC_UNIT         ADC_UNIT_1
/** ADC channel for GPIO 34 = ADC1_CH6 */
#define AD8232_ADC_CHANNEL      ADC_CHANNEL_6
/** 12 dB attenuation - measurable range 0-3100 mV (covers AD8232 0-3.3 V) */
#define AD8232_ADC_ATTEN        ADC_ATTEN_DB_12
/** 12-bit resolution → 0–4095 */
#define AD8232_ADC_BITWIDTH     ADC_BITWIDTH_12

/** Lead-off detection positive pin */
#define AD8232_LO_PLUS_GPIO     GPIO_NUM_32
/** Lead-off detection negative pin */
#define AD8232_LO_MINUS_GPIO    GPIO_NUM_33

/** SDN (shutdown) pin — driven HIGH by ESP32 to keep AD8232 active */
#define AD8232_SDN_GPIO         GPIO_NUM_15

/* ────────────────────────── Sampling Configuration ────────────────────────── */

/** Sample rate in Hz (medical ECG standard) */
#define AD8232_SAMPLE_RATE_HZ   500
/** Timer period in microseconds (1 000 000 / 500 = 2000 µs) */
#define AD8232_TIMER_PERIOD_US  (1000000 / AD8232_SAMPLE_RATE_HZ)

/* ────────────────────────── Data Types ──────────────────────────────────── */

/**
 * @brief IIR biquad filter state (Direct Form I)
 *
 * Transfer function:
 *   H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)
 */
typedef struct {
    float b0, b1, b2;   /**< Numerator (feed-forward) coefficients */
    float a1, a2;        /**< Denominator (feed-back) coefficients (a0 = 1) */
    float x1, x2;        /**< Previous input samples  x[n-1], x[n-2] */
    float y1, y2;        /**< Previous output samples y[n-1], y[n-2] */
} ad8232_biquad_t;

/**
 * @brief Single ECG sample
 */
typedef struct {
    uint32_t timestamp_ms;  /**< Milliseconds since boot */
    int      raw;           /**< Raw 12-bit ADC reading (0–4095) */
    int      voltage_mv;    /**< Calibrated voltage in mV (from ADC cal) */
    float    filtered_mv;   /**< Voltage after DSP filter chain (mV) */
    bool     leads_off;     /**< true if LO+ or LO− indicates disconnection */
} ad8232_sample_t;

/**
 * @brief Driver handle (opaque to caller)
 */
typedef struct {
    /* ADC */
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t         cali_handle;
    bool                      cali_valid;

    /* DSP filter bank — order: HP → notch50 → notch60 → LP */
    ad8232_biquad_t hp_filter;      /**< 0.5 Hz high-pass   */
    ad8232_biquad_t notch50;        /**< 50 Hz notch        */
    ad8232_biquad_t notch60;        /**< 60 Hz notch        */
    ad8232_biquad_t lp_filter;      /**< 40 Hz low-pass     */
} ad8232_handle_t;

/* ────────────────────────── Public API ──────────────────────────────────── */

/**
 * @brief Initialise the AD8232 driver
 *
 * Configures ADC oneshot on CH6, creates calibration handle (line-fitting),
 * sets up lead-off GPIOs, and pre-loads IIR filter coefficients.
 *
 * @param[out] handle  Pointer to driver handle to initialise
 * @return ESP_OK on success
 */
esp_err_t ad8232_init(ad8232_handle_t *handle);

/**
 * @brief De-initialise the AD8232 driver
 *
 * Releases ADC unit, calibration handle, and resets GPIOs.
 *
 * @param[in] handle  Pointer to initialised driver handle
 * @return ESP_OK on success
 */
esp_err_t ad8232_deinit(ad8232_handle_t *handle);

/**
 * @brief Acquire and process a single ECG sample
 *
 * Reads ADC → calibrates to mV → applies 4-stage IIR filter chain →
 * populates the sample struct with timestamp, raw, calibrated, and
 * filtered values plus lead-off status.
 *
 * @param[in]  handle  Pointer to initialised driver handle
 * @param[out] sample  Pointer to sample struct to populate
 * @return ESP_OK on success
 */
esp_err_t ad8232_read_sample(ad8232_handle_t *handle, ad8232_sample_t *sample);

/**
 * @brief Check electrode lead-off status
 *
 * @param[out] lo_plus   true if LO+ is HIGH (positive lead off)
 * @param[out] lo_minus  true if LO− is HIGH (negative lead off)
 * @return ESP_OK on success
 */
esp_err_t ad8232_check_leads_off(bool *lo_plus, bool *lo_minus);

#ifdef __cplusplus
}
#endif

#endif /* AD8232_H */
