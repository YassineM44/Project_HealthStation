/**
 * @file ad8232.c
 * @brief Clinical-grade AD8232 ECG driver implementation
 *
 * ADC oneshot acquisition with hardware calibration (line-fitting),
 * lead-off detection, and 4-stage IIR DSP filter pipeline.
 */

#include "ad8232.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AD8232";

/* ═══════════════════════════ IIR Biquad Engine ═══════════════════════════ */

/**
 * @brief Apply a single biquad IIR section (Direct Form II Transposed)
 *
 * More numerically stable than Direct Form I for float arithmetic.
 *   y[n] = b0*x[n] + s1
 *   s1   = b1*x[n] - a1*y[n] + s2
 *   s2   = b2*x[n] - a2*y[n]
 */
static float biquad_process(ad8232_biquad_t *f, float x)
{
    float y = f->b0 * x + f->x1;
    f->x1   = f->b1 * x - f->a1 * y + f->x2;
    f->x2   = f->b2 * x - f->a2 * y;
    return y;
}

/**
 * @brief Reset a biquad filter's state to zero
 */
static void biquad_reset(ad8232_biquad_t *f)
{
    f->x1 = 0.0f;  f->x2 = 0.0f;
    f->y1 = 0.0f;  f->y2 = 0.0f;
}

/* ═══════════════════════ Filter Coefficient Init ═════════════════════════ */

/*
 * All coefficients computed for Fs = 500 Hz.
 *
 * 4th-order Butterworth is realised as two cascaded 2nd-order biquad
 * sections.  Each section uses a different pole angle from the analog
 * prototype:
 *   Section 0:  2·cos(5π/8) = 2·cos(112.5°)  (Q ≈ 0.541)
 *   Section 1:  2·cos(7π/8) = 2·cos(157.5°)  (Q ≈ 1.307)
 *
 * This gives -24 dB/octave rolloff vs -12 dB/octave for a single
 * 2nd-order section — critical for eliminating the massive baseline
 * wander and broadband EMG noise in the AD8232 output.
 */

/* 4th-order Butterworth pole widths: 2*cos(π*(2k+N+1)/(2N)) for N=4 */
static const double BW4_ALPHA[2] = {
    0.76536686473,  /* 2·cos(5π/8) = 2·sin(π/8) */
    1.84775906502,  /* 2·cos(7π/8) = 2·cos(π/8) — note: used as |α| */
};

/**
 * @brief Initialise IIR notch filter using Audio EQ Cookbook formula
 */
static void init_notch(ad8232_biquad_t *f, double f0, double Fs, double Q)
{
    const double w0     = 2.0 * M_PI * f0 / Fs;
    const double cos_w0 = cos(w0);
    const double sin_w0 = sin(w0);
    const double alpha  = sin_w0 / (2.0 * Q);

    const double a0 = 1.0 + alpha;

    f->b0 = (float)(1.0 / a0);
    f->b1 = (float)((-2.0 * cos_w0) / a0);
    f->b2 = (float)(1.0 / a0);
    f->a1 = (float)((-2.0 * cos_w0) / a0);
    f->a2 = (float)((1.0 - alpha) / a0);

    biquad_reset(f);
}

/**
 * @brief Initialise one section of a 4th-order Butterworth high-pass
 *
 * @param section  0 or 1 (selects the pole angle)
 */
static void init_highpass_4th(ad8232_biquad_t *f, double Fc, double Fs, int section)
{
    const double wc    = 2.0 * Fs * tan(M_PI * Fc / Fs);
    const double k     = 2.0 * Fs;
    const double k2    = k * k;
    const double wc2   = wc * wc;
    const double alpha = BW4_ALPHA[section]; /* 2·cos(θ_k) */
    const double a_wc_k = alpha * wc * k;
    const double a0    = k2 + a_wc_k + wc2;

    f->b0 = (float)(k2 / a0);
    f->b1 = (float)(-2.0 * k2 / a0);
    f->b2 = (float)(k2 / a0);
    f->a1 = (float)((2.0 * (wc2 - k2)) / a0);
    f->a2 = (float)((k2 - a_wc_k + wc2) / a0);

    biquad_reset(f);
}

/**
 * @brief Initialise one section of a 4th-order Butterworth low-pass
 *
 * @param section  0 or 1 (selects the pole angle)
 */
static void init_lowpass_4th(ad8232_biquad_t *f, double Fc, double Fs, int section)
{
    const double wc    = 2.0 * Fs * tan(M_PI * Fc / Fs);
    const double k     = 2.0 * Fs;
    const double k2    = k * k;
    const double wc2   = wc * wc;
    const double alpha = BW4_ALPHA[section];
    const double a_wc_k = alpha * wc * k;
    const double a0    = k2 + a_wc_k + wc2;

    f->b0 = (float)(wc2 / a0);
    f->b1 = (float)(2.0 * wc2 / a0);
    f->b2 = (float)(wc2 / a0);
    f->a1 = (float)((2.0 * (wc2 - k2)) / a0);
    f->a2 = (float)((k2 - a_wc_k + wc2) / a0);

    biquad_reset(f);
}

/* ═══════════════════════ Median Filter (impulse removal) ═════════════════ */

static float median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    if (a > b) { b = a; }
    return b;
}

static float median_filter(ad8232_handle_t *h, float x)
{
    h->med_buf[h->med_idx] = x;
    h->med_idx = (h->med_idx + 1) % AD8232_MEDIAN_LEN;
    if (!h->med_primed) {
        if (h->med_idx == 0) h->med_primed = true;
        return x;  /* pass-through until buffer is full */
    }
    return median3(h->med_buf[0], h->med_buf[1], h->med_buf[2]);
}

/* ══════════════════════════ ADC & Calibration ═════════════════════════════ */

static esp_err_t adc_calibration_init(adc_unit_t unit,
                                       adc_atten_t atten,
                                       adc_cali_handle_t *out_handle,
                                       bool *out_valid)
{
    esp_err_t ret = ESP_FAIL;
    *out_valid = false;

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id  = unit,
        .atten    = atten,
        .bitwidth = AD8232_ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, out_handle);
    if (ret == ESP_OK) {
        *out_valid = true;
        ESP_LOGI(TAG, "ADC calibration: line-fitting scheme created");
    } else {
        ESP_LOGW(TAG, "ADC calibration failed (0x%x), using raw values", ret);
    }
#else
    ESP_LOGW(TAG, "Line-fitting calibration not supported on this chip");
#endif

    return ret;
}

/* ═══════════════════════════ GPIO Lead-Off ════════════════════════════════ */

static esp_err_t leadoff_gpio_init(void)
{
    /* NO pull-up or pull-down — the AD8232 module has its own pull
     * resistors on LO+/LO-. Adding ESP32 internal pull-downs (~45k)
     * creates a voltage divider that causes the pins to oscillate
     * near the logic threshold at 50 Hz. */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AD8232_LO_PLUS_GPIO) |
                        (1ULL << AD8232_LO_MINUS_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_conf);
}

/* ═════════════════════════ Public API Implementation ═════════════════════ */

esp_err_t ad8232_init(ad8232_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(handle, 0, sizeof(ad8232_handle_t));

    esp_err_t ret;

    /* ── ADC oneshot unit ── */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = AD8232_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&unit_cfg, &handle->adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit: 0x%x", ret);
        return ret;
    }

    /* ── ADC channel configuration ── */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = AD8232_ADC_ATTEN,
        .bitwidth = AD8232_ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(handle->adc_handle,
                                     AD8232_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: 0x%x", ret);
        adc_oneshot_del_unit(handle->adc_handle);
        return ret;
    }

    /* ── ADC calibration ── */
    adc_calibration_init(AD8232_ADC_UNIT, AD8232_ADC_ATTEN,
                         &handle->cali_handle, &handle->cali_valid);

    /* ── SDN pin: drive HIGH to keep AD8232 active (not shutdown) ── */
    gpio_config_t sdn_conf = {
        .pin_bit_mask = (1ULL << AD8232_SDN_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&sdn_conf);
    if (ret == ESP_OK) {
        gpio_set_level(AD8232_SDN_GPIO, 1);  /* HIGH = active */
        ESP_LOGI(TAG, "SDN pin GPIO%d set HIGH (AD8232 active)", AD8232_SDN_GPIO);
    } else {
        ESP_LOGW(TAG, "SDN GPIO config failed: 0x%x", ret);
    }

    /* Let the AD8232 power up and stabilize */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* ── Lead-off GPIOs ── */
    ret = leadoff_gpio_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Lead-off GPIO init warning: 0x%x", ret);
    }

    /* ── DSP Filter Bank (Monitor Mode Bandwidth: 0.67 Hz – 18 Hz) ── */
    /*
     * 4th-order Butterworth HP @ 0.67 Hz  (2 cascaded sections)
     *   → -24 dB/octave below cut-off → kills baseline wander
     *
     * 50/60 Hz notch Q=30
     *   → narrow notch removes power-line without distorting QRS
     *
     * 4th-order Butterworth LP @ 18 Hz  (2 cascaded sections)
     *   → -24 dB/octave above cut-off → kills EMG / broadband noise
     *   → preserves QRS complex (R-peak energy is below 15 Hz)
     *
     * 3-point median filter (handled in read_sample)
     *   → removes impulse spikes without phase distortion
     */
    init_highpass_4th(&handle->hp1, 0.67, 500.0, 0);
    init_highpass_4th(&handle->hp2, 0.67, 500.0, 1);
    init_notch(&handle->notch50, 50.0, 500.0, 30.0);
    init_notch(&handle->notch60, 60.0, 500.0, 30.0);
    init_lowpass_4th(&handle->lp1, 18.0, 500.0, 0);
    init_lowpass_4th(&handle->lp2, 18.0, 500.0, 1);

    ESP_LOGI(TAG, "AD8232 driver initialised — %d Hz, GPIO%d (ADC1_CH%d)",
             AD8232_SAMPLE_RATE_HZ, AD8232_ECG_GPIO, AD8232_ADC_CHANNEL);
    return ESP_OK;
}

esp_err_t ad8232_deinit(ad8232_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (handle->cali_valid && handle->cali_handle) {
        adc_cali_delete_scheme_line_fitting(handle->cali_handle);
        handle->cali_handle = NULL;
        handle->cali_valid = false;
    }
#endif

    if (handle->adc_handle) {
        adc_oneshot_del_unit(handle->adc_handle);
        handle->adc_handle = NULL;
    }

    gpio_reset_pin(AD8232_LO_PLUS_GPIO);
    gpio_reset_pin(AD8232_LO_MINUS_GPIO);

    ESP_LOGI(TAG, "AD8232 driver de-initialised");
    return ESP_OK;
}

esp_err_t ad8232_read_sample(ad8232_handle_t *handle, ad8232_sample_t *sample)
{
    if (handle == NULL || sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* ── Timestamp ── */
    sample->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

    /* ── Lead-off check ── */
    bool lo_plus  = (gpio_get_level(AD8232_LO_PLUS_GPIO)  == 1);
    bool lo_minus = (gpio_get_level(AD8232_LO_MINUS_GPIO) == 1);
    sample->leads_off = lo_plus || lo_minus;

    /* ── ADC read ── */
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(handle->adc_handle,
                                      AD8232_ADC_CHANNEL, &raw);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: 0x%x", ret);
        return ret;
    }
    sample->raw = raw;

    /* ── Raw to millivolts via calibration ── */
    int voltage_mv = 0;
    if (handle->cali_valid) {
        adc_cali_raw_to_voltage(handle->cali_handle, raw, &voltage_mv);
    } else {
        voltage_mv = (int)((float)raw * 3100.0f / 4095.0f);
    }
    sample->voltage_mv = voltage_mv;

    /* ── DSP filter chain (ALWAYS runs, never reset on lead-off) ── */
    float v = (float)voltage_mv;

    /* 4th-order HP: two cascaded biquad sections */
    v = biquad_process(&handle->hp1, v);
    v = biquad_process(&handle->hp2, v);

    /* Notch filters */
    v = biquad_process(&handle->notch50, v);
    v = biquad_process(&handle->notch60, v);

    /* 4th-order LP: two cascaded biquad sections */
    v = biquad_process(&handle->lp1, v);
    v = biquad_process(&handle->lp2, v);

    /* 3-point median: remove impulse spikes */
    v = median_filter(handle, v);

    sample->filtered_mv = v;
    return ESP_OK;
}

esp_err_t ad8232_check_leads_off(bool *lo_plus, bool *lo_minus)
{
    if (lo_plus == NULL || lo_minus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *lo_plus  = (gpio_get_level(AD8232_LO_PLUS_GPIO)  == 1);
    *lo_minus = (gpio_get_level(AD8232_LO_MINUS_GPIO) == 1);
    return ESP_OK;
}
