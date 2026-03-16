#include "ad8232.h"

#include <inttypes.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

#define AD8232_ADC_UNIT ADC_UNIT_1
#define AD8232_ADC_ATTEN ADC_ATTEN_DB_12
#define AD8232_ADC_CALI_BIT_WIDTH ADC_BITWIDTH_DEFAULT
#define AD8232_ADC_PATTERN_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH
#define AD8232_RAW_SAMPLE_RATE_HZ 20000U
#define AD8232_PUBLISHED_SAMPLE_RATE_HZ 250U
#define AD8232_DECIMATION_FACTOR (AD8232_RAW_SAMPLE_RATE_HZ / AD8232_PUBLISHED_SAMPLE_RATE_HZ)
#define AD8232_SAMPLE_QUEUE_LENGTH 64U
#define AD8232_ADC_FRAME_SIZE 256U
#define AD8232_ADC_BUFFER_SIZE 2048U
#define AD8232_ADC_MAX_RAW 4095U
#define AD8232_ADC_CLIP_MARGIN 32U
#define AD8232_TASK_STACK_SIZE 4096U
#define AD8232_TASK_PRIORITY 5U
#define AD8232_READ_TIMEOUT_MS 100U
#define AD8232_REACQUIRE_MS 250U
#define AD8232_DEFAULT_VREF_MV 1100U
#define AD8232_LEAD_OFF_ASSERT_SAMPLES 3U
#define AD8232_LEAD_OFF_RELEASE_SAMPLES 3U
#define AD8232_CLIP_WINDOW_SAMPLES 32U
#define AD8232_CLIP_LATCH_THRESHOLD 4U

#define AD8232_BASELINE_ALPHA 0.9875122565f

#if (AD8232_RAW_SAMPLE_RATE_HZ % AD8232_PUBLISHED_SAMPLE_RATE_HZ) != 0
#error "AD8232 raw sample rate must be divisible by the published sample rate"
#endif

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} ad8232_biquad_coeffs_t;

typedef struct {
    float x1;
    float x2;
    float y1;
    float y2;
} ad8232_biquad_state_t;

typedef struct {
    ad8232_config_t config;
    adc_unit_t adc_unit;
    adc_channel_t adc_channel;
    adc_continuous_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    QueueHandle_t sample_queue;
    TaskHandle_t task_handle;
    uint32_t decimator_sum;
    uint16_t decimator_count;
    uint64_t reacquire_deadline_ms;
    float baseline_uv;
    float cali_gain_uv_per_count;
    int32_t cali_offset_uv;
    bool baseline_initialized;
    bool was_lead_off;
    bool lead_off_confirmed;
    bool initialized;
    bool started;
    bool stop_requested;
    bool limited_absolute_accuracy;
    uint8_t lead_off_high_count;
    uint8_t lead_off_low_count;
    uint8_t median_count;
    uint8_t clip_window_head;
    uint8_t clip_window_count;
    uint8_t clip_window_sum;
    int32_t median_window_uv[3];
    uint8_t clip_window[AD8232_CLIP_WINDOW_SAMPLES];
    ad8232_biquad_state_t notch_state;
    ad8232_biquad_state_t lowpass_state;
} ad8232_driver_t;

static const char *TAG = "AD8232";

static const ad8232_biquad_coeffs_t s_notch_coeffs = {
    .b0 = 0.9756567131f,
    .b1 = -0.1225239838f,
    .b2 = 0.9756567131f,
    .a1 = -0.1225239838f,
    .a2 = 0.9513134261f,
};

static const ad8232_biquad_coeffs_t s_lowpass_coeffs = {
    .b0 = 0.1453238839f,
    .b1 = 0.2906477678f,
    .b2 = 0.1453238839f,
    .a1 = -0.6710290908f,
    .a2 = 0.2523246263f,
};

static ad8232_driver_t s_driver;

static void ad8232_reset_decimator(void)
{
    s_driver.decimator_sum = 0;
    s_driver.decimator_count = 0;
}

static void ad8232_reset_signal_chain(void)
{
    memset(&s_driver.notch_state, 0, sizeof(s_driver.notch_state));
    memset(&s_driver.lowpass_state, 0, sizeof(s_driver.lowpass_state));
    s_driver.baseline_uv = 0.0f;
    s_driver.baseline_initialized = false;
    s_driver.median_count = 0;
    s_driver.clip_window_head = 0;
    s_driver.clip_window_count = 0;
    s_driver.clip_window_sum = 0;
    memset(s_driver.median_window_uv, 0, sizeof(s_driver.median_window_uv));
    memset(s_driver.clip_window, 0, sizeof(s_driver.clip_window));
    ad8232_reset_decimator();
}

static float ad8232_biquad_process(const ad8232_biquad_coeffs_t *coeffs, ad8232_biquad_state_t *state, float x)
{
    const float y = (coeffs->b0 * x) +
                    (coeffs->b1 * state->x1) +
                    (coeffs->b2 * state->x2) -
                    (coeffs->a1 * state->y1) -
                    (coeffs->a2 * state->y2);

    state->x2 = state->x1;
    state->x1 = x;
    state->y2 = state->y1;
    state->y1 = y;

    return y;
}

static int32_t ad8232_round_float_to_i32(float value)
{
    if (value >= 0.0f) {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

static int32_t ad8232_median3(int32_t a, int32_t b, int32_t c)
{
    if (a > b) {
        const int32_t temp = a;
        a = b;
        b = temp;
    }

    if (b > c) {
        const int32_t temp = b;
        b = c;
        c = temp;
    }

    if (a > b) {
        const int32_t temp = a;
        a = b;
        b = temp;
    }

    return b;
}

static int32_t ad8232_push_median_sample(int32_t calibrated_uv)
{
    s_driver.median_window_uv[s_driver.median_count % 3U] = calibrated_uv;

    if (s_driver.median_count < 3U) {
        s_driver.median_count++;
    } else {
        s_driver.median_count = 3U;
    }

    if (s_driver.median_count < 3U) {
        return calibrated_uv;
    }

    return ad8232_median3(s_driver.median_window_uv[0], s_driver.median_window_uv[1], s_driver.median_window_uv[2]);
}

static int32_t ad8232_raw_to_uv(uint16_t raw)
{
    const float calibrated_uv = ((float)raw * s_driver.cali_gain_uv_per_count) + (float)s_driver.cali_offset_uv;

    return ad8232_round_float_to_i32(calibrated_uv);
}

static bool ad8232_update_clip_window(bool instant_clip)
{
    if (s_driver.clip_window_count < AD8232_CLIP_WINDOW_SAMPLES) {
        s_driver.clip_window[s_driver.clip_window_head] = instant_clip ? 1U : 0U;
        s_driver.clip_window_sum += instant_clip ? 1U : 0U;
        s_driver.clip_window_count++;
        s_driver.clip_window_head = (uint8_t)((s_driver.clip_window_head + 1U) % AD8232_CLIP_WINDOW_SAMPLES);
    } else {
        s_driver.clip_window_sum -= s_driver.clip_window[s_driver.clip_window_head];
        s_driver.clip_window[s_driver.clip_window_head] = instant_clip ? 1U : 0U;
        s_driver.clip_window_sum += instant_clip ? 1U : 0U;
        s_driver.clip_window_head = (uint8_t)((s_driver.clip_window_head + 1U) % AD8232_CLIP_WINDOW_SAMPLES);
    }

    return instant_clip || (s_driver.clip_window_sum >= AD8232_CLIP_LATCH_THRESHOLD);
}

static void ad8232_queue_sample(const ad8232_sample_t *sample)
{
    ad8232_sample_t dropped_sample;

    if (xQueueSend(s_driver.sample_queue, sample, 0) == pdPASS) {
        return;
    }

    if (xQueueReceive(s_driver.sample_queue, &dropped_sample, 0) == pdPASS) {
        (void)xQueueSend(s_driver.sample_queue, sample, 0);
    }
}

static void ad8232_publish_invalid_sample(uint64_t timestamp_ms, bool lo_plus, bool lo_minus)
{
    ad8232_sample_t sample = {
        .timestamp_ms = timestamp_ms,
        .raw = 0,
        .signal_mv = 0,
        .ecg_uv = 0,
        .valid = false,
        .lo_plus = lo_plus,
        .lo_minus = lo_minus,
        .clipped = false,
    };

    ad8232_queue_sample(&sample);
}

static bool ad8232_update_lead_state(bool lo_plus, bool lo_minus)
{
    const bool lead_off_detected = lo_plus || lo_minus;

    if (lead_off_detected) {
        s_driver.lead_off_low_count = 0;

        if (s_driver.lead_off_high_count < AD8232_LEAD_OFF_ASSERT_SAMPLES) {
            s_driver.lead_off_high_count++;
        }

        if (!s_driver.lead_off_confirmed && s_driver.lead_off_high_count >= AD8232_LEAD_OFF_ASSERT_SAMPLES) {
            s_driver.lead_off_confirmed = true;
            s_driver.was_lead_off = true;
            ad8232_reset_signal_chain();
        }
    } else {
        s_driver.lead_off_high_count = 0;

        if (s_driver.lead_off_low_count < AD8232_LEAD_OFF_RELEASE_SAMPLES) {
            s_driver.lead_off_low_count++;
        }

        if (s_driver.lead_off_confirmed && s_driver.lead_off_low_count >= AD8232_LEAD_OFF_RELEASE_SAMPLES) {
            s_driver.lead_off_confirmed = false;
        }
    }

    return s_driver.lead_off_confirmed;
}

static esp_err_t ad8232_calibration_init(void)
{
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_efuse_val_t efuse_value = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
    const esp_err_t efuse_ret = adc_cali_scheme_line_fitting_check_efuse(&efuse_value);
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = s_driver.adc_unit,
        .atten = AD8232_ADC_ATTEN,
        .bitwidth = AD8232_ADC_CALI_BIT_WIDTH,
        .default_vref = 0,
    };

    s_driver.limited_absolute_accuracy = false;

    if (efuse_ret == ESP_OK && efuse_value == ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF) {
        cali_config.default_vref = AD8232_DEFAULT_VREF_MV;
        s_driver.limited_absolute_accuracy = true;
        ESP_LOGW(TAG, "ADC eFuse calibration is incomplete, using default Vref %u mV", AD8232_DEFAULT_VREF_MV);
    } else if (efuse_ret == ESP_ERR_NOT_SUPPORTED) {
        cali_config.default_vref = AD8232_DEFAULT_VREF_MV;
        s_driver.limited_absolute_accuracy = true;
        ESP_LOGW(TAG, "ADC eFuse calibration unavailable, falling back to default Vref %u mV", AD8232_DEFAULT_VREF_MV);
    } else if (efuse_ret != ESP_OK) {
        return efuse_ret;
    }

    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_line_fitting(&cali_config, &s_driver.cali_handle), TAG, "failed to create ADC calibration");

    {
        int low_mv = 0;
        int high_mv = 0;

        if (adc_cali_raw_to_voltage(s_driver.cali_handle, 0, &low_mv) == ESP_OK &&
            adc_cali_raw_to_voltage(s_driver.cali_handle, AD8232_ADC_MAX_RAW, &high_mv) == ESP_OK &&
            high_mv > low_mv) {
            s_driver.cali_offset_uv = low_mv * 1000;
            s_driver.cali_gain_uv_per_count = ((float)(high_mv - low_mv) * 1000.0f) / (float)AD8232_ADC_MAX_RAW;
        } else {
            s_driver.cali_offset_uv = 0;
            s_driver.cali_gain_uv_per_count = 3100000.0f / (float)AD8232_ADC_MAX_RAW;
            ESP_LOGW(TAG, "Falling back to nominal ADC scale %.1f uV/count", s_driver.cali_gain_uv_per_count);
        }
    }

    if (!s_driver.limited_absolute_accuracy) {
        ESP_LOGI(TAG, "ADC calibration source: eFuse line fitting");
    }

    ESP_LOGI(TAG,
             "ADC transfer estimate: offset=%" PRId32 " uV, gain=%.1f uV/count",
             s_driver.cali_offset_uv,
             s_driver.cali_gain_uv_per_count);

    return ESP_OK;
#else
    ESP_LOGE(TAG, "ADC line fitting calibration is not supported on this build");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void ad8232_calibration_deinit(void)
{
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (s_driver.cali_handle != NULL) {
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(s_driver.cali_handle));
        s_driver.cali_handle = NULL;
    }
#endif
}

static esp_err_t ad8232_setup_gpio(const ad8232_config_t *config)
{
    const gpio_config_t lead_off_gpio_cfg = {
        .pin_bit_mask = (1ULL << config->lo_plus_gpio) | (1ULL << config->lo_minus_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&lead_off_gpio_cfg);
}

static esp_err_t ad8232_setup_adc(void)
{
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = AD8232_ADC_BUFFER_SIZE,
        .conv_frame_size = AD8232_ADC_FRAME_SIZE,
    };
    adc_continuous_config_t adc_config = {
        .sample_freq_hz = AD8232_RAW_SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    adc_digi_pattern_config_t adc_pattern = {
        .atten = AD8232_ADC_ATTEN,
        .channel = (uint8_t)(s_driver.adc_channel & 0x7),
        .unit = AD8232_ADC_UNIT,
        .bit_width = AD8232_ADC_PATTERN_BIT_WIDTH,
    };

    adc_config.pattern_num = 1;
    adc_config.adc_pattern = &adc_pattern;

    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&handle_cfg, &s_driver.adc_handle), TAG, "failed to create ADC continuous handle");
    ESP_RETURN_ON_ERROR(adc_continuous_config(s_driver.adc_handle, &adc_config), TAG, "failed to configure ADC continuous mode");

    return ESP_OK;
}

static void ad8232_cleanup_driver(void)
{
    if (s_driver.started && s_driver.adc_handle != NULL) {
        (void)adc_continuous_stop(s_driver.adc_handle);
        s_driver.started = false;
    }

    if (s_driver.task_handle != NULL) {
        vTaskDelete(s_driver.task_handle);
        s_driver.task_handle = NULL;
    }

    if (s_driver.adc_handle != NULL) {
        (void)adc_continuous_deinit(s_driver.adc_handle);
        s_driver.adc_handle = NULL;
    }

    ad8232_calibration_deinit();

    if (s_driver.sample_queue != NULL) {
        vQueueDelete(s_driver.sample_queue);
        s_driver.sample_queue = NULL;
    }

    memset(&s_driver, 0, sizeof(s_driver));
}

static void ad8232_process_connected_sample(uint16_t averaged_raw, bool lo_plus, bool lo_minus)
{
    const bool instant_clip = (averaged_raw <= AD8232_ADC_CLIP_MARGIN) || (averaged_raw >= (AD8232_ADC_MAX_RAW - AD8232_ADC_CLIP_MARGIN));
    ad8232_sample_t sample = {
        .timestamp_ms = (uint64_t)(esp_timer_get_time() / 1000),
        .raw = averaged_raw,
        .signal_mv = 0,
        .ecg_uv = 0,
        .valid = false,
        .lo_plus = lo_plus,
        .lo_minus = lo_minus,
        .clipped = false,
    };
    int calibrated_mv = 0;
    int32_t calibrated_uv = 0;
    int32_t filtered_input_uv = 0;

    if (adc_cali_raw_to_voltage(s_driver.cali_handle, averaged_raw, &calibrated_mv) != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration conversion failed for raw sample %u", averaged_raw);
    }

    calibrated_uv = ad8232_raw_to_uv(averaged_raw);
    sample.signal_mv = calibrated_mv;
    sample.clipped = ad8232_update_clip_window(instant_clip);
    filtered_input_uv = ad8232_push_median_sample(calibrated_uv);

    if (!s_driver.baseline_initialized) {
        s_driver.baseline_uv = (float)filtered_input_uv;
        s_driver.baseline_initialized = true;
    }

    s_driver.baseline_uv = (AD8232_BASELINE_ALPHA * s_driver.baseline_uv) +
                           ((1.0f - AD8232_BASELINE_ALPHA) * (float)filtered_input_uv);

    {
        const float centered_uv = (float)filtered_input_uv - s_driver.baseline_uv;
        const float notch_uv = ad8232_biquad_process(&s_notch_coeffs, &s_driver.notch_state, centered_uv);
        const float filtered_uv = ad8232_biquad_process(&s_lowpass_coeffs, &s_driver.lowpass_state, notch_uv);

        if (sample.timestamp_ms >= s_driver.reacquire_deadline_ms) {
            sample.valid = true;
            sample.ecg_uv = ad8232_round_float_to_i32(filtered_uv);
        }
    }

    ad8232_queue_sample(&sample);
}

static void ad8232_acquisition_task(void *arg)
{
    uint8_t raw_buffer[AD8232_ADC_FRAME_SIZE];

    (void)arg;

    while (!s_driver.stop_requested) {
        uint32_t bytes_read = 0;
        const esp_err_t ret = adc_continuous_read(s_driver.adc_handle, raw_buffer, sizeof(raw_buffer), &bytes_read, AD8232_READ_TIMEOUT_MS);

        if (ret == ESP_ERR_TIMEOUT) {
            continue;
        }

        if (ret == ESP_ERR_INVALID_STATE && s_driver.stop_requested) {
            break;
        }

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC continuous read failed: %s", esp_err_to_name(ret));
            continue;
        }

        for (uint32_t offset = 0; offset < bytes_read; offset += SOC_ADC_DIGI_RESULT_BYTES) {
            const adc_digi_output_data_t *sample = (const adc_digi_output_data_t *)&raw_buffer[offset];
            const uint32_t channel = sample->type1.channel;
            const uint16_t raw = sample->type1.data;

            if (channel >= SOC_ADC_CHANNEL_NUM(s_driver.adc_unit) || channel != (uint32_t)s_driver.adc_channel) {
                continue;
            }

            s_driver.decimator_sum += raw;
            s_driver.decimator_count++;

            if (s_driver.decimator_count < AD8232_DECIMATION_FACTOR) {
                continue;
            }

            {
                const uint16_t averaged_raw = (uint16_t)((s_driver.decimator_sum + (AD8232_DECIMATION_FACTOR / 2U)) / AD8232_DECIMATION_FACTOR);
                const bool lo_plus = gpio_get_level(s_driver.config.lo_plus_gpio) == 1;
                const bool lo_minus = gpio_get_level(s_driver.config.lo_minus_gpio) == 1;
                const bool lead_off = ad8232_update_lead_state(lo_plus, lo_minus);

                ad8232_reset_decimator();

                if (lead_off) {
                    ad8232_publish_invalid_sample((uint64_t)(esp_timer_get_time() / 1000), lo_plus, lo_minus);
                    continue;
                }

                if (s_driver.was_lead_off) {
                    s_driver.was_lead_off = false;
                    s_driver.reacquire_deadline_ms = (uint64_t)(esp_timer_get_time() / 1000) + AD8232_REACQUIRE_MS;
                    ad8232_reset_signal_chain();
                }

                ad8232_process_connected_sample(averaged_raw, lo_plus, lo_minus);
            }
        }
    }

    s_driver.task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t ad8232_init(const ad8232_config_t *config)
{
    const ad8232_config_t resolved_config = (config != NULL) ? *config : (ad8232_config_t)AD8232_DEFAULT_CONFIG();
    esp_err_t ret;

    if (s_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (resolved_config.output_gpio < 0 || resolved_config.lo_plus_gpio < 0 || resolved_config.lo_minus_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(adc_continuous_io_to_channel((int)resolved_config.output_gpio, &s_driver.adc_unit, &s_driver.adc_channel), TAG, "invalid AD8232 output GPIO");

    if (s_driver.adc_unit != AD8232_ADC_UNIT) {
        ESP_LOGE(TAG, "GPIO %d maps to ADC unit %d, but ESP32 continuous sampling requires ADC1", resolved_config.output_gpio, s_driver.adc_unit + 1);
        memset(&s_driver, 0, sizeof(s_driver));
        return ESP_ERR_INVALID_ARG;
    }

    s_driver.config = resolved_config;
    s_driver.sample_queue = xQueueCreate(AD8232_SAMPLE_QUEUE_LENGTH, sizeof(ad8232_sample_t));

    if (s_driver.sample_queue == NULL) {
        memset(&s_driver, 0, sizeof(s_driver));
        return ESP_ERR_NO_MEM;
    }

    ret = ad8232_setup_gpio(&resolved_config);
    if (ret != ESP_OK) {
        ad8232_cleanup_driver();
        return ret;
    }

    ret = ad8232_setup_adc();
    if (ret != ESP_OK) {
        ad8232_cleanup_driver();
        return ret;
    }

    ret = ad8232_calibration_init();
    if (ret != ESP_OK) {
        ad8232_cleanup_driver();
        return ret;
    }

    ad8232_reset_signal_chain();
    s_driver.was_lead_off = true;
    s_driver.initialized = true;

    ESP_LOGI(TAG,
             "ECG input GPIO %d (ADC1 channel %d), LO+ GPIO %d, LO- GPIO %d",
             resolved_config.output_gpio,
             s_driver.adc_channel,
             resolved_config.lo_plus_gpio,
             resolved_config.lo_minus_gpio);
    ESP_LOGI(TAG,
             "Sampling %u Hz raw -> %u Hz published, attenuation %d dB",
             AD8232_RAW_SAMPLE_RATE_HZ,
             AD8232_PUBLISHED_SAMPLE_RATE_HZ,
             12);

    return ESP_OK;
}

esp_err_t ad8232_start(void)
{
    if (!s_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_driver.started) {
        return ESP_ERR_INVALID_STATE;
    }

    s_driver.stop_requested = false;
    s_driver.reacquire_deadline_ms = (uint64_t)(esp_timer_get_time() / 1000) + AD8232_REACQUIRE_MS;

    ESP_RETURN_ON_ERROR(adc_continuous_start(s_driver.adc_handle), TAG, "failed to start ADC continuous mode");
    s_driver.started = true;

    if (xTaskCreate(ad8232_acquisition_task, "ad8232_task", AD8232_TASK_STACK_SIZE, NULL, AD8232_TASK_PRIORITY, &s_driver.task_handle) != pdPASS) {
        (void)adc_continuous_stop(s_driver.adc_handle);
        s_driver.started = false;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t ad8232_read_sample(ad8232_sample_t *out_sample, TickType_t timeout_ticks)
{
    if (!s_driver.initialized || !s_driver.started) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueueReceive(s_driver.sample_queue, out_sample, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t ad8232_stop(void)
{
    if (!s_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_driver.stop_requested = true;

    if (s_driver.started && s_driver.adc_handle != NULL) {
        const esp_err_t stop_ret = adc_continuous_stop(s_driver.adc_handle);

        if (stop_ret != ESP_OK && stop_ret != ESP_ERR_INVALID_STATE) {
            return stop_ret;
        }

        s_driver.started = false;
    }

    if (s_driver.task_handle != NULL) {
        for (int attempt = 0; attempt < 20 && s_driver.task_handle != NULL; ++attempt) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ad8232_cleanup_driver();

    return ESP_OK;
}
