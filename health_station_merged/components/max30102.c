// MAX30102 heart-rate + SpO2 driver (basic demo-grade processing)
#include "max30102.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define HO2_I2C_ADDR 0x57
#define HO2_I2C_TIMEOUT_MS 100

// Register map
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12

#define MODE_RESET 0x40
#define MODE_SPO2 0x03
#define HO2_MAX_BPM_SAMPLES 1024
#define HO2_MAX_PEAKS 32

static const char *TAG = "ho2";

static i2c_port_t s_i2c_port = I2C_NUM_0;
static bool s_inited = false;
static bool s_use_interrupt = false;
static bool s_swap_red_ir = false;
static SemaphoreHandle_t s_int_sem = NULL;
static float s_bpm_stage1[HO2_MAX_BPM_SAMPLES];
static float s_bpm_stage2[HO2_MAX_BPM_SAMPLES];
static int s_bpm_peak_indices[HO2_MAX_PEAKS];
static int s_bpm_intervals[HO2_MAX_PEAKS];

static esp_err_t ho2_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(s_i2c_port, HO2_I2C_ADDR, data, sizeof(data),
                                      pdMS_TO_TICKS(HO2_I2C_TIMEOUT_MS));
}

static esp_err_t ho2_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(s_i2c_port, HO2_I2C_ADDR, &reg, 1, val, 1,
                                        pdMS_TO_TICKS(HO2_I2C_TIMEOUT_MS));
}

static esp_err_t ho2_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(s_i2c_port, HO2_I2C_ADDR, &reg, 1, buf, len,
                                        pdMS_TO_TICKS(HO2_I2C_TIMEOUT_MS));
}

static void IRAM_ATTR ho2_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher = pdFALSE;
    if (s_int_sem != NULL) {
        xSemaphoreGiveFromISR(s_int_sem, &higher);
    }
    if (higher == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

uint16_t ho2_sample_rate_hz(ho2_sample_rate_t rate)
{
    switch (rate) {
    case HO2_SR_50:
        return 50;
    case HO2_SR_100:
        return 100;
    case HO2_SR_200:
        return 200;
    case HO2_SR_400:
        return 400;
    case HO2_SR_800:
        return 800;
    case HO2_SR_1000:
        return 1000;
    case HO2_SR_1600:
        return 1600;
    case HO2_SR_3200:
        return 3200;
    default:
        return 100;
    }
}

esp_err_t ho2_init(const ho2_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_port = cfg->i2c_port;
    s_use_interrupt = cfg->use_interrupt && (cfg->int_io != HO2_GPIO_UNUSED);
    s_swap_red_ir = cfg->swap_red_ir;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda_io,
        .scl_io_num = cfg->scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_clk_hz,
    };
    ESP_ERROR_CHECK(i2c_param_config(s_i2c_port, &conf));
    esp_err_t i2c_err = i2c_driver_install(s_i2c_port, conf.mode, 0, 0, 0);
    if (i2c_err != ESP_OK && i2c_err != ESP_ERR_INVALID_STATE) {
        return i2c_err;
    }

    if (s_use_interrupt) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << cfg->int_io,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        esp_err_t isr = gpio_install_isr_service(0);
        if (isr != ESP_OK && isr != ESP_ERR_INVALID_STATE) {
            return isr;
        }
        if (s_int_sem == NULL) {
            s_int_sem = xSemaphoreCreateBinary();
        }
        ESP_ERROR_CHECK(gpio_isr_handler_add(cfg->int_io, ho2_isr_handler, NULL));
    }

    // Reset
    ESP_ERROR_CHECK(ho2_write_reg(REG_MODE_CONFIG, MODE_RESET));
    vTaskDelay(pdMS_TO_TICKS(20));

    // Clear interrupts
    uint8_t dummy;
    (void)ho2_read_reg(REG_INTR_STATUS_1, &dummy);
    (void)ho2_read_reg(REG_INTR_STATUS_2, &dummy);

    // FIFO: sample average = 1, rollover enabled, almost full = 0x0F.
    // Using raw FIFO samples keeps the effective output rate aligned with the
    // app's 200 Hz timing assumptions.
    ESP_ERROR_CHECK(ho2_write_reg(REG_FIFO_CONFIG, 0x1F));
    ESP_ERROR_CHECK(ho2_write_reg(REG_FIFO_WR_PTR, 0x00));
    ESP_ERROR_CHECK(ho2_write_reg(REG_OVF_COUNTER, 0x00));
    ESP_ERROR_CHECK(ho2_write_reg(REG_FIFO_RD_PTR, 0x00));

    // Mode and SpO2 configuration
    ESP_ERROR_CHECK(ho2_write_reg(REG_MODE_CONFIG, MODE_SPO2));
    uint8_t spo2_cfg = ((uint8_t)cfg->adc_range << 5) | ((uint8_t)cfg->sample_rate << 2) |
                       ((uint8_t)cfg->pulse_width);
    ESP_ERROR_CHECK(ho2_write_reg(REG_SPO2_CONFIG, spo2_cfg));

    // LED currents
    ESP_ERROR_CHECK(ho2_write_reg(REG_LED1_PA, cfg->led_current_red));
    ESP_ERROR_CHECK(ho2_write_reg(REG_LED2_PA, cfg->led_current_ir));

    // Multi-LED slots: Slot1=RED, Slot2=IR
    ESP_ERROR_CHECK(ho2_write_reg(REG_MULTI_LED_CTRL1, 0x21));
    ESP_ERROR_CHECK(ho2_write_reg(REG_MULTI_LED_CTRL2, 0x00));

    // Enable PPG ready interrupt
    ESP_ERROR_CHECK(ho2_write_reg(REG_INTR_ENABLE_1, 0x40));
    ESP_ERROR_CHECK(ho2_write_reg(REG_INTR_ENABLE_2, 0x00));

    s_inited = true;
    ESP_LOGI(TAG, "MAX30102 initialized");
    return ESP_OK;
}

esp_err_t ho2_wait_for_data(TickType_t timeout)
{
    if (!s_use_interrupt || s_int_sem == NULL) {
        if (timeout > 0) {
            vTaskDelay(timeout);
        }
        return ESP_OK;
    }
    if (xSemaphoreTake(s_int_sem, timeout) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t ho2_read_fifo(ho2_sample_t *samples, size_t max_samples, size_t *samples_read)
{
    if (!s_inited || samples == NULL || samples_read == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *samples_read = 0;

    uint8_t intr1 = 0;
    uint8_t intr2 = 0;
    (void)ho2_read_reg(REG_INTR_STATUS_1, &intr1);
    (void)ho2_read_reg(REG_INTR_STATUS_2, &intr2);

    uint8_t wr_ptr = 0;
    uint8_t rd_ptr = 0;
    ESP_ERROR_CHECK(ho2_read_reg(REG_FIFO_WR_PTR, &wr_ptr));
    ESP_ERROR_CHECK(ho2_read_reg(REG_FIFO_RD_PTR, &rd_ptr));

    uint8_t count = (wr_ptr - rd_ptr) & 0x1F;
    if (count == 0) {
        return ESP_OK;
    }

    size_t to_read = count;
    if (to_read > max_samples) {
        to_read = max_samples;
    }

    uint8_t fifo_data[6 * 32];
    size_t bytes = to_read * 6;
    if (bytes > sizeof(fifo_data)) {
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_ERROR_CHECK(ho2_read_bytes(REG_FIFO_DATA, fifo_data, bytes));

    for (size_t i = 0; i < to_read; ++i) {
        size_t idx = i * 6;
        uint32_t red = ((uint32_t)fifo_data[idx] << 16) | ((uint32_t)fifo_data[idx + 1] << 8) |
                       (uint32_t)fifo_data[idx + 2];
        uint32_t ir = ((uint32_t)fifo_data[idx + 3] << 16) | ((uint32_t)fifo_data[idx + 4] << 8) |
                      (uint32_t)fifo_data[idx + 5];
        red &= 0x3FFFF;
        ir &= 0x3FFFF;
        if (s_swap_red_ir) {
            samples[i].red = ir;
            samples[i].ir = red;
        } else {
            samples[i].red = red;
            samples[i].ir = ir;
        }
    }

    *samples_read = to_read;
    return ESP_OK;
}

static double ho2_moving_avg_u32(const ho2_sample_t *samples, size_t i, size_t window,
                                 bool use_red)
{
    size_t start = (i >= window - 1) ? (i - (window - 1)) : 0;
    double sum = 0.0;
    size_t count = 0;
    for (size_t j = start; j <= i; ++j) {
        sum += use_red ? samples[j].red : samples[j].ir;
        count++;
    }
    return (count > 0) ? (sum / (double)count) : 0.0;
}

static float ho2_moving_avg_f32(const float *values, size_t i, size_t window)
{
    size_t start = (i >= window - 1) ? (i - (window - 1)) : 0;
    float sum = 0.0f;
    size_t count = 0;
    for (size_t j = start; j <= i; ++j) {
        sum += values[j];
        count++;
    }
    return (count > 0) ? (sum / (float)count) : 0.0f;
}

static int ho2_median_i32(const int *values, int count)
{
    if (count <= 0 || count > HO2_MAX_PEAKS) {
        return 0;
    }

    int sorted[HO2_MAX_PEAKS];
    for (int i = 0; i < count; ++i) {
        sorted[i] = values[i];
    }

    for (int i = 1; i < count; ++i) {
        int value = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > value) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = value;
    }

    if ((count % 2) == 1) {
        return sorted[count / 2];
    }
    return (sorted[count / 2 - 1] + sorted[count / 2]) / 2;
}

static float ho2_compute_spo2(const ho2_sample_t *samples, size_t n, float *red_dc, float *ir_dc)
{
    const size_t window = 4;
    double sum_red = 0.0;
    double sum_ir = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sum_red += ho2_moving_avg_u32(samples, i, window, true);
        sum_ir += ho2_moving_avg_u32(samples, i, window, false);
    }
    double mean_red = sum_red / (double)n;
    double mean_ir = sum_ir / (double)n;

    double ac_red = 0.0;
    double ac_ir = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double dr = ho2_moving_avg_u32(samples, i, window, true) - mean_red;
        double di = ho2_moving_avg_u32(samples, i, window, false) - mean_ir;
        ac_red += dr * dr;
        ac_ir += di * di;
    }
    ac_red = sqrt(ac_red / (double)n);
    ac_ir = sqrt(ac_ir / (double)n);

    if (red_dc) {
        *red_dc = (float)mean_red;
    }
    if (ir_dc) {
        *ir_dc = (float)mean_ir;
    }

    if (mean_red <= 0.0 || mean_ir <= 0.0 || ac_red <= 0.0 || ac_ir <= 0.0) {
        return NAN;
    }

    double ratio = (ac_red / mean_red) / (ac_ir / mean_ir);
    double spo2 = -45.060 * ratio * ratio + 30.354 * ratio + 94.845;

    if (spo2 < 0.0) {
        spo2 = 0.0;
    } else if (spo2 > 100.0) {
        spo2 = 100.0;
    }

    return (float)spo2;
}

static float ho2_compute_bpm(const ho2_sample_t *samples, size_t n, float sample_rate_hz)
{
    if (sample_rate_hz <= 0.0f || n < 10) {
        return NAN;
    }

    if (n > HO2_MAX_BPM_SAMPLES) {
        return NAN;
    }

    const size_t ma_window = 5;
    const size_t smooth_window = 9;
    double sum = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double v = ho2_moving_avg_u32(samples, i, ma_window, false);
        s_bpm_stage1[i] = (float)v;
        sum += v;
    }

    float mean = (float)(sum / (double)n);
    float dc_level = fabsf(mean);
    if (dc_level <= 1.0f) {
        return NAN;
    }

    for (size_t i = 0; i < n; ++i) {
        s_bpm_stage1[i] -= mean;
    }

    float min_v = 0.0f;
    float max_v = 0.0f;
    double energy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        s_bpm_stage2[i] = ho2_moving_avg_f32(s_bpm_stage1, i, smooth_window);
        if (i == 0) {
            min_v = s_bpm_stage2[i];
            max_v = s_bpm_stage2[i];
        } else {
            if (s_bpm_stage2[i] < min_v) {
                min_v = s_bpm_stage2[i];
            }
            if (s_bpm_stage2[i] > max_v) {
                max_v = s_bpm_stage2[i];
            }
        }
        energy += (double)s_bpm_stage2[i] * (double)s_bpm_stage2[i];
    }

    if (energy <= 1e-6) {
        return NAN;
    }

    float rms = (float)sqrt(energy / (double)n);
    float peak_to_peak = max_v - min_v;
    if (rms < dc_level * 0.0025f || peak_to_peak < dc_level * 0.01f) {
        return NAN;
    }

    int peak_count = 0;
    int min_gap = (int)(sample_rate_hz * 0.30f);
    if (min_gap < 1) {
        min_gap = 1;
    }
    int prominence_radius = min_gap / 2;
    float min_prominence = 0.35f * rms;

    for (size_t i = 1; i + 1 < n; ++i) {
        float current = s_bpm_stage2[i];
        if (current <= 0.0f || current <= s_bpm_stage2[i - 1] || current < s_bpm_stage2[i + 1]) {
            continue;
        }

        size_t left_start = (i > (size_t)prominence_radius) ? (i - (size_t)prominence_radius) : 0;
        size_t right_end = ((i + (size_t)prominence_radius) < n) ? (i + (size_t)prominence_radius)
                                                                  : (n - 1);
        float left_min = current;
        float right_min = current;
        for (size_t j = left_start; j <= i; ++j) {
            if (s_bpm_stage2[j] < left_min) {
                left_min = s_bpm_stage2[j];
            }
        }
        for (size_t j = i; j <= right_end; ++j) {
            if (s_bpm_stage2[j] < right_min) {
                right_min = s_bpm_stage2[j];
            }
        }

        float baseline = (left_min > right_min) ? left_min : right_min;
        float prominence = current - baseline;
        if (prominence < min_prominence) {
            continue;
        }

        if (peak_count > 0) {
            int gap = (int)i - s_bpm_peak_indices[peak_count - 1];
            if (gap < min_gap) {
                size_t prev_index = (size_t)s_bpm_peak_indices[peak_count - 1];
                if (current > s_bpm_stage2[prev_index]) {
                    s_bpm_peak_indices[peak_count - 1] = (int)i;
                }
                continue;
            }
        }

        if (peak_count >= HO2_MAX_PEAKS) {
            break;
        }
        s_bpm_peak_indices[peak_count++] = (int)i;
    }

    if (peak_count < 3) {
        return NAN;
    }

    int interval_count = 0;
    for (int i = 1; i < peak_count; ++i) {
        s_bpm_intervals[interval_count++] = s_bpm_peak_indices[i] - s_bpm_peak_indices[i - 1];
    }

    int median_interval = ho2_median_i32(s_bpm_intervals, interval_count);
    if (median_interval <= 0) {
        return NAN;
    }

    int consistent_count = 0;
    for (int i = 0; i < interval_count; ++i) {
        float delta = fabsf((float)(s_bpm_intervals[i] - median_interval)) / (float)median_interval;
        if (delta <= 0.20f) {
            s_bpm_intervals[consistent_count++] = s_bpm_intervals[i];
        }
    }

    if (consistent_count < 2) {
        return NAN;
    }

    median_interval = ho2_median_i32(s_bpm_intervals, consistent_count);
    if (median_interval <= 0) {
        return NAN;
    }

    float bpm = 60.0f * sample_rate_hz / (float)median_interval;
    if (bpm < 45.0f || bpm > 160.0f) {
        return NAN;
    }
    return bpm;
}

esp_err_t ho2_compute(const ho2_sample_t *samples, size_t n, float sample_rate_hz, ho2_result_t *out)
{
    if (samples == NULL || out == NULL || n < 10) {
        return ESP_ERR_INVALID_ARG;
    }

    float red_dc = 0.0f;
    float ir_dc = 0.0f;
    float spo2 = ho2_compute_spo2(samples, n, &red_dc, &ir_dc);
    float bpm = ho2_compute_bpm(samples, n, sample_rate_hz);

    out->red_dc = red_dc;
    out->ir_dc = ir_dc;
    out->spo2_percent = spo2;
    out->heart_rate_bpm = bpm;
    out->valid = !(isnan(spo2) || isnan(bpm));

    return ESP_OK;
}

void ho2_algo_init(ho2_algo_t *state, uint32_t now_ms)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->num_for_spo2 = 100;
    state->estimated_spo2 = 95.0f;
    state->frate = 0.95;
    state->f_spo2 = 0.7;
    state->finger_on_threshold = 30000;
    state->boot_time_ms = now_ms + 1000;
}

bool ho2_algo_update(ho2_algo_t *state, uint32_t red, uint32_t ir, uint32_t now_ms,
                     float *spo2_out, bool *finger_on)
{
    if (state == NULL) {
        return false;
    }

    if (finger_on != NULL) {
        *finger_on = (ir >= state->finger_on_threshold);
    }

    state->sample_count++;
    double fred = (double)red;
    double fir = (double)ir;
    state->avered = state->avered * state->frate + fred * (1.0 - state->frate);
    state->aveir = state->aveir * state->frate + fir * (1.0 - state->frate);
    double red_ac = fred - state->avered;
    double ir_ac = fir - state->aveir;
    state->sumredrms += red_ac * red_ac;
    state->sumirrms += ir_ac * ir_ac;

    if (state->sample_count % state->num_for_spo2 != 0) {
        return false;
    }

    if (now_ms < state->boot_time_ms) {
        state->sumredrms = 0.0;
        state->sumirrms = 0.0;
        state->sample_count = 0;
        return false;
    }

    if (state->aveir <= 0.0 || state->sumirrms <= 0.0) {
        state->sumredrms = 0.0;
        state->sumirrms = 0.0;
        state->sample_count = 0;
        return false;
    }

    double r = (sqrt(state->sumredrms) / state->avered) / (sqrt(state->sumirrms) / state->aveir);
    double spo2 = -23.3 * (r - 0.4) + 100.0;
    state->estimated_spo2 = (float)(state->f_spo2 * state->estimated_spo2 +
                                    (1.0 - state->f_spo2) * spo2);

    if (spo2_out != NULL) {
        *spo2_out = state->estimated_spo2;
    }

    state->sumredrms = 0.0;
    state->sumirrms = 0.0;
    state->sample_count = 0;
    return true;
}
