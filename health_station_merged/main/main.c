#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "ad8232.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max30102.h"
#include "mlx906.h"
#include "weight.h"

// Set to 1 only when you want numeric-only ECG output for an online plotter.
#define APP_ENABLE_ECG_PLOTTER 0
// Set to 1 only when you want ECG status/logs on the main serial monitor.
#define APP_ENABLE_ECG_CONSOLE_LOGS 0

#define HX711_DOUT_GPIO GPIO_NUM_4
#define HX711_SCK_GPIO GPIO_NUM_5
#define HX711_TARE_SAMPLES 20
#define HX711_READ_SAMPLES 10
#define HX711_TARE_SETTLE_MS 3000
#define HX711_SCALE_COUNTS_PER_GRAM 0.0f
#define HX711_REPORT_PERIOD_MS 500

#define MLX906_REPORT_PERIOD_MS 1000

#define MAX30102_I2C_PORT I2C_NUM_1
#define MAX30102_SDA_GPIO GPIO_NUM_18
#define MAX30102_SCL_GPIO GPIO_NUM_19
#define MAX30102_INT_GPIO GPIO_NUM_27
#define MAX30102_I2C_CLK_HZ 400000
#define MAX30102_SAMPLE_RATE_HZ 200
#define MAX30102_BPM_WINDOW_MS 4000
#define MAX30102_BPM_WINDOW_SAMPLES ((MAX30102_SAMPLE_RATE_HZ * MAX30102_BPM_WINDOW_MS) / 1000)
#define MAX30102_COMPUTE_PERIOD_MS 500
#define MAX30102_COMPUTE_STEP_SAMPLES ((MAX30102_SAMPLE_RATE_HZ * MAX30102_COMPUTE_PERIOD_MS) / 1000)
#define MAX30102_PRINT_PERIOD_MS 1000
#define MAX30102_STABILIZE_MS 5000
#define MAX30102_FINGER_ON_IR_THRESHOLD 30000U
#define MAX30102_MIN_VALID_SPO2 70.0f

#define ECG_SUMMARY_PERIOD_MS 250

#define HX711_TASK_STACK_SIZE 4096
#define MLX906_TASK_STACK_SIZE 4096
#define MAX30102_TASK_STACK_SIZE 6144
#define AD8232_TASK_STACK_SIZE 4096
#define SENSOR_TASK_PRIORITY 4

static const char *TAG = "health_station";
static const char *HX711_TAG = "hx711_task";
static const char *MLX906_TAG = "mlx906_task";
static const char *MAX30102_TAG = "max30102_task";
static const char *AD8232_TAG = "ad8232_task";

static bool s_plotter_mode_enabled = APP_ENABLE_ECG_PLOTTER;
static bool s_ecg_console_logs_enabled = APP_ENABLE_ECG_CONSOLE_LOGS;
static bool s_logs_disabled_for_plotter = false;

static ho2_sample_t s_ring[MAX30102_BPM_WINDOW_SAMPLES];
static ho2_sample_t s_window[MAX30102_BPM_WINDOW_SAMPLES];

typedef enum {
    APP_WAITING_FOR_FINGER = 0,
    APP_STABILIZING,
    APP_STREAMING,
} max30102_app_state_t;

static void disable_logs_for_plotter(void)
{
    if (!s_plotter_mode_enabled || s_logs_disabled_for_plotter) {
        return;
    }

    esp_log_level_set("*", ESP_LOG_NONE);
    s_logs_disabled_for_plotter = true;
}

static void tare_with_plate(weight_handle_t *scale)
{
    ESP_LOGI(HX711_TAG, "Keep only the empty plate on the scale. Taring in %d ms...",
             HX711_TARE_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(HX711_TARE_SETTLE_MS));

    weight_tare(scale, HX711_TARE_SAMPLES);
    ESP_LOGI(HX711_TAG, "Tare complete. Offset=%ld", (long)weight_get_offset(scale));
}

static void weight_task(void *arg)
{
    weight_handle_t scale = {0};
    weight_config_t cfg = {
        .dout_gpio = HX711_DOUT_GPIO,
        .sck_gpio = HX711_SCK_GPIO,
        .gain = WEIGHT_GAIN_128,
        .data_ready_timeout_ms = 500,
    };
    const bool has_saved_scale = HX711_SCALE_COUNTS_PER_GRAM != 0.0f;

    (void)arg;

    weight_init(&scale, &cfg);
    tare_with_plate(&scale);

    if (has_saved_scale) {
        weight_set_scale(&scale, HX711_SCALE_COUNTS_PER_GRAM);
        ESP_LOGI(HX711_TAG, "Using stored scale factor %.6f counts/g",
                 weight_get_scale(&scale));
    } else {
        ESP_LOGW(HX711_TAG, "No calibration factor stored. Weight in grams is disabled.");
    }

    while (1) {
        int32_t raw = weight_read_average(&scale, HX711_READ_SAMPLES);
        int32_t delta = raw - weight_get_offset(&scale);

        if (has_saved_scale) {
            float grams = (float)delta / weight_get_scale(&scale);
            ESP_LOGI(HX711_TAG, "Raw=%ld Delta=%ld Weight=%.2f g",
                     (long)raw, (long)delta, grams);
        } else {
            ESP_LOGI(HX711_TAG, "Raw=%ld Delta=%ld (uncalibrated)",
                     (long)raw, (long)delta);
        }

        vTaskDelay(pdMS_TO_TICKS(HX711_REPORT_PERIOD_MS));
    }
}

static void mlx906_task(void *arg)
{
    esp_err_t err = ESP_OK;

    (void)arg;

    err = mlx906_init();
    if (err != ESP_OK) {
        ESP_LOGE(MLX906_TAG, "Failed to init MLX90614: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        float object_temp = 0.0f;
        float ambient_temp = 0.0f;

        err = mlx906_read_object_temp(&object_temp);
        if (err == ESP_OK) {
            err = mlx906_read_ambient_temp(&ambient_temp);
        }

        if (err == ESP_OK) {
            ESP_LOGI(MLX906_TAG, "Object=%.2f C Ambient=%.2f C",
                     object_temp, ambient_temp);
        } else {
            ESP_LOGW(MLX906_TAG, "Read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(MLX906_REPORT_PERIOD_MS));
    }
}

static void reset_max30102_capture_state(size_t *ring_index, size_t *ring_count,
                                         size_t *since_last, ho2_result_t *latest_result,
                                         bool *latest_result_valid, uint32_t *latest_result_ms,
                                         uint32_t *last_print_ms,
                                         uint32_t *last_printed_result_ms)
{
    *ring_index = 0;
    *ring_count = 0;
    *since_last = 0;
    *latest_result = (ho2_result_t){0};
    *latest_result_valid = false;
    *latest_result_ms = 0;
    *last_print_ms = 0;
    *last_printed_result_ms = 0;
}

static bool compute_max30102_latest_result(size_t ring_index, size_t ring_count,
                                           size_t *since_last, ho2_result_t *out)
{
    if (ring_count < MAX30102_BPM_WINDOW_SAMPLES ||
        *since_last < MAX30102_COMPUTE_STEP_SAMPLES) {
        return false;
    }

    for (size_t i = 0; i < MAX30102_BPM_WINDOW_SAMPLES; ++i) {
        s_window[i] = s_ring[(ring_index + i) % MAX30102_BPM_WINDOW_SAMPLES];
    }

    *since_last = 0;
    *out = (ho2_result_t){0};
    if (ho2_compute(s_window, MAX30102_BPM_WINDOW_SAMPLES, MAX30102_SAMPLE_RATE_HZ, out) !=
        ESP_OK) {
        return false;
    }

    return out->valid;
}

static bool max30102_result_is_publishable(const ho2_result_t *result)
{
    return result->valid && result->spo2_percent >= MAX30102_MIN_VALID_SPO2 &&
           result->spo2_percent <= 100.0f;
}

static void log_max30102_result(const ho2_result_t *result)
{
    ESP_LOGI(MAX30102_TAG, "SpO2 %.1f %% | HR %.1f bpm", result->spo2_percent,
             result->heart_rate_bpm);
}

static void max30102_task(void *arg)
{
    ho2_config_t cfg = {
        .i2c_port = MAX30102_I2C_PORT,
        .sda_io = MAX30102_SDA_GPIO,
        .scl_io = MAX30102_SCL_GPIO,
        .int_io = MAX30102_INT_GPIO,
        .i2c_clk_hz = MAX30102_I2C_CLK_HZ,
        .sample_rate = HO2_SR_200,
        .pulse_width = HO2_PW_411,
        .adc_range = HO2_ADC_16384,
        .led_current_red = 0x7F,
        .led_current_ir = 0x7F,
        .use_interrupt = true,
        .swap_red_ir = true,
    };
    max30102_app_state_t state = APP_WAITING_FOR_FINGER;
    size_t ring_index = 0;
    size_t ring_count = 0;
    size_t since_last = 0;
    uint32_t finger_on_since = 0;
    int last_countdown = 0;
    ho2_result_t latest_result = {0};
    bool latest_result_valid = false;
    uint32_t latest_result_ms = 0;
    uint32_t last_print_ms = 0;
    uint32_t last_printed_result_ms = 0;

    (void)arg;

    esp_err_t err = ho2_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(MAX30102_TAG, "Failed to init MAX30102: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(MAX30102_TAG, "Place finger on sensor.");

    while (1) {
        ho2_sample_t fifo_buf[32];
        size_t read = 0;

        (void)ho2_wait_for_data(pdMS_TO_TICKS(200));

        err = ho2_read_fifo(fifo_buf, 32, &read);
        if (err != ESP_OK) {
            ESP_LOGE(MAX30102_TAG, "FIFO read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (read == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        for (size_t i = 0; i < read; ++i) {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            bool finger_on = fifo_buf[i].ir >= MAX30102_FINGER_ON_IR_THRESHOLD;

            if (!finger_on) {
                if (state != APP_WAITING_FOR_FINGER) {
                    ESP_LOGW(MAX30102_TAG, "Finger removed, place finger.");
                }

                state = APP_WAITING_FOR_FINGER;
                finger_on_since = 0;
                last_countdown = 0;
                reset_max30102_capture_state(&ring_index, &ring_count, &since_last,
                                             &latest_result, &latest_result_valid,
                                             &latest_result_ms, &last_print_ms,
                                             &last_printed_result_ms);
                continue;
            }

            if (state == APP_WAITING_FOR_FINGER) {
                reset_max30102_capture_state(&ring_index, &ring_count, &since_last,
                                             &latest_result, &latest_result_valid,
                                             &latest_result_ms, &last_print_ms,
                                             &last_printed_result_ms);
                state = APP_STABILIZING;
                finger_on_since = now_ms;
                last_countdown = 5;
                ESP_LOGI(MAX30102_TAG, "Stabilizing... %d", last_countdown);
            }

            s_ring[ring_index] = fifo_buf[i];
            ring_index = (ring_index + 1) % MAX30102_BPM_WINDOW_SAMPLES;
            if (ring_count < MAX30102_BPM_WINDOW_SAMPLES) {
                ring_count++;
            }
            since_last++;

            {
                ho2_result_t result = {0};

                if (compute_max30102_latest_result(ring_index, ring_count, &since_last,
                                                   &result) &&
                    max30102_result_is_publishable(&result)) {
                    latest_result = result;
                    latest_result_valid = true;
                    latest_result_ms = now_ms;
                }
            }

            if (state == APP_STABILIZING) {
                uint32_t elapsed_ms = now_ms - finger_on_since;
                if (elapsed_ms >= MAX30102_STABILIZE_MS) {
                    state = APP_STREAMING;
                    if (latest_result_valid &&
                        (now_ms - latest_result_ms) <= MAX30102_PRINT_PERIOD_MS) {
                        log_max30102_result(&latest_result);
                        last_print_ms = now_ms;
                        last_printed_result_ms = latest_result_ms;
                    }
                    continue;
                }

                {
                    int countdown =
                        (int)((MAX30102_STABILIZE_MS - elapsed_ms + 999U) / 1000U);
                    if (countdown < 1) {
                        countdown = 1;
                    }
                    if (countdown < last_countdown) {
                        last_countdown = countdown;
                        ESP_LOGI(MAX30102_TAG, "Stabilizing... %d", last_countdown);
                    }
                }
            }

            if (state == APP_STREAMING && latest_result_valid &&
                latest_result_ms != last_printed_result_ms &&
                (last_print_ms == 0 ||
                 (now_ms - last_print_ms) >= MAX30102_PRINT_PERIOD_MS)) {
                log_max30102_result(&latest_result);
                last_print_ms = now_ms;
                last_printed_result_ms = latest_result_ms;
            }
        }
    }
}

static void ad8232_task(void *arg)
{
    const ad8232_config_t config = AD8232_DEFAULT_CONFIG();
    ad8232_sample_t sample = {0};
    uint32_t last_summary_ms = 0;

    (void)arg;

    if (s_plotter_mode_enabled) {
        setvbuf(stdout, NULL, _IONBF, 0);
    }

    esp_err_t err = ad8232_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(AD8232_TAG, "Failed to init AD8232: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    err = ad8232_start();
    if (err != ESP_OK) {
        ESP_LOGE(AD8232_TAG, "Failed to start AD8232: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    if (s_plotter_mode_enabled) {
        ESP_LOGI(AD8232_TAG, "AD8232 acquisition started in plotter mode");
    } else if (s_ecg_console_logs_enabled) {
        ESP_LOGI(AD8232_TAG, "AD8232 acquisition started with console logs enabled");
    }

    if (s_plotter_mode_enabled) {
        disable_logs_for_plotter();
    }

    while (1) {
        err = ad8232_read_sample(&sample, portMAX_DELAY);
        if (err != ESP_OK) {
            if (!s_plotter_mode_enabled && s_ecg_console_logs_enabled) {
                ESP_LOGW(AD8232_TAG, "Read failed: %s", esp_err_to_name(err));
            }
            continue;
        }

        if (s_plotter_mode_enabled) {
            if (!sample.valid) {
                printf("0\n");
                continue;
            }

            printf("%.3f\n", (double)sample.ecg_uv / 1000.0);
            continue;
        }

        if (!s_ecg_console_logs_enabled) {
            continue;
        }

        if (sample.valid) {
            uint32_t now_ms = (uint32_t)(sample.timestamp_ms & 0xFFFFFFFFU);
            if ((now_ms - last_summary_ms) >= ECG_SUMMARY_PERIOD_MS) {
                last_summary_ms = now_ms;
                ESP_LOGI(AD8232_TAG, "ECG=%.3f mV raw=%u clipped=%d",
                         (double)sample.ecg_uv / 1000.0, sample.raw, sample.clipped);
            }
        } else {
            ESP_LOGW(AD8232_TAG, "Lead off detected (LO+=%d LO-=%d)", sample.lo_plus,
                     sample.lo_minus);
        }
    }
}

static void create_task_or_abort(TaskFunction_t task_fn, const char *name, uint32_t stack_size)
{
    BaseType_t result = xTaskCreate(task_fn, name, stack_size, NULL, SENSOR_TASK_PRIORITY, NULL);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task %s", name);
        abort();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting merged sensor coordinator");
    ESP_LOGI(TAG, "ECG output: %s",
             s_plotter_mode_enabled ? "plotter" :
             (s_ecg_console_logs_enabled ? "console logs" : "silent"));

    create_task_or_abort(weight_task, "weight_task", HX711_TASK_STACK_SIZE);
    create_task_or_abort(mlx906_task, "mlx906_task", MLX906_TASK_STACK_SIZE);
    create_task_or_abort(max30102_task, "max30102_task", MAX30102_TASK_STACK_SIZE);
    create_task_or_abort(ad8232_task, "ad8232_task", AD8232_TASK_STACK_SIZE);

    vTaskDelete(NULL);
}
