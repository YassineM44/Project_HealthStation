#include "weight.h"

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "weight";

/* Spinlock to protect HX711 bit-bang timing from preemption.
 * If a higher-priority task preempts during the 24-bit clock sequence
 * and SCK stays HIGH for >60µs, the HX711 enters power-down mode. */
static portMUX_TYPE s_hx711_mux = portMUX_INITIALIZER_UNLOCKED;

#define HX711_READY_TIMEOUT_US 200000ULL
#define HX711_CLOCK_PULSE_US 1U
#define HX711_GAIN_PULSES 1U
#define HX711_MAX_SAMPLE_BUFFER 64U
#define HALF_WINDOW (WEIGHT_HISTORY_SIZE / 2U)
#define WEIGHT_CLEAR_THRESHOLD_DIVISOR 2U

static esp_err_t hx711_wait_ready(const weight_sensor_t *sensor,
                                  uint32_t timeout_us) {
  int64_t deadline_us = esp_timer_get_time() + timeout_us;
  while (gpio_get_level(sensor->config.dout_gpio) != 0) {
    if (esp_timer_get_time() >= deadline_us)
      return ESP_ERR_TIMEOUT;
    esp_rom_delay_us(50);
  }
  return ESP_OK;
}

static int32_t hx711_sign_extend_24bit(uint32_t value) {
  if ((value & 0x00800000U) != 0U)
    value |= 0xFF000000U;
  return (int32_t)value;
}

static esp_err_t hx711_read_raw(weight_sensor_t *sensor, int32_t *raw_value) {
  esp_err_t err = hx711_wait_ready(sensor, HX711_READY_TIMEOUT_US);
  uint32_t value = 0;
  if (err != ESP_OK)
    return err;

  /* Critical section: disable interrupts during bit-bang (~50µs total).
   * Prevents higher-priority tasks from preempting mid-clock and
   * causing SCK HIGH >60µs which powers down the HX711. */
  portENTER_CRITICAL(&s_hx711_mux);

  for (size_t bit = 0; bit < 24; ++bit) {
    gpio_set_level(sensor->config.sck_gpio, 1);
    esp_rom_delay_us(HX711_CLOCK_PULSE_US);
    value = (value << 1U) | (uint32_t)gpio_get_level(sensor->config.dout_gpio);
    gpio_set_level(sensor->config.sck_gpio, 0);
    esp_rom_delay_us(HX711_CLOCK_PULSE_US);
  }

  for (size_t pulse = 0; pulse < HX711_GAIN_PULSES; ++pulse) {
    gpio_set_level(sensor->config.sck_gpio, 1);
    esp_rom_delay_us(HX711_CLOCK_PULSE_US);
    gpio_set_level(sensor->config.sck_gpio, 0);
    esp_rom_delay_us(HX711_CLOCK_PULSE_US);
  }

  portEXIT_CRITICAL(&s_hx711_mux);

  *raw_value = hx711_sign_extend_24bit(value);
  return ESP_OK;
}

static esp_err_t hx711_average_samples(weight_sensor_t *sensor,
                                       uint16_t sample_count,
                                       int32_t *average) {
  int32_t samples[HX711_MAX_SAMPLE_BUFFER] = {0};
  int64_t sum = 0;
  uint16_t trim_count = 0;
  uint16_t start_index = 0;
  uint16_t end_index = 0;

  if ((sample_count == 0U) || (average == NULL) ||
      (sample_count > HX711_MAX_SAMPLE_BUFFER)) {
    return ESP_ERR_INVALID_ARG;
  }

  for (uint16_t index = 0; index < sample_count; ++index) {
    int32_t sample = 0;
    esp_err_t err = hx711_read_raw(sensor, &sample);
    if (err != ESP_OK)
      return err;
    samples[index] = sample;
    /* Yield to prevent Task Watchdog timeout during long averages/tare */
    vTaskDelay(1);
  }

  for (uint16_t i = 1U; i < sample_count; ++i) {
    int32_t key = samples[i];
    int32_t j = (int32_t)i - 1;
    while ((j >= 0) && (samples[j] > key)) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  if (sample_count >= 8U) {
    trim_count = sample_count / 4U;
  } else if (sample_count > 2U) {
    trim_count = 1U;
  }

  start_index = trim_count;
  end_index = sample_count - trim_count;
  if (start_index >= end_index) {
    start_index = 0U;
    end_index = sample_count;
  }

  for (uint16_t index = start_index; index < end_index; ++index) {
    sum += samples[index];
  }

  *average = (int32_t)(sum / (int64_t)(end_index - start_index));
  return ESP_OK;
}

/*
 * Compute the average of a half-window of readings from the circular buffer.
 * recent=true  → most recent HALF_WINDOW readings
 * recent=false → older HALF_WINDOW readings
 */
static int32_t weight_half_window_avg(const weight_sensor_t *sensor,
                                      bool recent) {
  int64_t sum = 0;
  uint8_t start_offset = recent ? 0 : HALF_WINDOW;

  for (uint8_t i = 0; i < HALF_WINDOW; i++) {
    /* history_index points to where the NEXT write goes,
     * so most recent is at (history_index - 1), etc. */
    uint8_t idx =
        (sensor->history_index + WEIGHT_HISTORY_SIZE - 1 - start_offset - i) %
        WEIGHT_HISTORY_SIZE;
    sum += sensor->history[idx];
  }

  return (int32_t)(sum / HALF_WINDOW);
}

static int32_t weight_apply_present_sign(const weight_sensor_t *sensor,
                                         int32_t delta) {
  if (sensor->config.present_sign == 1)
    return delta;
  if (sensor->config.present_sign == -1)
    return -delta;
  return (delta >= 0) ? delta : -delta;
}

static uint32_t weight_clear_threshold(const weight_sensor_t *sensor) {
  uint32_t clear_threshold =
      sensor->config.present_threshold_raw / WEIGHT_CLEAR_THRESHOLD_DIVISOR;
  return (clear_threshold > 0U) ? clear_threshold : 1U;
}

static void weight_seed_history(weight_sensor_t *sensor, int32_t value) {
  for (uint8_t i = 0; i < WEIGHT_HISTORY_SIZE; ++i) {
    sensor->history[i] = value;
  }
  sensor->history_index = 0;
  sensor->history_count = WEIGHT_HISTORY_SIZE;
}

esp_err_t weight_init(weight_sensor_t *sensor, const weight_config_t *config) {
  gpio_config_t dout_config = {0};
  gpio_config_t sck_config = {0};
  esp_err_t err;

  if ((sensor == NULL) || (config == NULL))
    return ESP_ERR_INVALID_ARG;
  if (!GPIO_IS_VALID_GPIO(config->dout_gpio) ||
      !GPIO_IS_VALID_GPIO(config->sck_gpio))
    return ESP_ERR_INVALID_ARG;
  if ((config->samples_per_update == 0U) ||
      (config->settle_reads_required == 0U))
    return ESP_ERR_INVALID_ARG;
  if ((config->present_sign < -1) || (config->present_sign > 1))
    return ESP_ERR_INVALID_ARG;

  memset(sensor, 0, sizeof(*sensor));
  sensor->config = *config;

  dout_config.pin_bit_mask = 1ULL << config->dout_gpio;
  dout_config.mode = GPIO_MODE_INPUT;
  dout_config.pull_up_en = GPIO_PULLUP_DISABLE;
  dout_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  dout_config.intr_type = GPIO_INTR_DISABLE;
  err = gpio_config(&dout_config);
  if (err != ESP_OK)
    return err;

  sck_config.pin_bit_mask = 1ULL << config->sck_gpio;
  sck_config.mode = GPIO_MODE_OUTPUT;
  sck_config.pull_up_en = GPIO_PULLUP_DISABLE;
  sck_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  sck_config.intr_type = GPIO_INTR_DISABLE;
  err = gpio_config(&sck_config);
  if (err != ESP_OK)
    return err;

  err = gpio_set_level(config->sck_gpio, 0);
  if (err != ESP_OK)
    return err;

  sensor->initialized = true;
  ESP_LOGI(TAG, "HX711 init DOUT=%d SCK=%d thresh=%" PRIu32, config->dout_gpio,
           config->sck_gpio, config->present_threshold_raw);
  return ESP_OK;
}

esp_err_t weight_tare(weight_sensor_t *sensor, uint16_t sample_count) {
  int32_t baseline = 0;
  if ((sensor == NULL) || !sensor->initialized || (sample_count == 0U))
    return ESP_ERR_INVALID_ARG;

  esp_err_t err = hx711_average_samples(sensor, sample_count, &baseline);
  if (err != ESP_OK)
    return err;

  sensor->tare_raw = baseline;
  weight_seed_history(sensor, baseline);
  sensor->pill_present = false;
  sensor->streak = 0;
  sensor->clear_streak = 0;
  sensor->tared = true;

  ESP_LOGD(TAG, "Tare complete: baseline=%" PRId32, baseline);
  return ESP_OK;
}

esp_err_t weight_update(weight_sensor_t *sensor, weight_status_t *status) {
  int32_t raw_average = 0;
  int32_t recent_avg = 0;
  int32_t older_avg = 0;
  int32_t active_delta = 0;
  int32_t level_delta = 0;
  uint32_t thresh = 0;

  if ((sensor == NULL) || (status == NULL))
    return ESP_ERR_INVALID_ARG;
  if (!sensor->initialized || !sensor->tared)
    return ESP_ERR_INVALID_STATE;

  esp_err_t err = hx711_average_samples(
      sensor, sensor->config.samples_per_update, &raw_average);
  if (err != ESP_OK)
    return err;

  /* Push reading into circular buffer */
  sensor->history[sensor->history_index] = raw_average;
  sensor->history_index = (sensor->history_index + 1) % WEIGHT_HISTORY_SIZE;
  if (sensor->history_count < WEIGHT_HISTORY_SIZE) {
    sensor->history_count++;
  }

  thresh = sensor->config.present_threshold_raw;
  level_delta =
      weight_apply_present_sign(sensor, raw_average - sensor->tare_raw);

  /* Once pills are present, clear quickly when the filtered reading settles
   * back near the tare baseline instead of waiting for the full window shift.
   */
  if (sensor->pill_present) {
    uint32_t clear_threshold = weight_clear_threshold(sensor);
    if (level_delta <= (int32_t)clear_threshold) {
      sensor->clear_streak++;
      if (sensor->clear_streak >= sensor->config.settle_reads_required) {
        sensor->pill_present = false;
        sensor->streak = 0;
        sensor->clear_streak = 0;
        weight_seed_history(sensor, raw_average);
        ESP_LOGI(TAG,
                 "State changed: pill=false (level=%" PRId32 ", clear=%" PRIu32
                 ")",
                 level_delta, clear_threshold);
      }
    } else {
      sensor->clear_streak = 0;
    }
  } else {
    sensor->clear_streak = 0;
  }

  /* Need full buffer before comparing windows */
  if (sensor->history_count < WEIGHT_HISTORY_SIZE) {
    status->pill_present = sensor->pill_present;
    status->tared = sensor->tared;
    status->raw_average = raw_average;
    status->recent_avg = 0;
    status->older_avg = 0;
    status->window_delta = 0;
    status->history_count = sensor->history_count;
    return ESP_OK;
  }

  /*
   * SLIDING WINDOW COMPARISON
   *
   * Split the 16-reading circular buffer into two halves:
   *   "recent" = most recent 8 readings  (~40 seconds of data)
   *   "older"  = previous 8 readings      (~40 seconds of data)
   *
   * When pills are placed/removed, they create a STEP CHANGE:
   *   - recent_avg shifts by the pill weight
   *   - older_avg stays at the old level (or partially shifted)
   *   - window_delta = |recent - older| reflects the step
   *
   * When there's just drift:
   *   - Both halves drift together
   *   - window_delta ≈ drift_rate × half_window_time ≈ 52 counts
   *   - This is below the threshold (80)
   *
   * 3 pills (~90 counts) + drift (~52) = window_delta ~142
   *   → clearly above threshold 80
   */

  recent_avg = weight_half_window_avg(sensor, true);
  older_avg = weight_half_window_avg(sensor, false);
  int32_t window_delta = recent_avg - older_avg;
  active_delta = weight_apply_present_sign(sensor, window_delta);

  if (active_delta >= (int32_t)thresh) {
    sensor->streak++;
    if (sensor->streak >= sensor->config.settle_reads_required) {
      bool new_state = (level_delta >= (int32_t)thresh);
      bool changed = (new_state != sensor->pill_present);

      sensor->pill_present = new_state;
      sensor->streak = 0;
      if (changed) {
        sensor->clear_streak = 0;
        weight_seed_history(sensor, raw_average);
        ESP_LOGI(TAG,
                 "State changed: pill=%s (delta=%" PRId32 ", level=%" PRId32
                 ", th=%" PRIu32 ")",
                 new_state ? "true" : "false", active_delta, level_delta,
                 thresh);
      }
    }
  } else {
    sensor->streak = 0;
  }

  status->pill_present = sensor->pill_present;
  status->tared = sensor->tared;
  status->raw_average = raw_average;
  status->recent_avg = recent_avg;
  status->older_avg = older_avg;
  status->window_delta = active_delta;
  status->history_count = sensor->history_count;

  return ESP_OK;
}
