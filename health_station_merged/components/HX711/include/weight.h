#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#define WEIGHT_DEFAULT_TARE_SAMPLE_COUNT 64U
#define WEIGHT_DEFAULT_SAMPLES_PER_UPDATE 48U
#define WEIGHT_DEFAULT_SETTLE_READS_REQUIRED 2U
#define WEIGHT_DEFAULT_PRESENT_THRESHOLD_RAW 70U
#define WEIGHT_DEFAULT_PRESENT_SIGN 0
#define WEIGHT_HISTORY_SIZE 16U

typedef struct {
  gpio_num_t dout_gpio;
  gpio_num_t sck_gpio;
  uint32_t present_threshold_raw;
  uint8_t samples_per_update;
  uint8_t settle_reads_required;
  int8_t present_sign;
} weight_config_t;

typedef struct {
  bool tared;
  bool pill_present;
  int32_t raw_average;
  int32_t recent_avg;
  int32_t older_avg;
  int32_t window_delta;
  uint8_t history_count;
} weight_status_t;

typedef struct {
  weight_config_t config;
  bool initialized;
  bool tared;
  bool pill_present;
  int32_t tare_raw;
  int32_t history[WEIGHT_HISTORY_SIZE];
  uint8_t history_index;
  uint8_t history_count;
  uint8_t streak;
  uint8_t clear_streak;
} weight_sensor_t;

esp_err_t weight_init(weight_sensor_t *sensor, const weight_config_t *config);
esp_err_t weight_tare(weight_sensor_t *sensor, uint16_t sample_count);
esp_err_t weight_update(weight_sensor_t *sensor, weight_status_t *status);
