// MAX30102 heart-rate + SpO2 driver (basic demo-grade processing)
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifndef HO2_GPIO_UNUSED
#define HO2_GPIO_UNUSED ((gpio_num_t)(-1))
#endif

typedef enum {
    HO2_SR_50 = 0,
    HO2_SR_100 = 1,
    HO2_SR_200 = 2,
    HO2_SR_400 = 3,
    HO2_SR_800 = 4,
    HO2_SR_1000 = 5,
    HO2_SR_1600 = 6,
    HO2_SR_3200 = 7,
} ho2_sample_rate_t;

typedef enum {
    HO2_PW_69 = 0,   // 15-bit
    HO2_PW_118 = 1,  // 16-bit
    HO2_PW_215 = 2,  // 17-bit
    HO2_PW_411 = 3,  // 18-bit
} ho2_pulse_width_t;

typedef enum {
    HO2_ADC_2048 = 0,
    HO2_ADC_4096 = 1,
    HO2_ADC_8192 = 2,
    HO2_ADC_16384 = 3,
} ho2_adc_range_t;

typedef struct {
    i2c_port_t i2c_port;
    gpio_num_t sda_io;
    gpio_num_t scl_io;
    gpio_num_t int_io;  // set to HO2_GPIO_UNUSED if not connected
    uint32_t i2c_clk_hz;

    ho2_sample_rate_t sample_rate;
    ho2_pulse_width_t pulse_width;
    ho2_adc_range_t adc_range;
    uint8_t led_current_red;
    uint8_t led_current_ir;
    bool use_interrupt;
    bool swap_red_ir;  // set true for boards that expose FIFO channels swapped
} ho2_config_t;

typedef struct {
    uint32_t red;
    uint32_t ir;
} ho2_sample_t;

typedef struct {
    float heart_rate_bpm;
    float spo2_percent;
    float red_dc;
    float ir_dc;
    bool valid;
} ho2_result_t;

typedef struct {
    double avered;
    double aveir;
    double sumirrms;
    double sumredrms;
    int sample_count;
    int num_for_spo2;
    float estimated_spo2;
    double frate;
    double f_spo2;
    uint32_t finger_on_threshold;
    uint32_t boot_time_ms;
} ho2_algo_t;

esp_err_t ho2_init(const ho2_config_t *cfg);
esp_err_t ho2_wait_for_data(TickType_t timeout);
esp_err_t ho2_read_fifo(ho2_sample_t *samples, size_t max_samples, size_t *samples_read);
esp_err_t ho2_compute(const ho2_sample_t *samples, size_t n, float sample_rate_hz, ho2_result_t *out);

uint16_t ho2_sample_rate_hz(ho2_sample_rate_t rate);

void ho2_algo_init(ho2_algo_t *state, uint32_t now_ms);
bool ho2_algo_update(ho2_algo_t *state, uint32_t red, uint32_t ir, uint32_t now_ms,
                     float *spo2_out, bool *finger_on);
