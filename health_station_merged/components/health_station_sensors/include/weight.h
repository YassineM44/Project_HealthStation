#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WEIGHT_GAIN_128 = 1,
    WEIGHT_GAIN_64 = 3,
    WEIGHT_GAIN_32 = 2,
} weight_gain_t;

typedef struct {
    gpio_num_t dout_gpio;
    gpio_num_t sck_gpio;
    weight_gain_t gain;
    uint32_t data_ready_timeout_ms;
} weight_config_t;

typedef struct {
    weight_config_t cfg;
    int32_t offset;
    float scale;
} weight_handle_t;

void weight_init(weight_handle_t *handle, const weight_config_t *cfg);
bool weight_wait_ready(weight_handle_t *handle);
int32_t weight_read_raw(weight_handle_t *handle);
int32_t weight_read_average(weight_handle_t *handle, int samples);
void weight_set_offset(weight_handle_t *handle, int32_t offset);
void weight_set_scale(weight_handle_t *handle, float scale);
int32_t weight_get_offset(weight_handle_t *handle);
float weight_get_scale(weight_handle_t *handle);
void weight_tare(weight_handle_t *handle, int samples);
float weight_get_units(weight_handle_t *handle, int samples);

#ifdef __cplusplus
}
#endif
