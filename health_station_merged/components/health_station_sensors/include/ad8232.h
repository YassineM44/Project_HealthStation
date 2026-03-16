#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t output_gpio;
    gpio_num_t lo_plus_gpio;
    gpio_num_t lo_minus_gpio;
} ad8232_config_t;

typedef struct {
    uint64_t timestamp_ms;
    uint16_t raw;
    int32_t signal_mv;
    int32_t ecg_uv;
    bool valid;
    bool lo_plus;
    bool lo_minus;
    bool clipped;
} ad8232_sample_t;

#define AD8232_DEFAULT_CONFIG()      \
    {                                \
        .output_gpio = GPIO_NUM_34,  \
        .lo_plus_gpio = GPIO_NUM_32, \
        .lo_minus_gpio = GPIO_NUM_33, \
    }

esp_err_t ad8232_init(const ad8232_config_t *config);
esp_err_t ad8232_start(void);
esp_err_t ad8232_read_sample(ad8232_sample_t *out_sample, TickType_t timeout_ticks);
esp_err_t ad8232_stop(void);

#ifdef __cplusplus
}
#endif
