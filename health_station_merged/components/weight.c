#include "weight.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define HX711_PULSE_US 1

static void hx711_pulse_sck(weight_handle_t *handle)
{
    gpio_set_level(handle->cfg.sck_gpio, 1);
    esp_rom_delay_us(HX711_PULSE_US);
    gpio_set_level(handle->cfg.sck_gpio, 0);
    esp_rom_delay_us(HX711_PULSE_US);
}

void weight_init(weight_handle_t *handle, const weight_config_t *cfg)
{
    handle->cfg = *cfg;
    handle->offset = 0;
    handle->scale = 1.0f;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << handle->cfg.sck_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << handle->cfg.dout_gpio);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_set_level(handle->cfg.sck_gpio, 0);
}

bool weight_wait_ready(weight_handle_t *handle)
{
    uint64_t start = esp_timer_get_time();
    while (gpio_get_level(handle->cfg.dout_gpio) != 0) {
        if (handle->cfg.data_ready_timeout_ms == 0) {
            continue;
        }
        uint64_t elapsed_us = esp_timer_get_time() - start;
        if (elapsed_us > (uint64_t)handle->cfg.data_ready_timeout_ms * 1000ULL) {
            return false;
        }
    }
    return true;
}

int32_t weight_read_raw(weight_handle_t *handle)
{
    if (!weight_wait_ready(handle)) {
        return 0;
    }

    int32_t value = 0;
    for (int i = 0; i < 24; i++) {
        hx711_pulse_sck(handle);
        value = (value << 1) | gpio_get_level(handle->cfg.dout_gpio);
    }

    for (int i = 0; i < handle->cfg.gain; i++) {
        hx711_pulse_sck(handle);
    }

    if (value & 0x800000) {
        value |= ~0xFFFFFF;
    }

    return value;
}

int32_t weight_read_average(weight_handle_t *handle, int samples)
{
    if (samples <= 0) {
        samples = 1;
    }

    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += weight_read_raw(handle);
    }

    return (int32_t)(sum / samples);
}

void weight_set_offset(weight_handle_t *handle, int32_t offset)
{
    handle->offset = offset;
}

void weight_set_scale(weight_handle_t *handle, float scale)
{
    if (scale == 0.0f) {
        scale = 1.0f;
    }
    handle->scale = scale;
}

int32_t weight_get_offset(weight_handle_t *handle)
{
    return handle->offset;
}

float weight_get_scale(weight_handle_t *handle)
{
    return handle->scale;
}

void weight_tare(weight_handle_t *handle, int samples)
{
    int32_t avg = weight_read_average(handle, samples);
    weight_set_offset(handle, avg);
}

float weight_get_units(weight_handle_t *handle, int samples)
{
    int32_t raw = weight_read_average(handle, samples);
    return (float)(raw - handle->offset) / handle->scale;
}
