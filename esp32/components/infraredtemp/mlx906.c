#include "mlx906.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdbool.h>

#ifndef MLX906_SDA_PIN
#define MLX906_SDA_PIN  21
#endif

#ifndef MLX906_SCL_PIN
#define MLX906_SCL_PIN  22
#endif

#ifndef MLX906_I2C_PORT
#define MLX906_I2C_PORT I2C_NUM_0
#endif

#ifndef MLX906_ADDR
#define MLX906_ADDR     0x5A
#endif

#ifndef MLX906_I2C_FREQ_HZ
#define MLX906_I2C_FREQ_HZ 100000
#endif

#define MLX906_TOBJ1    0x07
#define MLX906_TA       0x06   // ambient temperature register
#define MLX906_TIMEOUT_MS 1000

static const char *TAG = "mlx906";

static i2c_master_bus_handle_t s_bus_handle;
static i2c_master_dev_handle_t s_dev_handle;
static bool s_inited;
static esp_err_t s_last_err;

esp_err_t mlx906_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    if (!GPIO_IS_VALID_GPIO(MLX906_SDA_PIN) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(MLX906_SDA_PIN) ||
        !GPIO_IS_VALID_GPIO(MLX906_SCL_PIN) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(MLX906_SCL_PIN)) {
        ESP_LOGE(TAG, "invalid SDA/SCL pins (SDA=%d SCL=%d)", MLX906_SDA_PIN, MLX906_SCL_PIN);
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = MLX906_I2C_PORT,
        .sda_io_num = MLX906_SDA_PIN,
        .scl_io_num = MLX906_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MLX906_ADDR,
        .scl_speed_hz = MLX906_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle);
    if (ret != ESP_OK) {
        i2c_del_master_bus(s_bus_handle);
        s_bus_handle = NULL;
        return ret;
    }

    s_inited = true;
    ESP_LOGI(TAG, "init ok (SDA=%d SCL=%d addr=0x%02X freq=%dHz)",
             MLX906_SDA_PIN, MLX906_SCL_PIN, MLX906_ADDR, MLX906_I2C_FREQ_HZ);
    return ESP_OK;
}

static esp_err_t mlx906_read_reg(uint8_t reg, float *temp)
{
    if (temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[3] = {0};
    esp_err_t ret = i2c_master_transmit_receive(
        s_dev_handle,
        &reg,
        1,
        data,
        sizeof(data),
        MLX906_TIMEOUT_MS
    );
    if (ret != ESP_OK) {
        if (ret != s_last_err) {
            ESP_LOGE(TAG, "read reg 0x%02X failed: %s (%d)",
                     reg, esp_err_to_name(ret), ret);
            s_last_err = ret;
        }
        return ret;
    }

    s_last_err = ESP_OK;
    uint16_t raw = ((uint16_t)data[1] << 8) | data[0];
    *temp = (raw * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t mlx906_read_object_temp(float *temp)
{
    return mlx906_read_reg(MLX906_TOBJ1, temp);
}

esp_err_t mlx906_read_ambient_temp(float *temp)
{
    return mlx906_read_reg(MLX906_TA, temp);
}