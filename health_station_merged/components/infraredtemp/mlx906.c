#include "mlx906.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdbool.h>

#ifndef MLX906_SDA_PIN
#define MLX906_SDA_PIN  21
#endif

#ifndef MLX906_SCL_PIN
#define MLX906_SCL_PIN  22
#endif

#ifndef MLX906_I2C_PORT
#define MLX906_I2C_PORT I2C_NUM_1
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

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MLX906_SDA_PIN,
        .scl_io_num = MLX906_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MLX906_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(MLX906_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(MLX906_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_inited = true;
    ESP_LOGI(TAG, "init ok (SDA=%d SCL=%d addr=0x%02X freq=%dHz port=%d)",
             MLX906_SDA_PIN, MLX906_SCL_PIN, MLX906_ADDR, MLX906_I2C_FREQ_HZ, MLX906_I2C_PORT);
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
    esp_err_t ret = i2c_master_write_read_device(
        MLX906_I2C_PORT,
        MLX906_ADDR,
        &reg,
        1,
        data,
        sizeof(data),
        pdMS_TO_TICKS(MLX906_TIMEOUT_MS)
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
