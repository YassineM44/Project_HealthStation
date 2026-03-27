#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "weight.h"

#define WEIGHT_DOUT_GPIO        GPIO_NUM_4
#define WEIGHT_SCK_GPIO         GPIO_NUM_5
#define WEIGHT_PRESENT_SIGN     0
#define WEIGHT_BOOT_SETTLE_MS   5000U
#define WEIGHT_POLL_INTERVAL_MS 300U
static const char *TAG = "main";

void app_main(void)
{
    static weight_sensor_t weight_sensor;
    weight_status_t status = {0};
    weight_config_t config = {
        .dout_gpio = WEIGHT_DOUT_GPIO,
        .sck_gpio = WEIGHT_SCK_GPIO,
        .present_threshold_raw = WEIGHT_DEFAULT_PRESENT_THRESHOLD_RAW,
        .samples_per_update = WEIGHT_DEFAULT_SAMPLES_PER_UPDATE,
        .settle_reads_required = WEIGHT_DEFAULT_SETTLE_READS_REQUIRED,
        .present_sign = WEIGHT_PRESENT_SIGN,
    };
    esp_err_t err;

    err = weight_init(&weight_sensor, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "weight_init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG,
             "Waiting %" PRIu32 " ms before tare; keep the compartment empty",
             (uint32_t)WEIGHT_BOOT_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(WEIGHT_BOOT_SETTLE_MS));

    err = weight_tare(&weight_sensor, WEIGHT_DEFAULT_TARE_SAMPLE_COUNT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "weight_tare failed: %s", esp_err_to_name(err));
        return;
    }

    while (true) {
        err = weight_update(&weight_sensor, &status);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "weight_update failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "pill_present=%s",
                     status.pill_present ? "true" : "false");
        }

        vTaskDelay(pdMS_TO_TICKS(WEIGHT_POLL_INTERVAL_MS));
    }
}
