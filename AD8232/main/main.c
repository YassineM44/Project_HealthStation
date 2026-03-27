/**
 * @file main.c
 * @brief AD8232 ECG Monitor - Main Application (Diagnostic Mode)
 *
 * Outputs BOTH raw calibrated mV and filtered mV for debugging.
 * Also outputs lead-off status so we can verify electrode contact.
 *
 * Serial Studio: COM4, 115200, Quick Plot
 * Frame: slash-star ... star-slash
 * Fields: timestamp_ms, raw_mv, filtered_mv, leads_off
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ad8232.h"

static const char *TAG = "ECG_MAIN";

static ad8232_handle_t s_ecg;
static SemaphoreHandle_t s_sample_sem = NULL;

static void ecg_timer_callback(void *arg)
{
    (void)arg;
    xSemaphoreGive(s_sample_sem);
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  AD8232 ECG Monitor - Diagnostic Mode  ");
    ESP_LOGI(TAG, "  500 Hz / 12-bit / 4-field CSV output   ");
    ESP_LOGI(TAG, "========================================");

    s_sample_sem = xSemaphoreCreateBinary();
    if (s_sample_sem == NULL) {
        ESP_LOGE(TAG, "Semaphore create failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    esp_err_t ret = ad8232_init(&s_ecg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AD8232 init failed: 0x%x", ret);
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    const esp_timer_create_args_t timer_args = {
        .callback        = ecg_timer_callback,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "ecg_timer",
    };

    esp_timer_handle_t timer_handle = NULL;
    ret = esp_timer_create(&timer_args, &timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Timer create failed: 0x%x", ret);
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    ret = esp_timer_start_periodic(timer_handle, AD8232_TIMER_PERIOD_US);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Timer start failed: 0x%x", ret);
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    ESP_LOGI(TAG, "Output: timestamp_ms, raw_mv, filtered_mv, leads_off");

    vTaskDelay(pdMS_TO_TICKS(500));

    ad8232_sample_t sample;

    while (1) {
        if (xSemaphoreTake(s_sample_sem, pdMS_TO_TICKS(100)) == pdTRUE) {

            ret = ad8232_read_sample(&s_ecg, &sample);
            if (ret != ESP_OK) {
                continue;
            }

            /* DECIMATION FOR ARDUINO SERIAL PLOTTER 
             * We calculate filters at 500 Hz for perfect 50 Hz noise removal,
             * but we only PRINT at 50 Hz (every 10th sample).
             * This slows down the Arduino Plotter so it takes 10 seconds 
             * to sweep across the screen!
             */
            static uint8_t decimate_counter = 0;
            if (++decimate_counter >= 10) {
                /* We ONLY print the filtered signal now so the plotter 
                 * perfectly auto-scales to the QRS complexes!
                 */
                printf("%.2f\n", sample.filtered_mv);
                decimate_counter = 0;
            }

            /* Yield briefly to feed watchdog if FreeRTOS 10ms tick misses us */
            taskYIELD();
        }
    }
}
