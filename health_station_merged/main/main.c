/**
 * @file main.c
 * @brief Health Station — Merged Sensor Coordinator
 *
 * Runs all four sensors simultaneously via dedicated FreeRTOS tasks:
 *   1. AD8232  ECG        — 500 Hz timer-driven ADC sampling
 *   2. HX711  Weight      — 300 ms polled pill detection
 *   3. MAX30102 HR/SpO2   — interrupt-driven FIFO at 200 Hz
 *   4. MLX90614 Temp      — 1 s polled IR temperature
 *
 * UART1 (GPIO 25 TX, GPIO 16 RX) streams JSON to Raspberry Pi.
 */

#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* ── Sensor headers ── */
#include "ad8232.h"
#include "max30102.h"
#include "mlx906.h"
#include "weight.h"
#include "ir.h"

static const char *TAG = "HEALTH";

/* ═══════════════════════════════════════════════════════════════════════════
 *  UART1 — JSON data link to Raspberry Pi
 * ═══════════════════════════════════════════════════════════════════════════
 */

#define RASPI_UART_NUM     UART_NUM_1
#define RASPI_UART_TX_PIN  GPIO_NUM_25
#define RASPI_UART_RX_PIN  GPIO_NUM_16
#define RASPI_UART_BAUD    115200
#define RASPI_UART_BUF_SZ  1024

static SemaphoreHandle_t s_uart_mutex = NULL;

/* ── Global sensor ready states for UI ── */
static bool s_ready_hb    = false;
static bool s_ready_temp  = false;
static bool s_ready_pilld = false;
static bool s_ready_pilln = false;
static bool s_ready_ecg   = false;

/**
 * @brief Send a JSON line over UART1 to the Raspberry Pi (thread-safe).
 *        Automatically appends '\n'.
 */
static void uart_send_json(const char *json_line) {
  if (s_uart_mutex == NULL) return;
  if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    uart_write_bytes(RASPI_UART_NUM, json_line, strlen(json_line));
    uart_write_bytes(RASPI_UART_NUM, "\n", 1);
    xSemaphoreGive(s_uart_mutex);
  }
}

static void raspi_uart_init(void) {
  s_uart_mutex = xSemaphoreCreateMutex();
  assert(s_uart_mutex != NULL);

  const uart_config_t uart_cfg = {
    .baud_rate  = RASPI_UART_BAUD,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_param_config(RASPI_UART_NUM, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(RASPI_UART_NUM,
                               RASPI_UART_TX_PIN,
                               RASPI_UART_RX_PIN,
                               UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(RASPI_UART_NUM,
                                      RASPI_UART_BUF_SZ,
                                      RASPI_UART_BUF_SZ,
                                      0, NULL, 0));
  ESP_LOGI(TAG, "UART1 → Raspberry Pi initialised (TX=%d, RX=%d, %d baud)",
           RASPI_UART_TX_PIN, RASPI_UART_RX_PIN, RASPI_UART_BAUD);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  1. AD8232 ECG Task — 500 Hz timer-driven
 *
 *  Set ECG_SERIAL_PLOT to 1 to enable bare printf output for Serial Studio
 *  Pro waveform plotting.  Set to 0 (default) to keep the monitor clean
 *  while still sampling/filtering in the background.
 * ═══════════════════════════════════════════════════════════════════════════
 */

#define ECG_SERIAL_PLOT 0 /* 1 = print for Serial Studio, 0 = silent */

static ad8232_handle_t s_ecg;
static SemaphoreHandle_t s_ecg_sem = NULL;

static void ecg_timer_cb(void *arg) {
  (void)arg;
  xSemaphoreGive(s_ecg_sem);
}

static void ecg_task(void *arg) {
  (void)arg;
  ad8232_sample_t sample;
  uint8_t decimate = 0;
  float   acc      = 0.0f;   /* accumulator for averaging decimation */

  /* ── UART ECG batch: always decimates 500→50 Hz, batches 10 samples ── */
  uint8_t uart_dec = 0;
  float   uart_acc = 0.0f;
  float   uart_batch[10];
  uint8_t uart_batch_idx = 0;

  /* Lead-off gating: track state to send ecg_on / ecg_end once */
  bool ecg_streaming = false;

  while (1) {
    if (xSemaphoreTake(s_ecg_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (ad8232_read_sample(&s_ecg, &sample) != ESP_OK) {
        continue;
      }

      /* ── Lead-off gating ── */
      if (sample.leads_off) {
        if (ecg_streaming) {
          ecg_streaming = false;
          uart_send_json("{\"t\":\"ecg_end\"}");
          ESP_LOGW("ECG", "Leads OFF — stopped streaming");
          /* Reset batch state */
          uart_dec = 0;
          uart_acc = 0.0f;
          uart_batch_idx = 0;
        }
        taskYIELD();
        continue;  /* Skip all processing when leads are off */
      }

      /* Leads are ON */
      if (!ecg_streaming) {
        ecg_streaming = true;
        uart_send_json("{\"t\":\"ecg_on\"}");
        ESP_LOGI("ECG", "Leads ON — streaming waveform");
      }

#if ECG_SERIAL_PLOT
      /* Averaging decimation 500 Hz → 50 Hz for Serial Studio Pro. */
      acc += sample.filtered_mv;
      if (++decimate >= 10) {
        printf("%.2f\n", acc / 10.0f);
        acc = 0.0f;
        decimate = 0;
      }
#else
      (void)decimate;
      (void)acc;
#endif

      /* ── UART: decimate 500→50 Hz then batch 10 samples (5 packets/s) ── */
      uart_acc += sample.filtered_mv;
      if (++uart_dec >= 10) {
        uart_batch[uart_batch_idx++] = uart_acc / 10.0f;
        uart_acc = 0.0f;
        uart_dec = 0;

        if (uart_batch_idx >= 10) {
          char buf[256];
          int pos = snprintf(buf, sizeof(buf), "{\"t\":\"ecg\",\"v\":[");
          for (int j = 0; j < 10; j++) {
            if ((size_t)pos >= sizeof(buf) - 10) break;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                            "%s%.2f", j ? "," : "", uart_batch[j]);
          }
          if ((size_t)pos < sizeof(buf) - 2) {
             snprintf(buf + pos, sizeof(buf) - pos, "]}");
          }
          uart_send_json(buf);
          uart_batch_idx = 0;
        }
      }

      taskYIELD();
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  2. HX711 Weight / Pill Detection Task — 300 ms poll
 * ═══════════════════════════════════════════════════════════════════════════
 */

#define WEIGHT_DOUT_GPIO GPIO_NUM_4
#define WEIGHT_SCK_GPIO GPIO_NUM_5
#define WEIGHT_PRESENT_SIGN 0
#define WEIGHT_BOOT_SETTLE_MS 5000U
#define WEIGHT_POLL_INTERVAL_MS 300U

static void weight_task(void *arg) {
  (void)arg;
  static const char *T = "WEIGHT";
  static weight_sensor_t sensor;
  weight_status_t status = {0};

  weight_config_t config = {
      .dout_gpio = WEIGHT_DOUT_GPIO,
      .sck_gpio = WEIGHT_SCK_GPIO,
      .present_threshold_raw = 150,  /* drift maxes ~124, real pills give ~229+ */
      .samples_per_update = WEIGHT_DEFAULT_SAMPLES_PER_UPDATE,
      .settle_reads_required = 4,  /* require 4 consecutive reads (~1.2s) to confirm */
      .present_sign = WEIGHT_PRESENT_SIGN,
  };

  esp_err_t err = weight_init(&sensor, &config);
  if (err != ESP_OK) {
    ESP_LOGE(T, "weight_init failed: %s", esp_err_to_name(err));
    vTaskDelete(NULL);
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(WEIGHT_BOOT_SETTLE_MS));

  err = weight_tare(&sensor, WEIGHT_DEFAULT_TARE_SAMPLE_COUNT);
  if (err != ESP_OK) {
    ESP_LOGE(T, "weight_tare failed: %s", esp_err_to_name(err));
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(T, "Tare complete (initial)");
  s_ready_pilld = true;

  /* Event-driven: only print when pill state changes */
  bool prev_pill_present = false;
  uint16_t idle_retare_count = 0;
  uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

  while (1) {
    err = weight_update(&sensor, &status);
    
    /* ── DIAGNOSTIC LOGGING (First 30 seconds) ── */
    uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000ULL) - start_ms;
    if (elapsed < 30000) {
        ESP_LOGI(T, "[DEBUG] Level Delta: %" PRId32 " (Thresh: 150)", status.window_delta);
    }

    if (err != ESP_OK) {
      ESP_LOGW(T, "weight_update failed: %s", esp_err_to_name(err));
    } else if (status.pill_present != prev_pill_present) {
      if (status.pill_present) {
        ESP_LOGI(T, "Day Pill PLACED");
        uart_send_json("{\"t\":\"pill_d\",\"p\":true}");
      } else {
        ESP_LOGW(T, "Day Pill REMOVED");
        uart_send_json("{\"t\":\"pill_d\",\"p\":false}");
      }
      prev_pill_present = status.pill_present;
      idle_retare_count = 0;
    }

    if (!prev_pill_present) {
      idle_retare_count++;
      if (idle_retare_count >= 10) {
        esp_err_t te = weight_tare(&sensor, 8);
        if (te != ESP_OK) {
          ESP_LOGW(T, "Retare failed: %s", esp_err_to_name(te));
        }
        idle_retare_count = 0;
      }
    } else {
      idle_retare_count = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(WEIGHT_POLL_INTERVAL_MS));
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  3. MAX30102 Heart Rate / SpO2 Task — interrupt-driven FIFO
 * ═══════════════════════════════════════════════════════════════════════════
 */

#define HO2_SAMPLE_RATE_HZ 200
#define HO2_BPM_WINDOW_MS 4000
#define HO2_BPM_WINDOW_SAMPLES ((HO2_SAMPLE_RATE_HZ * HO2_BPM_WINDOW_MS) / 1000)
#define HO2_COMPUTE_PERIOD_MS 500
#define HO2_COMPUTE_STEP_SAMPLES                                               \
  ((HO2_SAMPLE_RATE_HZ * HO2_COMPUTE_PERIOD_MS) / 1000)
#define HO2_PRINT_PERIOD_MS 1000
#define HO2_STABILIZE_MS 3000
#define HO2_FINGER_ON_THRESHOLD 30000U
#define HO2_MIN_VALID_SPO2 70.0f

static ho2_sample_t s_ho2_ring[HO2_BPM_WINDOW_SAMPLES];
static ho2_sample_t s_ho2_window[HO2_BPM_WINDOW_SAMPLES];

typedef enum {
  HO2_WAITING_FOR_FINGER = 0,
  HO2_STABILIZING,
  HO2_STREAMING,
} ho2_app_state_t;

static void ho2_reset_capture(size_t *ri, size_t *rc, size_t *sl,
                              ho2_result_t *lr, bool *lrv, uint32_t *lrm,
                              uint32_t *lpm, uint32_t *lprm) {
  *ri = 0;
  *rc = 0;
  *sl = 0;
  *lr = (ho2_result_t){0};
  *lrv = false;
  *lrm = 0;
  *lpm = 0;
  *lprm = 0;
}

static bool ho2_try_compute(size_t ri, size_t rc, size_t *sl,
                            ho2_result_t *out) {
  if (rc < HO2_BPM_WINDOW_SAMPLES || *sl < HO2_COMPUTE_STEP_SAMPLES) {
    return false;
  }
  size_t start = ri;
  for (size_t i = 0; i < HO2_BPM_WINDOW_SAMPLES; ++i) {
    s_ho2_window[i] = s_ho2_ring[(start + i) % HO2_BPM_WINDOW_SAMPLES];
  }
  *sl = 0;
  *out = (ho2_result_t){0};
  if (ho2_compute(s_ho2_window, HO2_BPM_WINDOW_SAMPLES, HO2_SAMPLE_RATE_HZ,
                  out) != ESP_OK) {
    return false;
  }
  /* Accept if SpO2 is valid (BPM may still be NaN — printed separately) */
  return !isnan(out->spo2_percent);
}

static void max30102_task(void *arg) {
  (void)arg;
  static const char *T = "MAX30102";

  ho2_config_t cfg = {
      .i2c_port = I2C_NUM_0,
      .sda_io = GPIO_NUM_18,
      .scl_io = GPIO_NUM_19,
      .int_io = GPIO_NUM_27,
      .i2c_clk_hz = 400000,
      .sample_rate = HO2_SR_200,
      .pulse_width = HO2_PW_411,
      .adc_range = HO2_ADC_16384,
      .led_current_red = 0x7F,
      .led_current_ir = 0x7F,
      .use_interrupt = true,
      .swap_red_ir = true,
  };

  esp_err_t err = ho2_init(&cfg);
  if (err != ESP_OK) {
    ESP_LOGE(T, "ho2_init failed: %s", esp_err_to_name(err));
    vTaskDelete(NULL);
    return;
  }
  s_ready_hb = true;

  /* Silent at boot — output only when finger is detected */

  ho2_app_state_t state = HO2_WAITING_FOR_FINGER;
  size_t ring_index = 0, ring_count = 0, since_last = 0;
  uint32_t finger_on_since = 0;
  int last_countdown = 0;
  ho2_result_t latest_result = {0};
  bool latest_result_valid = false;
  uint32_t latest_result_ms = 0, last_print_ms = 0, last_printed_result_ms = 0;

  /* BPM smoother: outlier-rejecting EMA filter */
  float smoothed_bpm = 0.0f;
  bool  bpm_initialized = false;
  uint32_t streaming_start_ms = 0;

  /* Grace period: suppress BPM display for this many ms after streaming
   * starts.  The EMA still runs during this time — it converges to the
   * real heart rate so that the FIRST displayed BPM value is accurate. */
  #define HO2_BPM_GRACE_MS 4000U

  while (1) {
    (void)ho2_wait_for_data(pdMS_TO_TICKS(200));

    size_t nread = 0;
    ho2_sample_t fifo_buf[32];
    err = ho2_read_fifo(fifo_buf, 32, &nread);
    if (err != ESP_OK) {
      ESP_LOGE(T, "FIFO read failed: %s", esp_err_to_name(err));
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (nread == 0) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    for (size_t i = 0; i < nread; ++i) {
      uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
      bool finger_on = fifo_buf[i].ir >= HO2_FINGER_ON_THRESHOLD;

      if (!finger_on) {
        if (state != HO2_WAITING_FOR_FINGER) {
          ESP_LOGW(T, "FINGER REMOVED, PLACE FINGER.");
          uart_send_json("{\"t\":\"hr_end\"}");
        }
        state = HO2_WAITING_FOR_FINGER;
        finger_on_since = 0;
        last_countdown = 0;
        ho2_reset_capture(&ring_index, &ring_count, &since_last, &latest_result,
                          &latest_result_valid, &latest_result_ms,
                          &last_print_ms, &last_printed_result_ms);
        /* Reset BPM smoother for next measurement */
        smoothed_bpm = 0.0f;
        bpm_initialized = false;
        streaming_start_ms = 0;
        continue;
      }

      if (state == HO2_WAITING_FOR_FINGER) {
        ho2_reset_capture(&ring_index, &ring_count, &since_last, &latest_result,
                          &latest_result_valid, &latest_result_ms,
                          &last_print_ms, &last_printed_result_ms);
        state = HO2_STABILIZING;
        finger_on_since = now_ms;
        last_countdown = 4;
        ESP_LOGI(T, "STABILIZING... %d", last_countdown);
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"t\":\"hr_stab\",\"wait\":%d}", last_countdown);
        uart_send_json(buf);
      }

      s_ho2_ring[ring_index] = fifo_buf[i];
      ring_index = (ring_index + 1) % HO2_BPM_WINDOW_SAMPLES;
      if (ring_count < HO2_BPM_WINDOW_SAMPLES)
        ring_count++;
      since_last++;

      ho2_result_t result = {0};
      if (ho2_try_compute(ring_index, ring_count, &since_last, &result)) {
        if (!isnan(result.spo2_percent) &&
            result.spo2_percent >= HO2_MIN_VALID_SPO2 &&
            result.spo2_percent <= 100.0f) {

          /* ── Smooth the BPM with outlier-rejecting EMA ── */
          if (!isnan(result.heart_rate_bpm)) {
            if (!bpm_initialized || smoothed_bpm == 0.0f) {
              smoothed_bpm = result.heart_rate_bpm;
              bpm_initialized = true;
            } else {
              float delta_pct = fabsf(result.heart_rate_bpm - smoothed_bpm)
                              / smoothed_bpm;
              if (delta_pct > 0.20f) {
                /* Outlier (>20% off): near-ignore */
                smoothed_bpm = smoothed_bpm * 0.97f
                             + result.heart_rate_bpm * 0.03f;
              } else {
                /* Normal reading: heavy EMA tracking */
                smoothed_bpm = smoothed_bpm * 0.8f
                             + result.heart_rate_bpm * 0.2f;
              }
            }
            /* Clamp BPM to physiological resting range */
            if (smoothed_bpm < 45.0f) smoothed_bpm = 45.0f;
            if (smoothed_bpm > 120.0f) smoothed_bpm = 120.0f;
            result.heart_rate_bpm = smoothed_bpm;
          }

          latest_result = result;
          latest_result_valid = true;
          latest_result_ms = now_ms;
        }
      }

      /* Check if BPM is still in grace period (converging) */
      bool bpm_ready = (streaming_start_ms > 0) &&
                       (now_ms - streaming_start_ms >= HO2_BPM_GRACE_MS);

      if (state == HO2_STABILIZING) {
        uint32_t elapsed_ms = now_ms - finger_on_since;
        if (elapsed_ms >= HO2_STABILIZE_MS) {
          state = HO2_STREAMING;
          streaming_start_ms = now_ms;
          /* Keep ring buffer — SpO2 data is already usable */
          since_last = 0;
          if (latest_result_valid &&
              (now_ms - latest_result_ms) <= HO2_PRINT_PERIOD_MS) {
            /* Send SpO2 immediately so display updates fast */
            ESP_LOGI(T, "SpO2 %.1f %%", latest_result.spo2_percent);
            char buf[64];
            snprintf(buf, sizeof(buf),
                     "{\"t\":\"hr\",\"spo2\":%.1f}",
                     latest_result.spo2_percent);
            uart_send_json(buf);
            last_print_ms = now_ms;
            last_printed_result_ms = latest_result_ms;
          }
          continue;
        }
        int countdown = (int)((HO2_STABILIZE_MS - elapsed_ms + 999U) / 1000U);
        if (countdown < 1)
          countdown = 1;
        if (countdown < last_countdown) {
          last_countdown = countdown;
          ESP_LOGI(T, "STABILIZING... %d", last_countdown);
          char buf[64];
          snprintf(buf, sizeof(buf), "{\"t\":\"hr_stab\",\"wait\":%d}", last_countdown);
          uart_send_json(buf);
        }
      }

      if (state == HO2_STREAMING && latest_result_valid &&
          latest_result_ms != last_printed_result_ms &&
          (last_print_ms == 0 ||
           (now_ms - last_print_ms) >= HO2_PRINT_PERIOD_MS)) {
        if (bpm_ready && !isnan(latest_result.heart_rate_bpm)) {
          ESP_LOGI(T, "SpO2 %.1f %% | HR %.1f bpm", latest_result.spo2_percent,
                   latest_result.heart_rate_bpm);
          char buf[96];
          snprintf(buf, sizeof(buf),
                   "{\"t\":\"hr\",\"spo2\":%.1f,\"bpm\":%.1f}",
                   latest_result.spo2_percent,
                   latest_result.heart_rate_bpm);
          uart_send_json(buf);
        } else {
          ESP_LOGI(T, "SpO2 %.1f %%", latest_result.spo2_percent);
          char buf[64];
          snprintf(buf, sizeof(buf),
                   "{\"t\":\"hr\",\"spo2\":%.1f}",
                   latest_result.spo2_percent);
          uart_send_json(buf);
        }
        last_print_ms = now_ms;
        last_printed_result_ms = latest_result_ms;
      }
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  4. MLX90614 Temperature Task — 1 s poll (event-driven output)
 *
 *  Uses an adaptive baseline to handle thermal drift from ESP32 heat.
 *  During warm-up (first 10 readings) the baseline is established.
 *  After warm-up, baseline tracks drift via EMA (α=0.05, τ≈20s).
 *  Finger detected = object temp rises >1.5°C above baseline.
 *  Hysteresis: stays "on" until temp drops within 0.7°C of baseline.
 *
 *  IMPORTANT: EMA is updated ONLY when not detecting.  This prevents
 *  the finger's heat from being absorbed into the baseline.
 * ═══════════════════════════════════════════════════════════════════════════
 */

#define MLX_WARMUP_READINGS   15     /* Increased for better baseline */
#define MLX_DELTA_ON_C        1.2f   /* Robust trigger threshold */
#define MLX_DELTA_OFF_C       0.6f   /* Robust hysteresis threshold */
#define MLX_BASELINE_ALPHA    0.05f  /* EMA smoothing factor (slower = less drift noise) */
#define MLX_WRIST_OFFSET_C    4.0f   /* Wrist→body temp compensation */

static void mlx_task(void *arg) {
  (void)arg;
  static const char *T = "MLX906";

  if (mlx906_init() != ESP_OK) {
    ESP_LOGE(T, "mlx906_init failed");
    vTaskDelete(NULL);
    return;
  }
  s_ready_temp = true;

  bool mlx_finger_on = false;
  float mlx_baseline = 0.0f;
  int mlx_warmup = 0;

  while (1) {
    float obj_temp, amb_temp;
    if (mlx906_read_object_temp(&obj_temp) == ESP_OK &&
        mlx906_read_ambient_temp(&amb_temp) == ESP_OK) {

      /* ── Warm-up phase: establish baseline ── */
      if (mlx_warmup < MLX_WARMUP_READINGS) {
        mlx_baseline = obj_temp; /* overwrite until warm-up completes */
        mlx_warmup++;
        if (mlx_warmup == MLX_WARMUP_READINGS) {
          ESP_LOGI(T, "Baseline set: obj=%.2f C  amb=%.2f C", obj_temp, amb_temp);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      /* ── Detection logic ── */
      if (!mlx_finger_on) {
        /* Check FIRST against current baseline (before EMA update) */
        if (obj_temp > mlx_baseline + MLX_DELTA_ON_C) {
          mlx_finger_on = true;
          ESP_LOGI(T, "Object detected");
          ESP_LOGI(T, "Object: %.2f C | Ambient: %.2f C", obj_temp, amb_temp);
          float display_temp = obj_temp + MLX_WRIST_OFFSET_C;
          char buf[80];
          snprintf(buf, sizeof(buf),
                   "{\"t\":\"temp\",\"obj\":%.2f,\"amb\":%.2f}",
                   display_temp, amb_temp);
          uart_send_json(buf);
        } else {
          /* Only update baseline when NOT detecting — prevents finger
           * heat from being absorbed into the baseline. */
          mlx_baseline = mlx_baseline * (1.0f - MLX_BASELINE_ALPHA)
                       + obj_temp * MLX_BASELINE_ALPHA;
        }
      } else {
        /* Finger on — keep printing until temp drops near baseline */
        if (obj_temp < mlx_baseline + MLX_DELTA_OFF_C) {
          mlx_finger_on = false;
          mlx_baseline = obj_temp; /* reset baseline to current reading */
          ESP_LOGW(T, "Object removed");
          uart_send_json("{\"t\":\"temp_end\"}");
        } else {
          ESP_LOGI(T, "Object: %.2f C | Ambient: %.2f C", obj_temp, amb_temp);
          float display_temp = obj_temp + MLX_WRIST_OFFSET_C;
          char buf[80];
          snprintf(buf, sizeof(buf),
                   "{\"t\":\"temp\",\"obj\":%.2f,\"amb\":%.2f}",
                   display_temp, amb_temp);
          uart_send_json(buf);
        }
      }
    } else {
      ESP_LOGW(T, "Failed to read sensor");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  5. IR Obstacle Sensor — Night Pill Detection
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define IR_OUT_GPIO             GPIO_NUM_17

static void ir_task(void *arg) {
    (void)arg;
    static const char *T = "IR";
    ir_sensor_t sensor;
    ir_status_t status = {0};

    ir_config_t config = {
        .out_gpio        = IR_OUT_GPIO,
        .active_low      = true,
        .debounce_reads  = 3,
        .poll_interval_ms = 200,
    };

    if (ir_init(&sensor, &config) != ESP_OK) {
        ESP_LOGE(T, "ir_init failed");
        vTaskDelete(NULL); return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    if (ir_calibrate(&sensor, 10, 200) != ESP_OK) {
        ESP_LOGE(T, "Calibration failed - check potentiometer");
        vTaskDelete(NULL); return;
    }
    s_ready_pilln = true;

    bool prev_pill_present = false;
    while (1) {
        if (ir_update(&sensor, &status) == ESP_OK) {
            if (status.pill_present != prev_pill_present) {
                prev_pill_present = status.pill_present;
                if (status.pill_present) {
                    ESP_LOGI(T, "Night Pill PLACED");
                    uart_send_json("{\"t\":\"pill_n\",\"p\":true}");
                } else {
                    ESP_LOGW(T, "Night Pill REMOVED");
                    uart_send_json("{\"t\":\"pill_n\",\"p\":false}");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  6. System Status Heartbeat — Reports 'Ready' icons to Pi
 * ═══════════════════════════════════════════════════════════════════════════
 */
static void status_task(void *arg) {
    (void)arg;
    while(1) {
        char buf[128];
        snprintf(buf, sizeof(buf), 
                 "{\"t\":\"status\",\"hb\":%d,\"temp\":%d,\"pd\":%d,\"pn\":%d,\"ecg\":%d}",
                 s_ready_hb, s_ready_temp, s_ready_pilld, s_ready_pilln, s_ready_ecg);
        uart_send_json(buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  app_main — Initialize ECG + create all 4 sensor tasks
 * ═══════════════════════════════════════════════════════════════════════════
 */

void app_main(void) {
  ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
  ESP_LOGI(TAG, "║   Health Station — All Sensors       ║");
  ESP_LOGI(TAG, "║   AD8232 | HX711 | MAX30102 | MLX   ║");
  ESP_LOGI(TAG, "║   + UART1 → Raspberry Pi             ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

  /* ── 0. UART1 to Raspberry Pi ── */
  raspi_uart_init();

  /* ── 1. AD8232 ECG init ── */
  s_ecg_sem = xSemaphoreCreateBinary();
  if (s_ecg_sem == NULL) {
    ESP_LOGE(TAG, "ECG semaphore create failed");
    return;
  }

  esp_err_t ret = ad8232_init(&s_ecg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "AD8232 init failed: 0x%x", ret);
    return;
  }
  s_ready_ecg = true;

  const esp_timer_create_args_t ecg_timer_args = {
      .callback = ecg_timer_cb,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "ecg_timer",
  };
  esp_timer_handle_t ecg_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&ecg_timer_args, &ecg_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(ecg_timer, AD8232_TIMER_PERIOD_US));

  /* Brief settle time for ADC + filters */
  vTaskDelay(pdMS_TO_TICKS(500));

  /* ── 2. Create all sensor tasks ── */
  xTaskCreate(ecg_task, "ecg_task", 4096, NULL, 5, NULL);
  xTaskCreate(weight_task, "weight_task", 4096, NULL, 3, NULL);
  xTaskCreate(max30102_task, "max30102_task", 16384, NULL, 4, NULL);
  xTaskCreate(mlx_task, "mlx_task", 3072, NULL, 2, NULL);
  xTaskCreate(ir_task, "ir_task", 2048, NULL, 3, NULL);
  xTaskCreate(status_task, "status_task", 2048, NULL, 1, NULL);

  ESP_LOGI(TAG, "All sensor tasks launched.");
}
