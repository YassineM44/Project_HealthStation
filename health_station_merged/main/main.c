/**
 * @file main.c
 * @brief Health Station — Merged Sensor Coordinator
 *
 * Runs all four sensors simultaneously via dedicated FreeRTOS tasks:
 *   1. AD8232  ECG        — 500 Hz timer-driven ADC sampling
 *   2. HX711  Weight      — 300 ms polled pill detection
 *   3. MAX30102 HR/SpO2   — interrupt-driven FIFO at 200 Hz
 *   4. MLX90614 Temp      — 1 s polled IR temperature
 */

#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* ── Sensor headers ── */
#include "ad8232.h"
#include "max30102.h"
#include "mlx906.h"
#include "weight.h"

static const char *TAG = "HEALTH";

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

  while (1) {
    if (xSemaphoreTake(s_ecg_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (ad8232_read_sample(&s_ecg, &sample) != ESP_OK) {
        continue;
      }

#if ECG_SERIAL_PLOT
      /* Averaging decimation 500 Hz → 50 Hz for Serial Studio Pro.
       * Instead of dropping 9 samples and keeping 1 (thin decimation),
       * we average all 10 filtered samples.  This reduces random noise
       * by √10 ≈ 3.2× (~10 dB) while preserving the coherent ECG
       * waveform shape (all samples within a QRS peak are high, so
       * their average stays high). */
      acc += sample.filtered_mv;
      if (++decimate >= 10) {
        printf("%.2f\n", acc / 10.0f);
        acc = 0.0f;
        decimate = 0;
      }
#else
      (void)decimate; /* suppress unused-variable warning */
      (void)acc;
#endif
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

  ESP_LOGI(T, "Waiting %" PRIu32 " ms before tare; keep compartment empty",
           (uint32_t)WEIGHT_BOOT_SETTLE_MS);
  vTaskDelay(pdMS_TO_TICKS(WEIGHT_BOOT_SETTLE_MS));

  err = weight_tare(&sensor, WEIGHT_DEFAULT_TARE_SAMPLE_COUNT);
  if (err != ESP_OK) {
    ESP_LOGE(T, "weight_tare failed: %s", esp_err_to_name(err));
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(T, "Tare complete (initial)");

  /* Event-driven: only print when pill state changes */
  bool prev_pill_present = false;
  uint16_t idle_retare_count = 0;

  while (1) {
    err = weight_update(&sensor, &status);
    if (err != ESP_OK) {
      ESP_LOGW(T, "weight_update failed: %s", esp_err_to_name(err));
    } else if (status.pill_present != prev_pill_present) {
      if (status.pill_present) {
        ESP_LOGI(T, "Pill PLACED");
      } else {
        ESP_LOGW(T, "Pill REMOVED");
      }
      prev_pill_present = status.pill_present;
      idle_retare_count = 0;
    }

    /* Auto re-tare every ~3s when idle.  Runs OUTSIDE the if-else so it
     * fires even when weight_update fails.  Uses prev_pill_present
     * (which only updates on confirmed state changes) to avoid
     * retaring while a pill is genuinely present.
     *
     * 10 iterations × 300ms ≈ 3s.  Tare (8 samples at 10 Hz) ≈ 0.8s.
     * Total cycle ≈ 3.8s.  At ~10 units/sec drift → max ~38 counts,
     * well below threshold 150. */
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
#define HO2_STABILIZE_MS 5000
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
  #define HO2_BPM_GRACE_MS 8000U

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
        last_countdown = 5;
        ESP_LOGI(T, "STABILIZING... %d", last_countdown);
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
            if (!bpm_initialized) {
              smoothed_bpm = result.heart_rate_bpm;
              bpm_initialized = true;
            } else {
              float delta_pct = fabsf(result.heart_rate_bpm - smoothed_bpm)
                              / smoothed_bpm;
              if (delta_pct > 0.25f) {
                /* Outlier (>25% off): near-ignore */
                smoothed_bpm = smoothed_bpm * 0.97f
                             + result.heart_rate_bpm * 0.03f;
              } else {
                /* Normal reading: heavy EMA tracking */
                smoothed_bpm = smoothed_bpm * 0.8f
                             + result.heart_rate_bpm * 0.2f;
              }
            }
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
          /* Clear ring buffer so BPM uses only post-stabilization data */
          ring_index = 0;
          ring_count = 0;
          since_last = 0;
          if (latest_result_valid &&
              (now_ms - latest_result_ms) <= HO2_PRINT_PERIOD_MS) {
            /* Grace period: always SpO2-only at start */
            ESP_LOGI(T, "SpO2 %.1f %%", latest_result.spo2_percent);
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
        }
      }

      if (state == HO2_STREAMING && latest_result_valid &&
          latest_result_ms != last_printed_result_ms &&
          (last_print_ms == 0 ||
           (now_ms - last_print_ms) >= HO2_PRINT_PERIOD_MS)) {
        if (bpm_ready && !isnan(latest_result.heart_rate_bpm)) {
          ESP_LOGI(T, "SpO2 %.1f %% | HR %.1f bpm", latest_result.spo2_percent,
                   latest_result.heart_rate_bpm);
        } else {
          ESP_LOGI(T, "SpO2 %.1f %%", latest_result.spo2_percent);
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

#define MLX_WARMUP_READINGS   10
#define MLX_DELTA_ON_C        1.5f   /* obj must exceed baseline by this to trigger */
#define MLX_DELTA_OFF_C       0.7f   /* obj must drop within this of baseline to clear */
#define MLX_BASELINE_ALPHA    0.05f  /* EMA smoothing factor (slower = less drift noise) */

static void mlx_task(void *arg) {
  (void)arg;
  static const char *T = "MLX906";

  if (mlx906_init() != ESP_OK) {
    ESP_LOGE(T, "mlx906_init failed");
    vTaskDelete(NULL);
    return;
  }

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
        } else {
          ESP_LOGI(T, "Object: %.2f C | Ambient: %.2f C", obj_temp, amb_temp);
        }
      }
    } else {
      ESP_LOGW(T, "Failed to read sensor");
    }
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
  ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

  /* ── 1. AD8232 ECG init (must happen here for timer setup) ── */
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
  xTaskCreate(max30102_task, "max30102_task", 8192, NULL, 4, NULL);
  xTaskCreate(mlx_task, "mlx_task", 3072, NULL, 2, NULL);

  ESP_LOGI(TAG, "All sensor tasks launched.");
}
