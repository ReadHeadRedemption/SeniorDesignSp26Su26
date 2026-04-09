#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"

// ======== USER PIN CONFIGURATION ========
// Replace these pins with your actual PCB pins.
#define PIN_X_STEP          GPIO_NUM_2
#define PIN_X_DIR           GPIO_NUM_4
#define PIN_X_EN            GPIO_NUM_1
#define PIN_X_MIN           GPIO_NUM_NC
#define PIN_X_MAX           GPIO_NUM_NC

#define PIN_Y_STEP          GPIO_NUM_NC
#define PIN_Y_DIR           GPIO_NUM_NC
#define PIN_Y_EN            GPIO_NUM_NC
#define PIN_Y_MIN           GPIO_NUM_NC
#define PIN_Y_MAX           GPIO_NUM_NC

#define PIN_Z_STEP          GPIO_NUM_NC
#define PIN_Z_DIR           GPIO_NUM_NC
#define PIN_Z_EN            GPIO_NUM_NC
#define PIN_Z_MIN           GPIO_NUM_NC
#define PIN_Z_MAX           GPIO_NUM_NC

#define PIN_E_STEP          GPIO_NUM_NC
#define PIN_E_DIR           GPIO_NUM_NC
#define PIN_E_EN            GPIO_NUM_NC

#define PIN_Z_PROBE         GPIO_NUM_NC
#define PIN_BED_SSR         GPIO_NUM_NC

#define PIN_SD_MISO         GPIO_NUM_NC
#define PIN_SD_MOSI         GPIO_NUM_NC
#define PIN_SD_SCLK         GPIO_NUM_NC
#define PIN_SD_CS           GPIO_NUM_NC

// Optional SPI touchscreen on a separate general-purpose SPI host (SPI3_HOST / VSPI on classic ESP32).
// Keep this on a separate bus from the SD card to simplify timing and bus ownership.
#define PIN_TFT_MISO        GPIO_NUM_46   // Usually shared with touch MISO
#define PIN_TFT_MOSI        GPIO_NUM_11
#define PIN_TFT_SCLK        GPIO_NUM_10
#define PIN_TFT_CS          GPIO_NUM_14
#define PIN_TFT_DC          GPIO_NUM_12
#define PIN_TFT_RST         GPIO_NUM_13
#define PIN_TFT_BCKL        GPIO_NUM_9
#define PIN_TOUCH_CS        GPIO_NUM_3
#define PIN_TOUCH_IRQ       GPIO_NUM_8   // Optional. Wire if available for better touch detection.

// Thermistor on ADC1 channel 6 => GPIO34 on classic ESP32.
// Change both if you use a different ESP32 variant / pin.
#define BED_THERM_ADC_UNIT      ADC_UNIT_1
#define BED_THERM_ADC_CHANNEL   ADC_CHANNEL_6
#define BED_THERM_ATTEN         ADC_ATTEN_DB_12

// ======== MACHINE CONFIGURATION ========
#define STEPS_PER_MM_X      320.0f
#define STEPS_PER_MM_Y      80.0f
#define STEPS_PER_MM_Z      400.0f
#define STEPS_PER_MM_E      800.0f   // Plunger axis. Replace with your real value.

#define HOMING_FEED_MM_MIN  600.0f
#define JOG_FEED_MM_MIN     1200.0f
#define DEFAULT_FEED_MM_MIN 1200.0f
#define HOMING_BACKOFF_MM   3.0f

#define BED_MAX_C           180.0f
#define BED_MIN_VALID_C     0.0f
#define BED_MAX_VALID_C     220.0f
#define BED_BANGBANG_HYST   2.0f
#define BED_HEATUP_TIMEOUT_MS   (120000) // 2 minutes starter safety timeout
#define BED_MIN_RISE_C          2.0f

#define STEP_PULSE_US       4   // DRV8825 min pulse width is low single-digit us.
#define STEP_MIN_INTERVAL_US 300
#define CONSOLE_LINE_MAX    160

#define MOUNT_POINT         "/sdcard"

#define TFT_HOST            SPI3_HOST
#define TFT_WIDTH           320
#define TFT_HEIGHT          240

#define TFT_COLOR_BG        0x0841
#define TFT_COLOR_PANEL     0x1082
#define TFT_COLOR_BORDER    0x7BEF
#define TFT_COLOR_TEXT      0xFFFF
#define TFT_COLOR_ACCENT    0x07FF
#define TFT_COLOR_ACTIVE    0xFD20
#define TFT_COLOR_GO        0x07E0
#define TFT_COLOR_STOP      0xF800
#define TFT_COLOR_KEY       0x39E7
#define TFT_COLOR_FIELD     0x2104

// Touch calibration for XPT2046. Adjust after you verify raw readings on your panel.
#define TOUCH_RAW_X_MIN     250
#define TOUCH_RAW_X_MAX     3880
#define TOUCH_RAW_Y_MIN     290
#define TOUCH_RAW_Y_MAX     3880
#define TOUCH_SWAP_XY       1
#define TOUCH_INVERT_X      1
#define TOUCH_INVERT_Y      1

#define UI_POLL_MS          120
#define UI_STATUS_REFRESH_MS 1000
#define UI_FIELD_MAX        15

static const char *TAG = "ink_printer_m1";

typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_E,
    AXIS_COUNT
} axis_id_t;

typedef struct {
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t en_pin;
    gpio_num_t min_pin;
    gpio_num_t max_pin;
    float steps_per_mm;
    bool invert_dir;
    bool home_to_min;
    bool enabled_low;
} axis_hw_t;

typedef struct {
    int32_t pos_steps[AXIS_COUNT];
    bool absolute_mode;
    bool homed[3];
    bool faulted;
    char fault_msg[96];
    bool printing;
    bool stop_requested;
    float bed_target_c;
    float bed_temp_c;
    bool heater_enabled;
} machine_state_t;

typedef struct {
    bool has_x, has_y, has_z, has_e, has_f;
    float x, y, z, e, f;
} move_words_t;

static const axis_hw_t g_axes[AXIS_COUNT] = {
    [AXIS_X] = { PIN_X_STEP, PIN_X_DIR, PIN_X_EN, PIN_X_MIN, PIN_X_MAX, STEPS_PER_MM_X, false, true, true },
    [AXIS_Y] = { PIN_Y_STEP, PIN_Y_DIR, PIN_Y_EN, PIN_Y_MIN, PIN_Y_MAX, STEPS_PER_MM_Y, false, true, true },
    [AXIS_Z] = { PIN_Z_STEP, PIN_Z_DIR, PIN_Z_EN, PIN_Z_MIN, PIN_Z_MAX, STEPS_PER_MM_Z, false, true, true },
    [AXIS_E] = { PIN_E_STEP, PIN_E_DIR, PIN_E_EN, GPIO_NUM_NC, GPIO_NUM_NC, STEPS_PER_MM_E, false, true, true },
};

static SemaphoreHandle_t g_state_mutex;
static machine_state_t g_state;
static sdmmc_card_t *g_sdcard;
static adc_oneshot_unit_handle_t g_adc_handle;
static adc_cali_handle_t g_adc_cali_handle;
static bool g_adc_cali_enabled;

static SemaphoreHandle_t g_motion_mutex;
static spi_device_handle_t g_tft_spi;
static spi_device_handle_t g_touch_spi;

SemaphoreHandle_t touch_semaphore = NULL;

// ---------- Utility ----------

static void IRAM_ATTR touch_isr(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( touch_semaphore, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


static inline bool gpio_is_valid(gpio_num_t pin) {
    return pin != GPIO_NUM_NC;
}

static void gpio_set_level_if_valid(gpio_num_t pin, int level) {
    if (gpio_is_valid(pin)) gpio_set_level(pin, level);
}

static void gpio_config_output_if_valid(gpio_num_t pin, int initial_level) {
    if (!gpio_is_valid(pin)) return;
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(pin, initial_level);
}

static void gpio_config_input_pullup_if_valid(gpio_num_t pin) {
    if (!gpio_is_valid(pin)) return;
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

static inline float steps_to_mm(axis_id_t axis, int32_t steps) {
    return (float)steps / g_axes[axis].steps_per_mm;
}

static inline int32_t mm_to_steps(axis_id_t axis, float mm) {
    return lroundf(mm * g_axes[axis].steps_per_mm);
}

static inline bool is_switch_triggered(gpio_num_t pin) {
    if (pin == GPIO_NUM_NC) return false;
    // Starter assumption: switch pulls low when triggered.
    return gpio_get_level(pin) == 0;
}

static void latch_fault(const char *msg) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.faulted = true;
    g_state.printing = false;
    g_state.stop_requested = true;
    g_state.heater_enabled = false;
    g_state.bed_target_c = 0.0f;
    snprintf(g_state.fault_msg, sizeof(g_state.fault_msg), "%s", msg);
    xSemaphoreGive(g_state_mutex);

    gpio_set_level_if_valid(PIN_BED_SSR, 0);
    for (int i = 0; i < AXIS_COUNT; ++i) {
        const int level = g_axes[i].enabled_low ? 1 : 0;
        gpio_set_level(g_axes[i].en_pin, level);
    }
    ESP_LOGE(TAG, "FAULT: %s", msg);
}

static bool machine_faulted(void) {
    bool faulted;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    faulted = g_state.faulted;
    xSemaphoreGive(g_state_mutex);
    return faulted;
}

static void set_bed_target(float target_c) {
    if (target_c < 0.0f) target_c = 0.0f;
    if (target_c > BED_MAX_C) target_c = BED_MAX_C;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.bed_target_c = target_c;
    g_state.heater_enabled = target_c > 0.1f;
    xSemaphoreGive(g_state_mutex);
    ESP_LOGI(TAG, "Bed target set to %.1f C", target_c);
}

static void set_axis_enable(axis_id_t axis, bool enabled) {
    const axis_hw_t *a = &g_axes[axis];
    if (!gpio_is_valid(a->en_pin)) return;
    const int level = a->enabled_low ? (enabled ? 0 : 1) : (enabled ? 1 : 0);
    gpio_set_level(a->en_pin, level);
}

static void set_all_axes_enabled(bool enabled) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
        set_axis_enable((axis_id_t)i, enabled);
    }
}

static void pulse_step(gpio_num_t pin) {
    if (!gpio_is_valid(pin)) return;
    gpio_set_level(pin, 1);
    esp_rom_delay_us(STEP_PULSE_US);
    gpio_set_level(pin, 0);
}

// ---------- Thermistor / heater ----------
static bool adc_calibration_init(void) {
    g_adc_cali_enabled = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = BED_THERM_ADC_UNIT,
        .atten = BED_THERM_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &g_adc_cali_handle) == ESP_OK) {
        g_adc_cali_enabled = true;
        return true;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = BED_THERM_ADC_UNIT,
        .atten = BED_THERM_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &g_adc_cali_handle) == ESP_OK) {
        g_adc_cali_enabled = true;
        return true;
    }
#endif
    ESP_LOGW(TAG, "ADC calibration unavailable; using raw ADC conversion");
    return false;
}

static esp_err_t thermistor_adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BED_THERM_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &g_adc_handle), TAG, "adc_oneshot_new_unit failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BED_THERM_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(g_adc_handle, BED_THERM_ADC_CHANNEL, &chan_cfg), TAG, "adc channel config failed");
    adc_calibration_init();
    return ESP_OK;
}

static float thermistor_c_from_adc_mv(int mv) {
    // Starter values for a 100K NTC, Beta 3950, with 4.7K series pull-up to 3.3V.
    // Change these to match your real thermistor network.
    const float r_series = 4700.0f;
    const float r0 = 100000.0f;
    const float beta = 3950.0f;
    const float t0 = 298.15f; // 25 C in Kelvin
    const float vref = 3300.0f;

    if (mv <= 1 || mv >= (int)(vref - 1.0f)) {
        return NAN;
    }

    const float v = (float)mv;
    const float r_therm = r_series * v / (vref - v);
    const float inv_t = (1.0f / t0) + (1.0f / beta) * logf(r_therm / r0);
    const float temp_k = 1.0f / inv_t;
    return temp_k - 273.15f;
}

static float read_bed_temp_c(void) {
    int raw = 0;
    if (adc_oneshot_read(g_adc_handle, BED_THERM_ADC_CHANNEL, &raw) != ESP_OK) {
        return NAN;
    }

    int mv = raw;
    if (g_adc_cali_enabled) {
        if (adc_cali_raw_to_voltage(g_adc_cali_handle, raw, &mv) != ESP_OK) {
            return NAN;
        }
    } else {
        // Very rough fallback. Use calibration in real hardware.
        mv = (raw * 3300) / 4095;
    }

    return thermistor_c_from_adc_mv(mv);
}

static void heater_task(void *arg) {
    (void)arg;
    int64_t heat_start_ms = 0;
    float start_temp = NAN;

    while (1) {
        const float temp_c = read_bed_temp_c();
        float target_c;
        bool heater_enabled;
        bool faulted;

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_state.bed_temp_c = temp_c;
        target_c = g_state.bed_target_c;
        heater_enabled = g_state.heater_enabled;
        faulted = g_state.faulted;
        xSemaphoreGive(g_state_mutex);

        if (faulted) {
            gpio_set_level_if_valid(PIN_BED_SSR, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (isnan(temp_c) || temp_c < BED_MIN_VALID_C || temp_c > BED_MAX_VALID_C) {
            gpio_set_level_if_valid(PIN_BED_SSR, 0);
            latch_fault("Bed thermistor invalid or out of range");
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (!heater_enabled || target_c < 0.1f) {
            gpio_set_level_if_valid(PIN_BED_SSR, 0);
            heat_start_ms = 0;
            start_temp = temp_c;
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        // Starter bang-bang control suitable for a zero-cross SSR; upgrade later if needed.
        if (temp_c < (target_c - BED_BANGBANG_HYST)) {
            gpio_set_level_if_valid(PIN_BED_SSR, 1);
        } else if (temp_c > (target_c + BED_BANGBANG_HYST)) {
            gpio_set_level_if_valid(PIN_BED_SSR, 0);
        }

        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (heat_start_ms == 0) {
            heat_start_ms = now_ms;
            start_temp = temp_c;
        }

        if ((now_ms - heat_start_ms) > BED_HEATUP_TIMEOUT_MS && (temp_c - start_temp) < BED_MIN_RISE_C) {
            gpio_set_level_if_valid(PIN_BED_SSR, 0);
            latch_fault("Bed failed to warm fast enough");
        }

        if (temp_c > BED_MAX_C + 5.0f) {
            gpio_set_level_if_valid(PIN_BED_SSR, 0);
            latch_fault("Bed over-temperature");
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ---------- Motion ----------
static void set_axis_direction(axis_id_t axis, bool positive) {
    bool level = positive;
    if (g_axes[axis].invert_dir) level = !level;
    gpio_set_level_if_valid(g_axes[axis].dir_pin, level ? 1 : 0);
}

static esp_err_t move_linear_steps(const int32_t target_steps[AXIS_COUNT], float feed_mm_min) {
    if (machine_faulted()) return ESP_FAIL;
    if (feed_mm_min < 1.0f) feed_mm_min = DEFAULT_FEED_MM_MIN;

    xSemaphoreTake(g_motion_mutex, portMAX_DELAY);

    int32_t start[AXIS_COUNT];
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    memcpy(start, g_state.pos_steps, sizeof(start));
    xSemaphoreGive(g_state_mutex);

    int32_t delta[AXIS_COUNT];
    int32_t abs_delta[AXIS_COUNT];
    int32_t max_steps = 0;
    for (int i = 0; i < AXIS_COUNT; ++i) {
        delta[i] = target_steps[i] - start[i];
        abs_delta[i] = abs(delta[i]);
        if (abs_delta[i] > max_steps) max_steps = abs_delta[i];
        set_axis_direction((axis_id_t)i, delta[i] >= 0);
    }

    if (max_steps == 0) {
        xSemaphoreGive(g_motion_mutex);
        return ESP_OK;
    }

    float dx = steps_to_mm(AXIS_X, delta[AXIS_X]);
    float dy = steps_to_mm(AXIS_Y, delta[AXIS_Y]);
    float dz = steps_to_mm(AXIS_Z, delta[AXIS_Z]);
    float de = steps_to_mm(AXIS_E, delta[AXIS_E]);
    float path_mm = sqrtf(dx*dx + dy*dy + dz*dz + de*de);
    if (path_mm < 0.001f) path_mm = fmaxf(fabsf(dx) + fabsf(dy) + fabsf(dz) + fabsf(de), 0.001f);
    float move_time_s = path_mm / (feed_mm_min / 60.0f);
    uint32_t interval_us = (uint32_t)fmaxf((move_time_s * 1e6f) / (float)max_steps, (float)STEP_MIN_INTERVAL_US);

    int32_t err[AXIS_COUNT] = {0};
    int32_t current[AXIS_COUNT];
    memcpy(current, start, sizeof(current));

    set_all_axes_enabled(true);

    for (int32_t i = 0; i < max_steps; ++i) {
        if (machine_faulted()) { xSemaphoreGive(g_motion_mutex); return ESP_FAIL; }

        if (is_switch_triggered(g_axes[AXIS_X].min_pin) && delta[AXIS_X] < 0) { latch_fault("Hit X min during move"); xSemaphoreGive(g_motion_mutex); return ESP_FAIL; }
        if (is_switch_triggered(g_axes[AXIS_X].max_pin) && delta[AXIS_X] > 0) { latch_fault("Hit X max during move"); xSemaphoreGive(g_motion_mutex); return ESP_FAIL; }
        if (is_switch_triggered(g_axes[AXIS_Y].min_pin) && delta[AXIS_Y] < 0) { latch_fault("Hit Y min during move"); xSemaphoreGive(g_motion_mutex); return ESP_FAIL; }
        if (is_switch_triggered(g_axes[AXIS_Y].max_pin) && delta[AXIS_Y] > 0) { latch_fault("Hit Y max during move"); xSemaphoreGive(g_motion_mutex); return ESP_FAIL; }
        if (is_switch_triggered(g_axes[AXIS_Z].min_pin) && delta[AXIS_Z] < 0) { latch_fault("Hit Z min during move"); xSemaphoreGive(g_motion_mutex); return ESP_FAIL; }
        if (is_switch_triggered(g_axes[AXIS_Z].max_pin) && delta[AXIS_Z] > 0) { latch_fault("Hit Z max during move"); xSemaphoreGive(g_motion_mutex); return ESP_FAIL; }

        for (int axis = 0; axis < AXIS_COUNT; ++axis) {
            err[axis] += abs_delta[axis];
            if (err[axis] >= max_steps) {
                pulse_step(g_axes[axis].step_pin);
                err[axis] -= max_steps;
                current[axis] += (delta[axis] >= 0) ? 1 : -1;
            }
        }
        esp_rom_delay_us(interval_us);
    }

    set_all_axes_enabled(false);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    memcpy(g_state.pos_steps, target_steps, sizeof(g_state.pos_steps));
    xSemaphoreGive(g_state_mutex);
    xSemaphoreGive(g_motion_mutex);

    return ESP_OK;
}

static esp_err_t home_single_axis(axis_id_t axis) {
    if (axis == AXIS_E) return ESP_OK;

    const axis_hw_t *a = &g_axes[axis];
    if (a->min_pin == GPIO_NUM_NC) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Move toward min switch.
    set_axis_enable(axis, true);
    set_axis_direction(axis, !a->home_to_min);
    if (a->home_to_min) set_axis_direction(axis, false);

    if (is_switch_triggered(a->min_pin)) {
        // Back off if already on the switch.
        set_axis_direction(axis, true);
        for (int i = 0; i < mm_to_steps(axis, HOMING_BACKOFF_MM); ++i) {
            pulse_step(a->step_pin);
            esp_rom_delay_us(1200);
        }
    }

    set_axis_direction(axis, false);
    for (int i = 0; i < mm_to_steps(axis, 400.0f); ++i) {
        if (is_switch_triggered(a->min_pin)) break;
        pulse_step(a->step_pin);
        esp_rom_delay_us(900);
    }
    if (!is_switch_triggered(a->min_pin)) {
        latch_fault("Homing switch not found");
        return ESP_FAIL;
    }

    // Back off and re-home slowly.
    set_axis_direction(axis, true);
    for (int i = 0; i < mm_to_steps(axis, HOMING_BACKOFF_MM); ++i) {
        pulse_step(a->step_pin);
        esp_rom_delay_us(1200);
    }

    set_axis_direction(axis, false);
    for (int i = 0; i < mm_to_steps(axis, HOMING_BACKOFF_MM * 2.0f); ++i) {
        if (is_switch_triggered(a->min_pin)) break;
        pulse_step(a->step_pin);
        esp_rom_delay_us(2000);
    }
    if (!is_switch_triggered(a->min_pin)) {
        latch_fault("Slow homing switch not found");
        return ESP_FAIL;
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.pos_steps[axis] = 0;
    g_state.homed[axis] = true;
    xSemaphoreGive(g_state_mutex);

    ESP_LOGI(TAG, "Axis %c homed", axis == AXIS_X ? 'X' : axis == AXIS_Y ? 'Y' : 'Z');
    return ESP_OK;
}

static esp_err_t home_all_axes(void) {
    xSemaphoreTake(g_motion_mutex, portMAX_DELAY);
    esp_err_t err = home_single_axis(AXIS_Z);
    if (err == ESP_OK) err = home_single_axis(AXIS_X);
    if (err == ESP_OK) err = home_single_axis(AXIS_Y);
    xSemaphoreGive(g_motion_mutex);
    return err;
}

static esp_err_t jog_relative(float x, float y, float z, float e, float f) {
    int32_t target[AXIS_COUNT];
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    memcpy(target, g_state.pos_steps, sizeof(target));
    xSemaphoreGive(g_state_mutex);

    target[AXIS_X] += mm_to_steps(AXIS_X, x);
    target[AXIS_Y] += mm_to_steps(AXIS_Y, y);
    target[AXIS_Z] += mm_to_steps(AXIS_Z, z);
    target[AXIS_E] += mm_to_steps(AXIS_E, e);
    return move_linear_steps(target, f > 0 ? f : JOG_FEED_MM_MIN);
}

// ---------- G-code ----------
static char *skip_ws(char *s) {
    while (*s && isspace((unsigned char)*s)) ++s;
    return s;
}

static bool read_word_value(const char *line, char letter, float *out) {
    const char *p = line;
    while ((p = strchr(p, letter)) != NULL) {
        ++p;
        char *end = NULL;
        float v = strtof(p, &end);
        if (end != p) {
            *out = v;
            return true;
        }
    }
    return false;
}

static move_words_t parse_move_words(const char *line) {
    move_words_t m = {0};
    m.has_x = read_word_value(line, 'X', &m.x);
    m.has_y = read_word_value(line, 'Y', &m.y);
    m.has_z = read_word_value(line, 'Z', &m.z);
    m.has_e = read_word_value(line, 'E', &m.e);
    m.has_f = read_word_value(line, 'F', &m.f);
    return m;
}

static esp_err_t execute_gcode_line(char *line) {
    char *comment = strchr(line, ';');
    if (comment) *comment = '\0';
    line = skip_ws(line);
    if (*line == '\0') return ESP_OK;

    if (strncasecmp(line, "HOME", 4) == 0) {
        return home_all_axes();
    }

    if (toupper((unsigned char)line[0]) == 'G') {
        int g = atoi(line + 1);
        switch (g) {
            case 0:
            case 1: {
                move_words_t m = parse_move_words(line);
                int32_t target[AXIS_COUNT];
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                memcpy(target, g_state.pos_steps, sizeof(target));
                bool absolute = g_state.absolute_mode;
                xSemaphoreGive(g_state_mutex);

                if (absolute) {
                    if (m.has_x) target[AXIS_X] = mm_to_steps(AXIS_X, m.x);
                    if (m.has_y) target[AXIS_Y] = mm_to_steps(AXIS_Y, m.y);
                    if (m.has_z) target[AXIS_Z] = mm_to_steps(AXIS_Z, m.z);
                    if (m.has_e) target[AXIS_E] = mm_to_steps(AXIS_E, m.e);
                } else {
                    if (m.has_x) target[AXIS_X] += mm_to_steps(AXIS_X, m.x);
                    if (m.has_y) target[AXIS_Y] += mm_to_steps(AXIS_Y, m.y);
                    if (m.has_z) target[AXIS_Z] += mm_to_steps(AXIS_Z, m.z);
                    if (m.has_e) target[AXIS_E] += mm_to_steps(AXIS_E, m.e);
                }
                return move_linear_steps(target, m.has_f ? m.f : DEFAULT_FEED_MM_MIN);
            }
            case 28:
                return home_all_axes();
            case 90:
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state.absolute_mode = true;
                xSemaphoreGive(g_state_mutex);
                return ESP_OK;
            case 91:
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state.absolute_mode = false;
                xSemaphoreGive(g_state_mutex);
                return ESP_OK;
            case 92: {
                move_words_t m = parse_move_words(line);
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                if (m.has_x) g_state.pos_steps[AXIS_X] = mm_to_steps(AXIS_X, m.x);
                if (m.has_y) g_state.pos_steps[AXIS_Y] = mm_to_steps(AXIS_Y, m.y);
                if (m.has_z) g_state.pos_steps[AXIS_Z] = mm_to_steps(AXIS_Z, m.z);
                if (m.has_e) g_state.pos_steps[AXIS_E] = mm_to_steps(AXIS_E, m.e);
                xSemaphoreGive(g_state_mutex);
                return ESP_OK;
            }
            default:
                ESP_LOGW(TAG, "Unsupported G-code: %s", line);
                return ESP_OK;
        }
    }

    if (toupper((unsigned char)line[0]) == 'M') {
        int m = atoi(line + 1);
        switch (m) {
            case 18:
            case 84:
                set_all_axes_enabled(false);
                return ESP_OK;
            case 105: {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                const float temp = g_state.bed_temp_c;
                const float target = g_state.bed_target_c;
                xSemaphoreGive(g_state_mutex);
                printf("ok T:%.1f /%.1f\n", temp, target);
                return ESP_OK;
            }
            case 112:
                latch_fault("Emergency stop");
                return ESP_FAIL;
            case 140: {
                float s = 0;
                if (read_word_value(line, 'S', &s)) set_bed_target(s);
                return ESP_OK;
            }
            case 190: {
                float s = 0;
                if (read_word_value(line, 'S', &s)) set_bed_target(s);
                while (1) {
                    if (machine_faulted()) return ESP_FAIL;
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    float temp = g_state.bed_temp_c;
                    float target = g_state.bed_target_c;
                    xSemaphoreGive(g_state_mutex);
                    if (temp >= target - 1.0f) return ESP_OK;
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
            default:
                ESP_LOGW(TAG, "Unsupported M-code: %s", line);
                return ESP_OK;
        }
    }

    if (strncasecmp(line, "JOG", 3) == 0) {
        move_words_t m = parse_move_words(line);
        return jog_relative(m.has_x ? m.x : 0.0f,
                            m.has_y ? m.y : 0.0f,
                            m.has_z ? m.z : 0.0f,
                            m.has_e ? m.e : 0.0f,
                            m.has_f ? m.f : JOG_FEED_MM_MIN);
    }

    if (strncasecmp(line, "TEMP", 4) == 0) {
        float s = 0.0f;
        if (read_word_value(line, 'S', &s)) {
            set_bed_target(s);
        } else {
            char *p = line + 4;
            set_bed_target(strtof(p, NULL));
        }
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Unknown command: %s", line);
    return ESP_OK;
}

static esp_err_t run_gcode_file(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return ESP_FAIL;
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.printing = true;
    g_state.stop_requested = false;
    xSemaphoreGive(g_state_mutex);

    char line[CONSOLE_LINE_MAX];
    int line_no = 0;
    while (fgets(line, sizeof(line), fp)) {
        ++line_no;
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        bool stop = g_state.stop_requested || g_state.faulted;
        xSemaphoreGive(g_state_mutex);
        if (stop) break;

        esp_err_t err = execute_gcode_line(line);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "G-code error at line %d: %s", line_no, line);
            fclose(fp);
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_state.printing = false;
            xSemaphoreGive(g_state_mutex);
            return err;
        }
    }

    fclose(fp);
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.printing = false;
    xSemaphoreGive(g_state_mutex);
    ESP_LOGI(TAG, "File complete: %s", path);
    return ESP_OK;
}

// ---------- SD ----------
static esp_err_t sdcard_init(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_SD_MOSI,
        .miso_io_num = PIN_SD_MISO,
        .sclk_io_num = PIN_SD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA), TAG, "spi bus init failed");

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = PIN_SD_CS;
    slot_cfg.host_id = SPI2_HOST;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    esp_err_t err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &g_sdcard);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(err));
        spi_bus_free(SPI2_HOST);
        return err;
    }

    sdmmc_card_print_info(stdout, g_sdcard);
    return ESP_OK;
}


// ---------- Touch UI (ILI9341 + XPT2046) ----------
typedef struct {
    int16_t x, y, w, h;
} ui_rect_t;

typedef enum {
    UI_BTN_NONE = 0,
    UI_BTN_FIELD_X,
    UI_BTN_FIELD_Y,
    UI_BTN_FIELD_Z,
    UI_BTN_FIELD_E,
    UI_BTN_KEY_7,
    UI_BTN_KEY_8,
    UI_BTN_KEY_9,
    UI_BTN_KEY_4,
    UI_BTN_KEY_5,
    UI_BTN_KEY_6,
    UI_BTN_KEY_1,
    UI_BTN_KEY_2,
    UI_BTN_KEY_3,
    UI_BTN_KEY_MINUS,
    UI_BTN_KEY_0,
    UI_BTN_KEY_DOT,
    UI_BTN_CLR,
    UI_BTN_BSP,
    UI_BTN_HOME,
    UI_BTN_MOVE
} ui_button_id_t;

typedef struct {
    ui_rect_t rect;
    ui_button_id_t id;
    const char *label;
    uint16_t color;
} ui_button_t;

typedef struct {
    bool present;
    int active_field;
    char target_text[4][UI_FIELD_MAX];
    bool replace_on_next_key[4];
    char status[48];
} ui_state_t;

static ui_state_t g_ui;

static const ui_button_t g_ui_buttons[] = {
    {{10,  36, 160, 28}, UI_BTN_FIELD_X, "X",    TFT_COLOR_FIELD},
    {{10,  68, 160, 28}, UI_BTN_FIELD_Y, "Y",    TFT_COLOR_FIELD},
    {{10, 100, 160, 28}, UI_BTN_FIELD_Z, "Z",    TFT_COLOR_FIELD},
    {{10, 132, 160, 28}, UI_BTN_FIELD_E, "E",    TFT_COLOR_FIELD},

    {{188,  38, 36, 30}, UI_BTN_KEY_7, "7",      TFT_COLOR_KEY},
    {{228,  38, 36, 30}, UI_BTN_KEY_8, "8",      TFT_COLOR_KEY},
    {{268,  38, 36, 30}, UI_BTN_KEY_9, "9",      TFT_COLOR_KEY},
    {{188,  72, 36, 30}, UI_BTN_KEY_4, "4",      TFT_COLOR_KEY},
    {{228,  72, 36, 30}, UI_BTN_KEY_5, "5",      TFT_COLOR_KEY},
    {{268,  72, 36, 30}, UI_BTN_KEY_6, "6",      TFT_COLOR_KEY},
    {{188, 106, 36, 30}, UI_BTN_KEY_1, "1",      TFT_COLOR_KEY},
    {{228, 106, 36, 30}, UI_BTN_KEY_2, "2",      TFT_COLOR_KEY},
    {{268, 106, 36, 30}, UI_BTN_KEY_3, "3",      TFT_COLOR_KEY},
    {{188, 140, 36, 30}, UI_BTN_KEY_MINUS, "-",  TFT_COLOR_KEY},
    {{228, 140, 36, 30}, UI_BTN_KEY_0, "0",      TFT_COLOR_KEY},
    {{268, 140, 36, 30}, UI_BTN_KEY_DOT, ".",    TFT_COLOR_KEY},

    {{188, 176, 56, 28}, UI_BTN_CLR,   "CLR",    TFT_COLOR_STOP},
    {{248, 176, 56, 28}, UI_BTN_BSP,   "BSP",    TFT_COLOR_BORDER},

    {{10,  180, 76, 40}, UI_BTN_HOME,  "HOME",   TFT_COLOR_ACTIVE},
    {{94,  180, 76, 40}, UI_BTN_MOVE,  "MOVE",   TFT_COLOR_GO},
};

typedef struct {
    char c;
    uint8_t cols[5];
} glyph5x7_t;

static const glyph5x7_t g_font5x7[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'.', {0x00,0x00,0x60,0x60,0x00}},
    {'/', {0x20,0x10,0x08,0x04,0x02}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x42,0x61,0x51,0x49,0x46}},
    {'3', {0x21,0x41,0x45,0x4B,0x31}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {':', {0x00,0x36,0x36,0x00,0x00}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}},
    {'V', {0x1F,0x20,0x40,0x20,0x1F}},
    {'X', {0x63,0x14,0x08,0x14,0x63}},
    {'Y', {0x03,0x04,0x78,0x04,0x03}},
    {'Z', {0x61,0x51,0x49,0x45,0x43}},
};

static const uint8_t *font_lookup(char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    for (size_t i = 0; i < sizeof(g_font5x7) / sizeof(g_font5x7[0]); ++i) {
        if (g_font5x7[i].c == c) return g_font5x7[i].cols;
    }
    return g_font5x7[0].cols;
}

static bool ui_is_available(void) {
    return gpio_is_valid(PIN_TFT_MOSI) && gpio_is_valid(PIN_TFT_SCLK) &&
           gpio_is_valid(PIN_TFT_CS) && gpio_is_valid(PIN_TFT_DC) &&
           gpio_is_valid(PIN_TOUCH_CS);
}

static esp_err_t tft_spi_tx(spi_device_handle_t dev, const void *tx_buf, size_t bytes) {
    if (bytes == 0) return ESP_OK;
    spi_transaction_t t = {
        .length = bytes * 8,
        .tx_buffer = tx_buf,
    };
    return spi_device_polling_transmit(dev, &t);
}

static esp_err_t tft_write_command(uint8_t cmd) {
    gpio_set_level(PIN_TFT_DC, 0);
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
    };
    t.tx_data[0] = cmd;
    return spi_device_polling_transmit(g_tft_spi, &t);
}

static esp_err_t tft_write_data(const void *data, size_t bytes) {
    gpio_set_level(PIN_TFT_DC, 1);
    return tft_spi_tx(g_tft_spi, data, bytes);
}

static void tft_hw_reset(void) {
    if (!gpio_is_valid(PIN_TFT_RST)) return;
    gpio_set_level(PIN_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(PIN_TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

static void tft_write_u16_be(uint16_t value, uint8_t out[2]) {
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)(value & 0xFF);
}

static esp_err_t tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];
    ESP_RETURN_ON_ERROR(tft_write_command(0x2A), TAG, "col addr cmd failed");
    tft_write_u16_be(x0, &data[0]);
    tft_write_u16_be(x1, &data[2]);
    ESP_RETURN_ON_ERROR(tft_write_data(data, sizeof(data)), TAG, "col addr data failed");

    ESP_RETURN_ON_ERROR(tft_write_command(0x2B), TAG, "row addr cmd failed");
    tft_write_u16_be(y0, &data[0]);
    tft_write_u16_be(y1, &data[2]);
    ESP_RETURN_ON_ERROR(tft_write_data(data, sizeof(data)), TAG, "row addr data failed");

    return tft_write_command(0x2C);
}

static esp_err_t tft_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (w <= 0 || h <= 0) return ESP_OK;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if ((x + w) > TFT_WIDTH)  w = TFT_WIDTH - x;
    if ((y + h) > TFT_HEIGHT) h = TFT_HEIGHT - y;
    if (w <= 0 || h <= 0) return ESP_OK;

    ESP_RETURN_ON_ERROR(tft_set_addr_window(x, y, x + w - 1, y + h - 1), TAG, "addr window failed");

    uint8_t chunk[2 * 64];
    for (size_t i = 0; i < sizeof(chunk); i += 2) {
        chunk[i] = (uint8_t)(color >> 8);
        chunk[i + 1] = (uint8_t)(color & 0xFF);
    }

    const int total_px = w * h;
    int sent_px = 0;
    while (sent_px < total_px) {
        int batch_px = total_px - sent_px;
        if (batch_px > 64) batch_px = 64;
        ESP_RETURN_ON_ERROR(tft_write_data(chunk, batch_px * 2), TAG, "pixel push failed");
        sent_px += batch_px;
    }
    return ESP_OK;
}

static esp_err_t tft_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    ESP_RETURN_ON_ERROR(tft_fill_rect(x, y, w, 1, color), TAG, "draw rect top failed");
    ESP_RETURN_ON_ERROR(tft_fill_rect(x, y + h - 1, w, 1, color), TAG, "draw rect bottom failed");
    ESP_RETURN_ON_ERROR(tft_fill_rect(x, y, 1, h, color), TAG, "draw rect left failed");
    return tft_fill_rect(x + w - 1, y, 1, h, color);
}

static esp_err_t tft_draw_char(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
    const uint8_t *cols = font_lookup(c);
    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 7; ++row) {
            const bool on = (cols[col] >> row) & 0x01;
            ESP_RETURN_ON_ERROR(
                tft_fill_rect(x + col * scale, y + row * scale, scale, scale, on ? fg : bg),
                TAG, "char pixel failed"
            );
        }
    }
    return tft_fill_rect(x + 5 * scale, y, scale, 7 * scale, bg);
}

static esp_err_t tft_draw_text(int16_t x, int16_t y, const char *text, uint16_t fg, uint16_t bg, uint8_t scale) {
    for (const char *p = text; *p; ++p) {
        ESP_RETURN_ON_ERROR(tft_draw_char(x, y, *p, fg, bg, scale), TAG, "draw char failed");
        x += 6 * scale;
    }
    return ESP_OK;
}

static esp_err_t tft_init_panel(void) {
    if (!ui_is_available()) return ESP_ERR_NOT_FOUND;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_TFT_MOSI,
        .miso_io_num = PIN_TFT_MISO,
        .sclk_io_num = PIN_TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(TFT_HOST, &bus_cfg, SPI_DMA_CH_AUTO), TAG, "TFT SPI bus init failed");

    spi_device_interface_config_t tft_cfg = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_TFT_CS,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TFT_HOST, &tft_cfg, &g_tft_spi), TAG, "TFT add device failed");

    spi_device_interface_config_t touch_cfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_TOUCH_CS,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TFT_HOST, &touch_cfg, &g_touch_spi), TAG, "Touch add device failed");

    gpio_config_output_if_valid(PIN_TFT_DC, 0);
    gpio_config_output_if_valid(PIN_TFT_RST, 1);
    gpio_config_output_if_valid(PIN_TFT_BCKL, 0);
    // gpio_config_input_pullup_if_valid(PIN_TOUCH_IRQ);

    tft_hw_reset();

    ESP_RETURN_ON_ERROR(tft_write_command(0x01), TAG, "SWRESET failed");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(tft_write_command(0x28), TAG, "DISPOFF failed");

    const uint8_t cmd_cf[] = {0x00, 0xC1, 0x30};
    ESP_RETURN_ON_ERROR(tft_write_command(0xCF), TAG, "cmd CF failed");
    ESP_RETURN_ON_ERROR(tft_write_data(cmd_cf, sizeof(cmd_cf)), TAG, "data CF failed");

    const uint8_t cmd_ed[] = {0x64, 0x03, 0x12, 0x81};
    ESP_RETURN_ON_ERROR(tft_write_command(0xED), TAG, "cmd ED failed");
    ESP_RETURN_ON_ERROR(tft_write_data(cmd_ed, sizeof(cmd_ed)), TAG, "data ED failed");

    const uint8_t cmd_e8[] = {0x85, 0x00, 0x78};
    ESP_RETURN_ON_ERROR(tft_write_command(0xE8), TAG, "cmd E8 failed");
    ESP_RETURN_ON_ERROR(tft_write_data(cmd_e8, sizeof(cmd_e8)), TAG, "data E8 failed");

    const uint8_t cmd_cb[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
    ESP_RETURN_ON_ERROR(tft_write_command(0xCB), TAG, "cmd CB failed");
    ESP_RETURN_ON_ERROR(tft_write_data(cmd_cb, sizeof(cmd_cb)), TAG, "data CB failed");

    const uint8_t cmd_f7[] = {0x20};
    ESP_RETURN_ON_ERROR(tft_write_command(0xF7), TAG, "cmd F7 failed");
    ESP_RETURN_ON_ERROR(tft_write_data(cmd_f7, sizeof(cmd_f7)), TAG, "data F7 failed");

    const uint8_t cmd_ea[] = {0x00, 0x00};
    ESP_RETURN_ON_ERROR(tft_write_command(0xEA), TAG, "cmd EA failed");
    ESP_RETURN_ON_ERROR(tft_write_data(cmd_ea, sizeof(cmd_ea)), TAG, "data EA failed");

    const uint8_t power1[] = {0x23};
    ESP_RETURN_ON_ERROR(tft_write_command(0xC0), TAG, "Power control 1 failed");
    ESP_RETURN_ON_ERROR(tft_write_data(power1, sizeof(power1)), TAG, "Power control 1 data failed");

    const uint8_t power2[] = {0x10};
    ESP_RETURN_ON_ERROR(tft_write_command(0xC1), TAG, "Power control 2 failed");
    ESP_RETURN_ON_ERROR(tft_write_data(power2, sizeof(power2)), TAG, "Power control 2 data failed");

    const uint8_t vm1[] = {0x3E, 0x28};
    ESP_RETURN_ON_ERROR(tft_write_command(0xC5), TAG, "VCOM failed");
    ESP_RETURN_ON_ERROR(tft_write_data(vm1, sizeof(vm1)), TAG, "VCOM data failed");

    const uint8_t vm2[] = {0x86};
    ESP_RETURN_ON_ERROR(tft_write_command(0xC7), TAG, "VCOM2 failed");
    ESP_RETURN_ON_ERROR(tft_write_data(vm2, sizeof(vm2)), TAG, "VCOM2 data failed");

    const uint8_t madctl[] = {0x28}; // Landscape, BGR
    ESP_RETURN_ON_ERROR(tft_write_command(0x36), TAG, "MADCTL failed");
    ESP_RETURN_ON_ERROR(tft_write_data(madctl, sizeof(madctl)), TAG, "MADCTL data failed");

    const uint8_t pixfmt[] = {0x55}; // RGB565
    ESP_RETURN_ON_ERROR(tft_write_command(0x3A), TAG, "PIXFMT failed");
    ESP_RETURN_ON_ERROR(tft_write_data(pixfmt, sizeof(pixfmt)), TAG, "PIXFMT data failed");

    const uint8_t frame[] = {0x00, 0x18};
    ESP_RETURN_ON_ERROR(tft_write_command(0xB1), TAG, "FRMCTR1 failed");
    ESP_RETURN_ON_ERROR(tft_write_data(frame, sizeof(frame)), TAG, "FRMCTR1 data failed");

    const uint8_t dfc[] = {0x08, 0x82, 0x27};
    ESP_RETURN_ON_ERROR(tft_write_command(0xB6), TAG, "DFC failed");
    ESP_RETURN_ON_ERROR(tft_write_data(dfc, sizeof(dfc)), TAG, "DFC data failed");

    const uint8_t gamma_off[] = {0x00};
    ESP_RETURN_ON_ERROR(tft_write_command(0xF2), TAG, "3Gamma disable failed");
    ESP_RETURN_ON_ERROR(tft_write_data(gamma_off, sizeof(gamma_off)), TAG, "3Gamma data failed");

    const uint8_t gamma_set[] = {0x01};
    ESP_RETURN_ON_ERROR(tft_write_command(0x26), TAG, "Gamma set failed");
    ESP_RETURN_ON_ERROR(tft_write_data(gamma_set, sizeof(gamma_set)), TAG, "Gamma data failed");

    const uint8_t pos_gamma[] = {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00};
    ESP_RETURN_ON_ERROR(tft_write_command(0xE0), TAG, "Positive gamma failed");
    ESP_RETURN_ON_ERROR(tft_write_data(pos_gamma, sizeof(pos_gamma)), TAG, "Positive gamma data failed");

    const uint8_t neg_gamma[] = {0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F};
    ESP_RETURN_ON_ERROR(tft_write_command(0xE1), TAG, "Negative gamma failed");
    ESP_RETURN_ON_ERROR(tft_write_data(neg_gamma, sizeof(neg_gamma)), TAG, "Negative gamma data failed");

    ESP_RETURN_ON_ERROR(tft_write_command(0x11), TAG, "Sleep out failed");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(tft_write_command(0x29), TAG, "Display on failed");
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level_if_valid(PIN_TFT_BCKL, 1);
    return ESP_OK;
}

static uint16_t xpt2046_read12(uint8_t command) {
    uint8_t tx[3] = { command, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
        .rxlength = 24,
        .rx_buffer = rx,
    };
    if (spi_device_polling_transmit(g_touch_spi, &t) != ESP_OK) return 0;
    return (uint16_t)(((rx[1] << 8) | rx[2]) >> 3);
}

static int16_t map_touch_axis(uint16_t raw, uint16_t raw_min, uint16_t raw_max, int16_t out_max, bool invert) {
    if (raw <= raw_min) return invert ? out_max : 0;
    if (raw >= raw_max) return invert ? 0 : out_max;
    int32_t scaled = (int32_t)(raw - raw_min) * out_max / (int32_t)(raw_max - raw_min);
    if (invert) scaled = out_max - scaled;
    return (int16_t)scaled;
}

static bool touch_read_screen(int16_t *sx, int16_t *sy) {
    if (!g_touch_spi) return false;
    if (gpio_is_valid(PIN_TOUCH_IRQ) && gpio_get_level(PIN_TOUCH_IRQ) != 0) return false;

    // Throw away the first reading after command to let the sample settle.
    (void)xpt2046_read12(0xD0);
    (void)xpt2046_read12(0x90);

    uint32_t sum_x = 0, sum_y = 0;
    const int samples = 1;
    for (int i = 0; i < samples; ++i) {
        sum_x += xpt2046_read12(0xD0);
        sum_y += xpt2046_read12(0x90);
    }

    printf("(%d, ", xpt2046_read12(0xD0));
    printf("%d)", xpt2046_read12(0x90));

    uint16_t raw_x = (uint16_t)(sum_x / samples);
    uint16_t raw_y = (uint16_t)(sum_y / samples);
    if (raw_x == 0 || raw_y == 0) return false;

    int16_t tx, ty;

    tx = map_touch_axis(raw_x, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, TFT_HEIGHT - 1, TOUCH_INVERT_X);
    ty = map_touch_axis(raw_y, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, TFT_WIDTH - 1, TOUCH_INVERT_Y);

    if (tx < 0 || tx >= TFT_HEIGHT || ty < 0 || ty >= TFT_WIDTH) return false;
    *sx = ty;
    *sy = tx;
    return true;
}

static bool ui_rect_contains(ui_rect_t r, int16_t x, int16_t y) {
    return x >= r.x && x < (r.x + r.w) && y >= r.y && y < (r.y + r.h);
}

static void ui_status(const char *msg) {
    snprintf(g_ui.status, sizeof(g_ui.status), "%s", msg ? msg : "");
}

static void ui_sync_fields_from_position(void) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    snprintf(g_ui.target_text[AXIS_X], UI_FIELD_MAX, "%.2f", steps_to_mm(AXIS_X, g_state.pos_steps[AXIS_X]));
    snprintf(g_ui.target_text[AXIS_Y], UI_FIELD_MAX, "%.2f", steps_to_mm(AXIS_Y, g_state.pos_steps[AXIS_Y]));
    snprintf(g_ui.target_text[AXIS_Z], UI_FIELD_MAX, "%.2f", steps_to_mm(AXIS_Z, g_state.pos_steps[AXIS_Z]));
    snprintf(g_ui.target_text[AXIS_E], UI_FIELD_MAX, "%.2f", steps_to_mm(AXIS_E, g_state.pos_steps[AXIS_E]));
    xSemaphoreGive(g_state_mutex);
    for (int i = 0; i < 4; ++i) g_ui.replace_on_next_key[i] = true;
}

static void ui_draw_button(const ui_button_t *btn) {
    uint16_t fill = btn->color;
    if (btn->id >= UI_BTN_FIELD_X && btn->id <= UI_BTN_FIELD_E) {
        int field = btn->id - UI_BTN_FIELD_X;
        if (field == g_ui.active_field) fill = TFT_COLOR_ACCENT;
    }

    tft_fill_rect(btn->rect.x, btn->rect.y, btn->rect.w, btn->rect.h, fill);
    tft_draw_rect(btn->rect.x, btn->rect.y, btn->rect.w, btn->rect.h, TFT_COLOR_BORDER);

    if (btn->id >= UI_BTN_FIELD_X && btn->id <= UI_BTN_FIELD_E) {
        char label[24];
        const char axis_name = (btn->id == UI_BTN_FIELD_X) ? 'X' :
                               (btn->id == UI_BTN_FIELD_Y) ? 'Y' :
                               (btn->id == UI_BTN_FIELD_Z) ? 'Z' : 'E';
        int field = (btn->id == UI_BTN_FIELD_X) ? 0 :
                        (btn->id == UI_BTN_FIELD_Y) ? 1 :
                        (btn->id == UI_BTN_FIELD_Z) ? 2 : 3;
        snprintf(label, sizeof(label), "%c %s", axis_name, g_ui.target_text[field]);
        tft_draw_text(btn->rect.x + 6, btn->rect.y + 8, label, TFT_COLOR_TEXT, fill, 2);
    } else {
        int16_t tx = btn->rect.x + 6;
        int16_t ty = btn->rect.y + 8;
        if (strlen(btn->label) <= 2) {
            tx = btn->rect.x + (btn->rect.w - ((int)strlen(btn->label) * 12 - 2)) / 2;
        }
        tft_draw_text(tx, ty, btn->label, TFT_COLOR_TEXT, fill, 2);
    }
}

static void ui_draw_static(void) {
    tft_fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_COLOR_BG);
    tft_fill_rect(0, 0, TFT_WIDTH, 24, TFT_COLOR_PANEL);
    tft_draw_text(8, 6, "INK PRINTER", TFT_COLOR_TEXT, TFT_COLOR_PANEL, 2);

    for (size_t i = 0; i < sizeof(g_ui_buttons) / sizeof(g_ui_buttons[0]); ++i) {
        ui_draw_button(&g_ui_buttons[i]);
    }

    tft_draw_text(10, 164, "TARGETS", TFT_COLOR_TEXT, TFT_COLOR_BG, 2);
}

static void ui_draw_dynamic(void) {
    char pos_line[64];
    char bed_line[48];
    char status_line[200];

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    snprintf(pos_line, sizeof(pos_line), "X%.1f Y%.1f Z%.1f E%.1f",
             steps_to_mm(AXIS_X, g_state.pos_steps[AXIS_X]),
             steps_to_mm(AXIS_Y, g_state.pos_steps[AXIS_Y]),
             steps_to_mm(AXIS_Z, g_state.pos_steps[AXIS_Z]),
             steps_to_mm(AXIS_E, g_state.pos_steps[AXIS_E]));
    snprintf(bed_line, sizeof(bed_line), "BED %.1f/%.1f",
             g_state.bed_temp_c, g_state.bed_target_c);
    snprintf(status_line, sizeof(status_line), "%s %s %s",
             g_state.faulted ? "FAULT " : "",
             g_state.faulted ? g_state.fault_msg : "",
             (!g_state.faulted && g_ui.status[0]) ? g_ui.status : (!g_state.faulted ? "READY" : ""));
    xSemaphoreGive(g_state_mutex);

    tft_fill_rect(110, 2, 206, 10, TFT_COLOR_PANEL);
    tft_draw_text(112, 6, pos_line, TFT_COLOR_TEXT, TFT_COLOR_PANEL, 1);

    tft_fill_rect(180, 214, 132, 10, TFT_COLOR_BG);
    tft_draw_text(180, 216, bed_line, TFT_COLOR_TEXT, TFT_COLOR_BG, 1);

    tft_fill_rect(10, 224, 300, 12, TFT_COLOR_BG);
    tft_draw_text(10, 226, status_line, TFT_COLOR_TEXT, TFT_COLOR_BG, 1);

    for (int i = 0; i < 4; ++i) ui_draw_button(&g_ui_buttons[i]);
}

static void ui_select_field(int field) {
    if (field < 0 || field > 3) return;
    g_ui.active_field = field;
    g_ui.replace_on_next_key[field] = true;
    ui_status("ENTER TARGET");
    ui_draw_dynamic();
}

static void ui_clear_active_field(void) {
    g_ui.target_text[g_ui.active_field][0] = '\0';
    g_ui.replace_on_next_key[g_ui.active_field] = false;
    ui_draw_dynamic();
}

static void ui_backspace_active_field(void) {
    size_t len = strlen(g_ui.target_text[g_ui.active_field]);
    if (len > 0) g_ui.target_text[g_ui.active_field][len - 1] = '\0';
    g_ui.replace_on_next_key[g_ui.active_field] = false;
    ui_draw_dynamic();
}

static void ui_append_char(char c) {
    char *buf = g_ui.target_text[g_ui.active_field];
    if (g_ui.replace_on_next_key[g_ui.active_field]) {
        buf[0] = '\0';
        g_ui.replace_on_next_key[g_ui.active_field] = false;
    }

    if (c == '.' && strchr(buf, '.') != NULL) return;
    if (c == '-' && strchr(buf, '-') != NULL) return;

    size_t len = strlen(buf);
    if (len + 1 >= UI_FIELD_MAX) return;

    if (c == '-' && len > 0) return;
    buf[len] = c;
    buf[len + 1] = '\0';
    ui_draw_dynamic();
}

static esp_err_t ui_move_to_targets(void) {
    int32_t target[AXIS_COUNT];
    float values[4];

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    target[AXIS_X] = g_state.pos_steps[AXIS_X];
    target[AXIS_Y] = g_state.pos_steps[AXIS_Y];
    target[AXIS_Z] = g_state.pos_steps[AXIS_Z];
    target[AXIS_E] = g_state.pos_steps[AXIS_E];
    bool printing = g_state.printing;
    bool faulted = g_state.faulted;
    xSemaphoreGive(g_state_mutex);

    if (faulted) {
        ui_status("CLEAR FAULT FIRST");
        ui_draw_dynamic();
        return ESP_FAIL;
    }
    if (printing) {
        ui_status("BUSY PRINTING");
        ui_draw_dynamic();
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < 4; ++i) {
        if (g_ui.target_text[i][0] == '\0') {
            values[i] = steps_to_mm((axis_id_t)i, target[i]);
        } else {
            values[i] = strtof(g_ui.target_text[i], NULL);
        }
    }

    target[AXIS_X] = mm_to_steps(AXIS_X, values[0]);
    target[AXIS_Y] = mm_to_steps(AXIS_Y, values[1]);
    target[AXIS_Z] = mm_to_steps(AXIS_Z, values[2]);
    target[AXIS_E] = mm_to_steps(AXIS_E, values[3]);

    ui_status("MOVING...");
    ui_draw_dynamic();
    esp_err_t err = move_linear_steps(target, DEFAULT_FEED_MM_MIN);
    if (err == ESP_OK) {
        ui_sync_fields_from_position();
        ui_status("MOVE DONE");
    } else {
        ui_status("MOVE FAILED");
    }
    ui_draw_dynamic();
    return err;
}

static esp_err_t ui_home(void) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    bool printing = g_state.printing;
    bool faulted = g_state.faulted;
    xSemaphoreGive(g_state_mutex);

    if (faulted) {
        ui_status("CLEAR FAULT FIRST");
        ui_draw_dynamic();
        return ESP_FAIL;
    }
    if (printing) {
        ui_status("BUSY PRINTING");
        ui_draw_dynamic();
        return ESP_ERR_INVALID_STATE;
    }

    ui_status("HOMING...");
    ui_draw_dynamic();
    esp_err_t err = home_all_axes();
    if (err == ESP_OK) {
        ui_sync_fields_from_position();
        ui_status("HOME DONE");
    } else {
        ui_status("HOME FAILED");
    }
    ui_draw_dynamic();
    return err;
}

static ui_button_id_t ui_hit_test(int16_t x, int16_t y) {
    for (size_t i = 0; i < sizeof(g_ui_buttons) / sizeof(g_ui_buttons[0]); ++i) {
        if (ui_rect_contains(g_ui_buttons[i].rect, x, y)) return g_ui_buttons[i].id;
    }
    return UI_BTN_NONE;
}

static void ui_handle_button(ui_button_id_t id) {
    switch (id) {
        case UI_BTN_FIELD_X: ui_select_field(AXIS_X); break;
        case UI_BTN_FIELD_Y: ui_select_field(AXIS_Y); break;
        case UI_BTN_FIELD_Z: ui_select_field(AXIS_Z); break;
        case UI_BTN_FIELD_E: ui_select_field(AXIS_E); break;
        case UI_BTN_KEY_0: ui_append_char('0'); break;
        case UI_BTN_KEY_1: ui_append_char('1'); break;
        case UI_BTN_KEY_2: ui_append_char('2'); break;
        case UI_BTN_KEY_3: ui_append_char('3'); break;
        case UI_BTN_KEY_4: ui_append_char('4'); break;
        case UI_BTN_KEY_5: ui_append_char('5'); break;
        case UI_BTN_KEY_6: ui_append_char('6'); break;
        case UI_BTN_KEY_7: ui_append_char('7'); break;
        case UI_BTN_KEY_8: ui_append_char('8'); break;
        case UI_BTN_KEY_9: ui_append_char('9'); break;
        case UI_BTN_KEY_MINUS: ui_append_char('-'); break;
        case UI_BTN_KEY_DOT: ui_append_char('.'); break;
        case UI_BTN_CLR: ui_clear_active_field(); break;
        case UI_BTN_BSP: ui_backspace_active_field(); break;
        case UI_BTN_HOME: (void)ui_home(); break;
        case UI_BTN_MOVE: (void)ui_move_to_targets(); break;
        default: break;
    }
}

static void ui_task(void *arg) {
    (void)arg;

    if (tft_init_panel() != ESP_OK) {
        ESP_LOGW(TAG, "Touch UI disabled. Check TFT/touch pin definitions.");
        vTaskDelete(NULL);
        return;
    }

    memset(&g_ui, 0, sizeof(g_ui));
    g_ui.present = true;
    g_ui.active_field = AXIS_X;
    ui_sync_fields_from_position();
    ui_status("READY");

    ui_draw_static();
    ui_draw_dynamic();

    bool touch_latched = false;
    int64_t last_status_redraw_ms = 0;

    while (xSemaphoreTake(touch_semaphore, portMAX_DELAY) == pdTRUE) {
        printf("HELLO3");
        int16_t tx = 0, ty = 0;
        const bool pressed = touch_read_screen(&tx, &ty);


        if (pressed && !touch_latched) {
            touch_latched = true;
            ui_button_id_t id = ui_hit_test(tx, ty);
            if (id != UI_BTN_NONE) ui_handle_button(id);
        } else if (!pressed) {
            touch_latched = false;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        if ((now_ms - last_status_redraw_ms) > UI_STATUS_REFRESH_MS) {
            ui_draw_dynamic();
            last_status_redraw_ms = now_ms;
        }

        printf("REFINED COORDINATES: (%d, %d)\n", tx, ty);

    }
}

// ---------- Console ----------
static void print_status(void) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    printf("X=%.3f Y=%.3f Z=%.3f E=%.3f | Bed=%.1f/%.1f | abs=%d | printing=%d | fault=%d\n",
           steps_to_mm(AXIS_X, g_state.pos_steps[AXIS_X]),
           steps_to_mm(AXIS_Y, g_state.pos_steps[AXIS_Y]),
           steps_to_mm(AXIS_Z, g_state.pos_steps[AXIS_Z]),
           steps_to_mm(AXIS_E, g_state.pos_steps[AXIS_E]),
           g_state.bed_temp_c,
           g_state.bed_target_c,
           g_state.absolute_mode,
           g_state.printing,
           g_state.faulted);
    if (g_state.faulted) {
        printf("FAULT: %s\n", g_state.fault_msg);
    }
    xSemaphoreGive(g_state_mutex);
}

static void print_help(void) {
    printf("\nCommands:\n");
    printf("  status\n");
    printf("  home\n");
    printf("  jog X10 Y0 Z0 E0.5 F1200\n");
    printf("  temp 60        or  M140 S60\n");
    printf("  run /sdcard/job.gcode\n");
    printf("  stop\n");
    printf("  Any supported G-code: G0/G1/G28/G90/G91/G92/M105/M140/M190/M112/M18/M84\n\n");
}

static void console_task(void *arg) {
    (void)arg;

    print_help();
    while (1) {
        fflush(stdout);
        char *line = linenoise("esp> ");
        if (line == NULL) continue;

        line[strcspn(line, "\r\n")] = '\0';
        char *cmd = skip_ws(line);

        if (*cmd == '\0') {
            linenoiseFree(line);
            continue;
        }

        if (strcasecmp(cmd, "help") == 0) {
            print_help();
            linenoiseFree(line);
            continue;
        }
        if (strcasecmp(cmd, "status") == 0) {
            print_status();
            linenoiseFree(line);
            continue;
        }
        if (strcasecmp(cmd, "stop") == 0) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_state.stop_requested = true;
            g_state.printing = false;
            g_state.heater_enabled = false;
            g_state.bed_target_c = 0.0f;
            xSemaphoreGive(g_state_mutex);
            gpio_set_level_if_valid(PIN_BED_SSR, 0);
            set_all_axes_enabled(false);
            ui_status("STOPPED");
            ESP_LOGW(TAG, "Stop requested");
            linenoiseFree(line);
            continue;
        }
        if (strncasecmp(cmd, "run ", 4) == 0) {
            char *path = skip_ws(cmd + 4);
            (void)run_gcode_file(path);
            if (g_ui.present) {
                ui_sync_fields_from_position();
                ui_status("FILE DONE");
            }
            linenoiseFree(line);
            continue;
        }

        (void)execute_gcode_line(cmd);
        if (g_ui.present) {
            ui_sync_fields_from_position();
            ui_status("SERIAL CMD");
        }

        linenoiseHistoryAdd(line);
        linenoiseFree(line);
    }
}

// ---------- Init ----------
static void gpio_init_all(void) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
        const axis_hw_t *a = &g_axes[i];
        gpio_config_output_if_valid(a->step_pin, 0);
        gpio_config_output_if_valid(a->dir_pin, 0);
        gpio_config_output_if_valid(a->en_pin, a->enabled_low ? 1 : 0);
        gpio_config_input_pullup_if_valid(a->min_pin);
        gpio_config_input_pullup_if_valid(a->max_pin);
    }

    gpio_config_input_pullup_if_valid(PIN_Z_PROBE);
    gpio_config_output_if_valid(PIN_BED_SSR, 0);

    set_all_axes_enabled(false);
}

void app_main(void) {
    // Set up serial command library linenoise for ESP32 serial input
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_console_new_repl_uart(&uart_config, &repl_config, &repl);

    // Create mutexes
    g_state_mutex = xSemaphoreCreateMutex();
    g_motion_mutex = xSemaphoreCreateMutex();
    memset(&g_state, 0, sizeof(g_state));
    g_state.absolute_mode = true;

    // Create touch semaphore
    touch_semaphore = xSemaphoreCreateBinary();

    // Initiate GPIO
    gpio_init_all();

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // Rising edge interrupt trigger
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_TOUCH_IRQ),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_TOUCH_IRQ, touch_isr, NULL);

    // Initiate thermistor ADC
    // ESP_ERROR_CHECK(thermistor_adc_init());

    if (gpio_is_valid(PIN_SD_MISO) && gpio_is_valid(PIN_SD_MOSI) &&
        gpio_is_valid(PIN_SD_SCLK) && gpio_is_valid(PIN_SD_CS)) {
        if (sdcard_init() != ESP_OK) {
            ESP_LOGW(TAG, "Continuing without SD card. You can still jog/home over serial.");
        }
    } else {
        ESP_LOGW(TAG, "SD card disabled. Set PIN_SD_* to enable it.");
    }

    // xTaskCreatePinnedToCore(heater_task, "heater_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(console_task, "console_task", 8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(ui_task, "ui_task", 8192, NULL, 3, NULL, 1);

    ESP_LOGI(TAG, "Ink printer firmware ready with serial + touch UI");
}
