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
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

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

// ======== TFT / TOUCH CONFIGURATION (LVGL + esp_lcd) ========
// Leave these as GPIO_NUM_NC until you wire the panel.
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

#define TFT_SPI_HOST        SPI3_HOST
#define TFT_H_RES           240
#define TFT_V_RES           320
#define TFT_PIXEL_CLOCK_HZ  (26 * 1000 * 1000)
#define TFT_DRAW_BUF_PIXELS (TFT_H_RES * 40)
#define TFT_BACKLIGHT_ON    1

// Touch orientation / calibration flags. Adjust these to match your module.
#define TOUCH_SWAP_XY       0
#define TOUCH_MIRROR_X      0
#define TOUCH_MIRROR_Y      1

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

#define UI_CMD_QUEUE_LEN    8

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

typedef enum {
    UI_CMD_MOVE_ABS = 0,
    UI_CMD_HOME,
    UI_CMD_STOP
} ui_cmd_type_t;

typedef struct {
    ui_cmd_type_t type;
    float x;
    float y;
    float z;
    float e;
    float f;
} ui_cmd_t;

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
static QueueHandle_t g_ui_cmd_queue;

static esp_lcd_panel_io_handle_t g_lcd_io;
static esp_lcd_panel_handle_t g_lcd_panel;
static esp_lcd_touch_handle_t g_touch;
static lv_disp_t *g_lv_disp;
static lv_indev_t *g_lv_touch_indev;

static lv_obj_t *g_ui_pos_label;
static lv_obj_t *g_ui_status_label;
static lv_obj_t *g_ui_temp_label;
static lv_obj_t *g_ui_ta_x;
static lv_obj_t *g_ui_ta_y;
static lv_obj_t *g_ui_ta_z;
static lv_obj_t *g_ui_ta_e;
static lv_obj_t *g_ui_active_ta;

// ---------- Utility ----------
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

    gpio_set_level(PIN_BED_SSR, 0);
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
    const int level = a->enabled_low ? (enabled ? 0 : 1) : (enabled ? 1 : 0);
    gpio_set_level(a->en_pin, level);
}

static void set_all_axes_enabled(bool enabled) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
        set_axis_enable((axis_id_t)i, enabled);
    }
}

static void pulse_step(gpio_num_t pin) {
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
            gpio_set_level(PIN_BED_SSR, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (isnan(temp_c) || temp_c < BED_MIN_VALID_C || temp_c > BED_MAX_VALID_C) {
            gpio_set_level(PIN_BED_SSR, 0);
            latch_fault("Bed thermistor invalid or out of range");
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (!heater_enabled || target_c < 0.1f) {
            gpio_set_level(PIN_BED_SSR, 0);
            heat_start_ms = 0;
            start_temp = temp_c;
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        // Starter bang-bang control suitable for a zero-cross SSR; upgrade later if needed.
        if (temp_c < (target_c - BED_BANGBANG_HYST)) {
            gpio_set_level(PIN_BED_SSR, 1);
        } else if (temp_c > (target_c + BED_BANGBANG_HYST)) {
            gpio_set_level(PIN_BED_SSR, 0);
        }

        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (heat_start_ms == 0) {
            heat_start_ms = now_ms;
            start_temp = temp_c;
        }

        if ((now_ms - heat_start_ms) > BED_HEATUP_TIMEOUT_MS && (temp_c - start_temp) < BED_MIN_RISE_C) {
            gpio_set_level(PIN_BED_SSR, 0);
            latch_fault("Bed failed to warm fast enough");
        }

        if (temp_c > BED_MAX_C + 5.0f) {
            gpio_set_level(PIN_BED_SSR, 0);
            latch_fault("Bed over-temperature");
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ---------- Motion ----------
static void set_axis_direction(axis_id_t axis, bool positive) {
    printf("Positive: %d", positive);
    bool level = positive;
    if (g_axes[axis].invert_dir) level = !level;
    gpio_set_level(g_axes[axis].dir_pin, level ? 1 : 0);
}

static esp_err_t move_linear_steps(const int32_t target_steps[AXIS_COUNT], float feed_mm_min) {
    xSemaphoreTakeRecursive(g_motion_mutex, portMAX_DELAY);

    esp_err_t ret = ESP_OK;
    int32_t start[AXIS_COUNT];
    int32_t delta[AXIS_COUNT];
    int32_t abs_delta[AXIS_COUNT];
    int32_t max_steps = 0;
    int32_t err[AXIS_COUNT] = {0};
    int32_t current[AXIS_COUNT];

    if (machine_faulted()) {
        ret = ESP_FAIL;
        goto cleanup;
    }
    if (feed_mm_min < 1.0f) feed_mm_min = DEFAULT_FEED_MM_MIN;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    memcpy(start, g_state.pos_steps, sizeof(start));
    xSemaphoreGive(g_state_mutex);

    for (int i = 0; i < AXIS_COUNT; ++i) {
        delta[i] = target_steps[i] - start[i];
        abs_delta[i] = abs(delta[i]);
        if (abs_delta[i] > max_steps) max_steps = abs_delta[i];
        set_axis_direction((axis_id_t)i, delta[i] >= 0);
    }

    if (max_steps == 0) {
        ret = ESP_OK;
        goto cleanup;
    }

    float dx = steps_to_mm(AXIS_X, delta[AXIS_X]);
    float dy = steps_to_mm(AXIS_Y, delta[AXIS_Y]);
    float dz = steps_to_mm(AXIS_Z, delta[AXIS_Z]);
    float de = steps_to_mm(AXIS_E, delta[AXIS_E]);
    float path_mm = sqrtf(dx*dx + dy*dy + dz*dz + de*de);
    if (path_mm < 0.001f) path_mm = fmaxf(fabsf(dx) + fabsf(dy) + fabsf(dz) + fabsf(de), 0.001f);
    float move_time_s = path_mm / (feed_mm_min / 60.0f);
    uint32_t interval_us = (uint32_t)fmaxf((move_time_s * 1e6f) / (float)max_steps, (float)STEP_MIN_INTERVAL_US);

    memcpy(current, start, sizeof(current));
    set_all_axes_enabled(true);

    for (int32_t i = 0; i < max_steps; ++i) {
        if (machine_faulted()) { ret = ESP_FAIL; goto cleanup; }

        if (is_switch_triggered(g_axes[AXIS_X].min_pin) && delta[AXIS_X] < 0) { latch_fault("Hit X min during move"); ret = ESP_FAIL; goto cleanup; }
        if (is_switch_triggered(g_axes[AXIS_X].max_pin) && delta[AXIS_X] > 0) { latch_fault("Hit X max during move"); ret = ESP_FAIL; goto cleanup; }
        if (is_switch_triggered(g_axes[AXIS_Y].min_pin) && delta[AXIS_Y] < 0) { latch_fault("Hit Y min during move"); ret = ESP_FAIL; goto cleanup; }
        if (is_switch_triggered(g_axes[AXIS_Y].max_pin) && delta[AXIS_Y] > 0) { latch_fault("Hit Y max during move"); ret = ESP_FAIL; goto cleanup; }
        if (is_switch_triggered(g_axes[AXIS_Z].min_pin) && delta[AXIS_Z] < 0) { latch_fault("Hit Z min during move"); ret = ESP_FAIL; goto cleanup; }
        if (is_switch_triggered(g_axes[AXIS_Z].max_pin) && delta[AXIS_Z] > 0) { latch_fault("Hit Z max during move"); ret = ESP_FAIL; goto cleanup; }

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

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    memcpy(g_state.pos_steps, target_steps, sizeof(g_state.pos_steps));
    xSemaphoreGive(g_state_mutex);

cleanup:
    set_all_axes_enabled(false);
    xSemaphoreGiveRecursive(g_motion_mutex);
    return ret;
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
    xSemaphoreTakeRecursive(g_motion_mutex, portMAX_DELAY);
    esp_err_t ret = ESP_OK;
    if ((ret = home_single_axis(AXIS_Z)) != ESP_OK) goto done;
    if ((ret = home_single_axis(AXIS_X)) != ESP_OK) goto done;
    if ((ret = home_single_axis(AXIS_Y)) != ESP_OK) goto done;
done:
    xSemaphoreGiveRecursive(g_motion_mutex);
    return ret;
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
    xSemaphoreTakeRecursive(g_motion_mutex, portMAX_DELAY);
    FILE *fp = fopen(path, "r");
    if (!fp) {
        xSemaphoreGiveRecursive(g_motion_mutex);
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
            xSemaphoreGiveRecursive(g_motion_mutex);
            return err;
        }
    }

    fclose(fp);
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.printing = false;
    xSemaphoreGive(g_state_mutex);
    ESP_LOGI(TAG, "File complete: %s", path);
    xSemaphoreGiveRecursive(g_motion_mutex);
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
    char* line;

    print_help();
    while (1) {
        // printf("> ");
        fflush(stdout);
        // Read line with prompt
        line = linenoise("esp> ");
        if (line == NULL) {
            // Free buffer
            linenoiseFree(line);
            continue; // Empty line
        }
        // Process line
        printf("Received: %s\n", line);

        line[strcspn(line, "\r\n")] = '\0';
        char *cmd = skip_ws(line);
        printf("Received: %s\n", line);

        if (*cmd == '\0') {
            // Free buffer
            linenoiseFree(line);
            continue;
        }

        if (strcasecmp(cmd, "help") == 0) {
            print_help();
            // Free buffer
            linenoiseFree(line);
            continue;
        }
        if (strcasecmp(cmd, "status") == 0) {
            print_status();
            // Free buffer
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
            gpio_set_level(PIN_BED_SSR, 0);
            set_all_axes_enabled(false);
            ESP_LOGW(TAG, "Stop requested");
            // Free buffer
            linenoiseFree(line);
            continue;
        }
        if (strncasecmp(cmd, "run ", 4) == 0) {
            char *path = skip_ws(cmd + 4);
            run_gcode_file(path);
            // Free buffer
            linenoiseFree(line);
            continue;
        }

        execute_gcode_line(cmd);

        // Add to history
        linenoiseHistoryAdd(line);
        // Free buffer
        linenoiseFree(line);
    }
}

// ---------- Touch UI (esp_lcd + XPT2046 + LVGL) ----------
static bool ui_parse_float_from_ta(lv_obj_t *ta, float *out_value) {
    const char *txt = lv_textarea_get_text(ta);
    if (txt == NULL || txt[0] == '\0' || (txt[0] == '-' && txt[1] == '\0')) {
        return false;
    }
    char *end = NULL;
    float v = strtof(txt, &end);
    if (end == txt) return false;
    *out_value = v;
    return true;
}

static void ui_set_active_ta(lv_obj_t *ta) {
    g_ui_active_ta = ta;
    if (g_ui_ta_x) lv_obj_set_style_border_width(g_ui_ta_x, ta == g_ui_ta_x ? 3 : 1, 0);
    if (g_ui_ta_y) lv_obj_set_style_border_width(g_ui_ta_y, ta == g_ui_ta_y ? 3 : 1, 0);
    if (g_ui_ta_z) lv_obj_set_style_border_width(g_ui_ta_z, ta == g_ui_ta_z ? 3 : 1, 0);
    if (g_ui_ta_e) lv_obj_set_style_border_width(g_ui_ta_e, ta == g_ui_ta_e ? 3 : 1, 0);
}

static void ui_textarea_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_FOCUSED || lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ui_set_active_ta(lv_event_get_target(e));
    }
}

static void ui_keypad_event_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *label = lv_obj_get_child(btn, 0);
    const char *txt = lv_label_get_text(label);
    if (!g_ui_active_ta || !txt) return;

    if (strcmp(txt, "CLR") == 0) {
        lv_textarea_set_text(g_ui_active_ta, "");
    } else if (strcmp(txt, "BSP") == 0) {
        lv_textarea_delete_char(g_ui_active_ta);
    } else if (strcmp(txt, "+/-") == 0) {
        char buf[24];
        const char *cur = lv_textarea_get_text(g_ui_active_ta);
        if (cur && cur[0] == '-') {
            strcpy(buf, cur + 1);
            lv_textarea_set_text(g_ui_active_ta, buf);
        } else {
            snprintf(buf, sizeof(buf), "-%s", cur ? cur : "");
            lv_textarea_set_text(g_ui_active_ta, buf);
        }
    } else {
        lv_textarea_add_text(g_ui_active_ta, txt);
    }
}

static void ui_queue_move_abs(void) {
    ui_cmd_t cmd = {
        .type = UI_CMD_MOVE_ABS,
        .f = DEFAULT_FEED_MM_MIN,
    };
    float value;
    if (ui_parse_float_from_ta(g_ui_ta_x, &value)) cmd.x = value; else cmd.x = NAN;
    if (ui_parse_float_from_ta(g_ui_ta_y, &value)) cmd.y = value; else cmd.y = NAN;
    if (ui_parse_float_from_ta(g_ui_ta_z, &value)) cmd.z = value; else cmd.z = NAN;
    if (ui_parse_float_from_ta(g_ui_ta_e, &value)) cmd.e = value; else cmd.e = NAN;
    xQueueSend(g_ui_cmd_queue, &cmd, 0);
}

static void ui_move_btn_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ui_queue_move_abs();
    }
}

static void ui_home_btn_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ui_cmd_t cmd = {.type = UI_CMD_HOME};
        xQueueSend(g_ui_cmd_queue, &cmd, 0);
    }
}

static void ui_stop_btn_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ui_cmd_t cmd = {.type = UI_CMD_STOP};
        xQueueSend(g_ui_cmd_queue, &cmd, 0);
    }
}

static void ui_refresh_timer_cb(lv_timer_t *timer) {
    (void)timer;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    int32_t x_steps = g_state.pos_steps[AXIS_X];
    int32_t y_steps = g_state.pos_steps[AXIS_Y];
    int32_t z_steps = g_state.pos_steps[AXIS_Z];
    int32_t e_steps = g_state.pos_steps[AXIS_E];
    float bed_temp = g_state.bed_temp_c;
    float bed_target = g_state.bed_target_c;
    bool faulted = g_state.faulted;
    bool printing = g_state.printing;
    char fault_msg[sizeof(g_state.fault_msg)];
    snprintf(fault_msg, sizeof(fault_msg), "%s", g_state.fault_msg);
    xSemaphoreGive(g_state_mutex);

    lv_label_set_text_fmt(g_ui_pos_label,
                          "X: %.2f  Y: %.2f  Z: %.2f  E: %.2f",
                          steps_to_mm(AXIS_X, x_steps),
                          steps_to_mm(AXIS_Y, y_steps),
                          steps_to_mm(AXIS_Z, z_steps),
                          steps_to_mm(AXIS_E, e_steps));
    lv_label_set_text_fmt(g_ui_temp_label, "Bed: %.1f / %.1f C", bed_temp, bed_target);
    if (faulted) {
        lv_label_set_text_fmt(g_ui_status_label, "FAULT: %s", fault_msg[0] ? fault_msg : "unknown");
    } else if (printing) {
        lv_label_set_text(g_ui_status_label, "Status: printing");
    } else {
        lv_label_set_text(g_ui_status_label, "Status: ready");
    }
}

static lv_obj_t *ui_create_labeled_ta(lv_obj_t *parent, const char *label_text, int x, int y) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, label_text);
    lv_obj_set_pos(label, x, y);

    lv_obj_t *ta = lv_textarea_create(parent);
    lv_obj_set_size(ta, 90, 25);
    lv_obj_set_style_pad_top(ta, 2, 0);
    lv_obj_set_style_pad_bottom(ta, 2, 0);
    lv_obj_set_pos(ta, x + 15, y - 4);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_max_length(ta, 12);
    lv_obj_add_event_cb(ta, ui_textarea_event_cb, LV_EVENT_ALL, NULL);
    return ta;
}

static void ui_add_keypad_button(lv_obj_t *parent, const char *txt, int col, int row) {
    const int btn_w = 50;
    const int btn_h = 40;
    const int x0 = 10;
    const int y0 = 138;
    const int gap = 6;
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, btn_w, btn_h);
    lv_obj_set_pos(btn, x0 + col * (btn_w + gap), y0 + row * (btn_h + gap));
    lv_obj_add_event_cb(btn, ui_keypad_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, txt);
    lv_obj_center(label);
}

static void ui_build_screen(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_set_style_text_color(scr, lv_color_hex(0xFFFFFF), 0);

    g_ui_pos_label = lv_label_create(scr);
    lv_obj_set_pos(g_ui_pos_label, 10, 8);
    lv_label_set_text(g_ui_pos_label, "X: 0.00  Y: 0.00  Z: 0.00  E: 0.00");

    g_ui_temp_label = lv_label_create(scr);
    lv_obj_set_pos(g_ui_temp_label, 160, 8);
    lv_label_set_text(g_ui_temp_label, "Bed: -- / -- C");

    g_ui_status_label = lv_label_create(scr);
    lv_obj_set_pos(g_ui_status_label, 10, 28);
    lv_label_set_text(g_ui_status_label, "Status: ready");

    g_ui_ta_x = ui_create_labeled_ta(scr, "X", 10, 80);
    g_ui_ta_y = ui_create_labeled_ta(scr, "Y", 118, 80);
    g_ui_ta_z = ui_create_labeled_ta(scr, "Z", 10, 110);
    g_ui_ta_e = ui_create_labeled_ta(scr, "E", 118, 110);

    lv_textarea_set_text(g_ui_ta_x, "0.0");
    lv_textarea_set_text(g_ui_ta_y, "0.0");
    lv_textarea_set_text(g_ui_ta_z, "0.0");
    lv_textarea_set_text(g_ui_ta_e, "0.0");
    ui_set_active_ta(g_ui_ta_x);

    const char *keys[4][4] = {
        {"7", "8", "9", "CLR"},
        {"4", "5", "6", "BSP"},
        {"1", "2", "3", "+/-"},
        {"0", ".", "MOVE", "HOME"},
    };
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            ui_add_keypad_button(scr, keys[r][c], c, r);
        }
    }

    // Override MOVE and HOME behavior with dedicated callbacks.
    lv_obj_t *move_btn = lv_obj_get_child(scr, lv_obj_get_child_cnt(scr) - 2);
    lv_obj_remove_event_cb(move_btn, ui_keypad_event_cb);
    lv_obj_add_event_cb(move_btn, ui_move_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *home_btn = lv_obj_get_child(scr, lv_obj_get_child_cnt(scr) - 1);
    lv_obj_remove_event_cb(home_btn, ui_keypad_event_cb);
    lv_obj_add_event_cb(home_btn, ui_home_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *stop_btn = lv_btn_create(scr);
    lv_obj_set_size(stop_btn, 90, 35);
    lv_obj_set_pos(stop_btn, 133, 30);
    lv_obj_set_style_bg_color(stop_btn, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(stop_btn, ui_stop_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *stop_label = lv_label_create(stop_btn);
    lv_label_set_text(stop_label, "STOP");
    lv_obj_center(stop_label);

    lv_timer_create(ui_refresh_timer_cb, 250, NULL);
}

static void ui_motion_task(void *arg) {
    (void)arg;
    ui_cmd_t cmd;
    while (1) {
        if (xQueueReceive(g_ui_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) continue;

        if (cmd.type == UI_CMD_STOP) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_state.stop_requested = true;
            g_state.printing = false;
            g_state.heater_enabled = false;
            g_state.bed_target_c = 0.0f;
            xSemaphoreGive(g_state_mutex);
            gpio_set_level(PIN_BED_SSR, 0);
            set_all_axes_enabled(false);
            continue;
        }

        if (cmd.type == UI_CMD_HOME) {
            home_all_axes();
            continue;
        }

        if (cmd.type == UI_CMD_MOVE_ABS) {
            int32_t target[AXIS_COUNT];
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            memcpy(target, g_state.pos_steps, sizeof(target));
            xSemaphoreGive(g_state_mutex);

            if (!isnan(cmd.x)) target[AXIS_X] = mm_to_steps(AXIS_X, cmd.x);
            if (!isnan(cmd.y)) target[AXIS_Y] = mm_to_steps(AXIS_Y, cmd.y);
            if (!isnan(cmd.z)) target[AXIS_Z] = mm_to_steps(AXIS_Z, cmd.z);
            if (!isnan(cmd.e)) target[AXIS_E] = mm_to_steps(AXIS_E, cmd.e);

            move_linear_steps(target, cmd.f > 0 ? cmd.f : DEFAULT_FEED_MM_MIN);
        }
    }
}

static esp_err_t ui_display_touch_init(void) {
    if (PIN_TFT_CS == GPIO_NUM_NC || PIN_TFT_DC == GPIO_NUM_NC || PIN_TFT_SCLK == GPIO_NUM_NC || PIN_TFT_MOSI == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "TFT pins not configured; skipping touch UI init");
        return ESP_ERR_NOT_FOUND;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_TFT_MOSI,
        .miso_io_num = PIN_TFT_MISO,
        .sclk_io_num = PIN_TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_H_RES * 40 * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(TFT_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "TFT SPI init failed");

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_TFT_DC,
        .cs_gpio_num = PIN_TFT_CS,
        .pclk_hz = TFT_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .flags = {
            .sio_mode = 0,
        },
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TFT_SPI_HOST, &io_config, &g_lcd_io), TAG, "LCD IO init failed");

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_TFT_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9341(g_lcd_io, &panel_config, &g_lcd_panel), TAG, "ILI9341 panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(g_lcd_panel), TAG, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(g_lcd_panel), TAG, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(g_lcd_panel, true), TAG, "panel on failed");

    if (PIN_TFT_BCKL != GPIO_NUM_NC) {
        gpio_config_t bk = {
            .pin_bit_mask = 1ULL << PIN_TFT_BCKL,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&bk));
        gpio_set_level(PIN_TFT_BCKL, TFT_BACKLIGHT_ON);
    }

    if (PIN_TOUCH_CS != GPIO_NUM_NC) {
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(PIN_TOUCH_CS);
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TFT_SPI_HOST, &tp_io_config, &tp_io_handle), TAG, "touch IO init failed");
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = TFT_H_RES,
            .y_max = TFT_V_RES,
            .rst_gpio_num = -1,
            .int_gpio_num = GPIO_NUM_NC,
            .flags = {
                .swap_xy = TOUCH_SWAP_XY,
                .mirror_x = TOUCH_MIRROR_X,
                .mirror_y = TOUCH_MIRROR_Y,
            },
        };
        ESP_RETURN_ON_ERROR(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &g_touch), TAG, "touch init failed");
    }

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port init failed");

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = g_lcd_io,
        .panel_handle = g_lcd_panel,
        .buffer_size = TFT_DRAW_BUF_PIXELS,
        .double_buffer = true,
        .hres = TFT_H_RES,
        .vres = TFT_V_RES,
        .monochrome = false,
        // .mipi_dsi = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,
        },
    };
    g_lv_disp = lvgl_port_add_disp(&disp_cfg);
    if (!g_lv_disp) return ESP_FAIL;

    if (g_touch) {
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = g_lv_disp,
            .handle = g_touch,
        };
        g_lv_touch_indev = lvgl_port_add_touch(&touch_cfg);
        (void)g_lv_touch_indev;
    }

    lvgl_port_lock(0);
    ui_build_screen();
    lvgl_port_unlock();
    return ESP_OK;
}

// ---------- Init ----------
static void gpio_init_all(void) {
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_X_STEP) | (1ULL << PIN_X_DIR) | (1ULL << PIN_X_EN), //|
        //                 (1ULL << PIN_Y_STEP) | (1ULL << PIN_Y_DIR) | (1ULL << PIN_Y_EN) |
        //                 (1ULL << PIN_Z_STEP) | (1ULL << PIN_Z_DIR) | (1ULL << PIN_Z_EN) |
        //                 (1ULL << PIN_E_STEP) | (1ULL << PIN_E_DIR) | (1ULL << PIN_E_EN) |
        //                 (1ULL << PIN_BED_SSR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    gpio_config_t in_cfg = {
        .pin_bit_mask = 0ULL,
        // .pin_bit_mask = (1ULL << PIN_X_MIN) | (1ULL << PIN_X_MAX) |
        //                 (1ULL << PIN_Y_MIN) | (1ULL << PIN_Y_MAX) |
        //                 (1ULL << PIN_Z_MIN) | (1ULL << PIN_Z_MAX) |
        //                 (1ULL << PIN_Z_PROBE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // ESP_ERROR_CHECK(gpio_config(&in_cfg));

    for (int i = 0; i < AXIS_COUNT; ++i) {
        set_axis_enable((axis_id_t)i, false);
        gpio_set_level(g_axes[i].step_pin, 0);
        gpio_set_level(g_axes[i].dir_pin, 0);
    }
    gpio_set_level(PIN_BED_SSR, 0);
}

#define BLINK_GPIO GPIO_NUM_48

void app_main(void) {

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_console_new_repl_uart(&uart_config, &repl_config, &repl);

    g_state_mutex = xSemaphoreCreateMutex();
    g_motion_mutex = xSemaphoreCreateRecursiveMutex();
    g_ui_cmd_queue = xQueueCreate(UI_CMD_QUEUE_LEN, sizeof(ui_cmd_t));
    memset(&g_state, 0, sizeof(g_state));
    g_state.absolute_mode = true;

    gpio_init_all();
    ESP_ERROR_CHECK(thermistor_adc_init());

    if (sdcard_init() != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without SD card. You can still jog/home over serial.");
    }

    if (ui_display_touch_init() != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without touchscreen UI.");
    } else {
        xTaskCreatePinnedToCore(ui_motion_task, "ui_motion_task", 4096, NULL, 4, NULL, 1);
    }

    // xTaskCreatePinnedToCore(heater_task, "heater_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(console_task, "console_task", 8192, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Ink printer milestone firmware ready");
}
