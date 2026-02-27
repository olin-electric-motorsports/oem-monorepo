#include "vehicle/mkviii/software/lv/VCU/vcu.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define VCU_DEFAULT_ADC_MAX_COUNTS (4095u)
#define VCU_DEFAULT_APPS_MISMATCH_THRESHOLD_PERMILLE (100u)
#define VCU_DEFAULT_APPS_IMPLAUS_PERSIST_MS (100u)
#define VCU_DEFAULT_BRAKE_SENSOR_IMPLAUS_PERSIST_MS (100u)
#define VCU_DEFAULT_BRAKE_THROTTLE_TRIGGER_PERMILLE (250u)
#define VCU_DEFAULT_BRAKE_THROTTLE_CLEAR_PERMILLE (50u)
#define VCU_DEFAULT_HARD_BRAKE_THRESHOLD_PERMILLE (300u)
#define VCU_DEFAULT_BSPD_TRIGGER_DELAY_MS (500u)
#define VCU_DEFAULT_TORQUE_COMMAND_MAX (2540)
#define VCU_DEFAULT_THROTTLE_DEADBAND_PERMILLE (20u)
#define VCU_DEFAULT_THROTTLE_ZERO_CLEAR_PERMILLE (50u)
#define VCU_DEFAULT_FILTER_SHIFT (2u)
#define VCU_DEFAULT_HEARTBEAT_TOGGLE_MS (500u)
#define VCU_DEFAULT_INVERTER_COMMAND_PERIOD_MS (10u)

/* Subset of faults that immediately block torque output. */
#define VCU_BLOCKING_FAULT_MASK                                               \
    (VCU_FAULT_APPS_TIMEOUT_LATCHED | VCU_FAULT_BRAKE_SENSOR_TIMEOUT         \
     | VCU_FAULT_BRAKE_THROTTLE_IMPLAUS | VCU_FAULT_BSPD_POWER_LATCHED       \
     | VCU_FAULT_SHUTDOWN_BSPD_OPEN | VCU_FAULT_INERTIA_SWITCH_OPEN          \
     | VCU_FAULT_INVERTER_REPORTED | VCU_FAULT_INTERNAL_ADC)

static vcu_hw_s s_hw = { 0 };
static vcu_calib_s s_calib = { 0 };
static vcu_hooks_s s_hooks = { 0 };
static vcu_can_inputs_s s_can_inputs = { 0 };
static vcu_state_s s_state = { 0 };
static bool s_hw_initialized = false;

static volatile bool s_tick_1ms = false;
static volatile bool s_tick_10ms = false;

/* A GPIO is considered usable only if both port and pin are configured. */
static inline bool vcu_gpio_is_configured(GPIO_TypeDef* port, uint16_t pin) {
    return (port != NULL) && (pin != 0u);
}

static inline uint16_t vcu_min_u16(uint16_t a, uint16_t b) {
    return (a < b) ? a : b;
}

static inline uint16_t vcu_abs_diff_u16(uint16_t a, uint16_t b) {
    return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static inline bool vcu_bool_from_pin(GPIO_TypeDef* port,
                                     uint16_t pin,
                                     GPIO_PinState active_state,
                                     bool default_if_unconfigured) {
    if (!vcu_gpio_is_configured(port, pin)) {
        return default_if_unconfigured;
    }

    return HAL_GPIO_ReadPin(port, pin) == active_state;
}

/*
 * Write a logical state to a GPIO with active-high/active-low support.
 * "active" is the logical command; active_state defines the physical level.
 */
static inline void vcu_write_pin(GPIO_TypeDef* port,
                                 uint16_t pin,
                                 GPIO_PinState active_state,
                                 bool active) {
    GPIO_PinState pin_state = GPIO_PIN_RESET;
    if (!vcu_gpio_is_configured(port, pin)) {
        return;
    }

    pin_state = active ? active_state
                       : (active_state == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(port, pin, pin_state);
}

static inline uint16_t vcu_increment_sat_u16(uint16_t value) {
    return (value == UINT16_MAX) ? UINT16_MAX : (uint16_t)(value + 1u);
}

static inline bool vcu_hw_ready(void) {
    return s_hw_initialized;
}

/*
 * Blocking single-shot ADC read helper.
 * Sequence: Start -> Poll -> GetValue -> Stop.
 */
static HAL_StatusTypeDef vcu_read_adc_once(ADC_HandleTypeDef* hadc, uint16_t* out_counts) {
    if ((hadc == NULL) || (out_counts == NULL)) {
        return HAL_ERROR;
    }

    if (HAL_ADC_Start(hadc) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_ADC_PollForConversion(hadc, 1u) != HAL_OK) {
        (void)HAL_ADC_Stop(hadc);
        return HAL_ERROR;
    }

    *out_counts = (uint16_t)HAL_ADC_GetValue(hadc);

    if (HAL_ADC_Stop(hadc) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/*
 * Map raw ADC counts into 0..VCU_PERMILLE_MAX with clamp and optional inversion.
 * out_of_range indicates whether raw input was outside [raw_min, raw_max].
 */
static uint16_t vcu_scale_counts_to_permille(uint16_t raw,
                                             uint16_t raw_min,
                                             uint16_t raw_max,
                                             bool inverted,
                                             bool* out_of_range) {
    uint16_t clamped = raw;
    uint32_t scaled = 0u;
    bool local_out_of_range = true;

    if (raw_max > raw_min) {
        local_out_of_range = (raw < raw_min) || (raw > raw_max);

        if (clamped < raw_min) {
            clamped = raw_min;
        } else if (clamped > raw_max) {
            clamped = raw_max;
        }

        scaled = ((uint32_t)(clamped - raw_min) * VCU_PERMILLE_MAX)
                 / (uint32_t)(raw_max - raw_min);
    } else {
        local_out_of_range = true;
        scaled = 0u;
    }

    if (inverted) {
        scaled = VCU_PERMILLE_MAX - scaled;
    }

    if (out_of_range != NULL) {
        *out_of_range = local_out_of_range;
    }

    return (uint16_t)scaled;
}

/* First-order IIR filter implemented with shift arithmetic for deterministic cost. */
static uint16_t vcu_iir_filter_permille(uint16_t previous, uint16_t sample, uint8_t shift) {
    int32_t filtered = previous;

    if (shift == 0u) {
        return sample;
    }

    filtered += ((int32_t)sample - filtered) >> shift;

    if (filtered < 0) {
        return 0u;
    }
    if (filtered > (int32_t)VCU_PERMILLE_MAX) {
        return VCU_PERMILLE_MAX;
    }
    return (uint16_t)filtered;
}

/*
 * Read one ADC signal into target. For required channels, read failure raises
 * internal ADC fault state.
 */
static void vcu_read_adc_signal(ADC_HandleTypeDef* hadc, uint16_t* target, bool required) {
    uint16_t sample = 0u;

    if (hadc == NULL) {
        if (required) {
            s_state.adc_read_error = true;
        }
        return;
    }

    if (vcu_read_adc_once(hadc, &sample) != HAL_OK) {
        s_state.adc_read_error = true;
        return;
    }

    *target = sample;
}

/*
 * Runs services that should execute every main-loop iteration regardless of
 * scheduler ticks (watchdog + optional app hooks).
 */
static void vcu_background_service(void) {
    if (vcu_hw_ready() && (s_hw.hiwdg != NULL)) {
        (void)HAL_IWDG_Refresh(s_hw.hiwdg);
    }

    if (s_hooks.watchdog_refresh != NULL) {
        s_hooks.watchdog_refresh(s_hooks.user_ctx);
    }
    if (s_hooks.bootloader_service != NULL) {
        s_hooks.bootloader_service(s_hooks.user_ctx);
    }
}

/* Apply VCU command outputs to hardware pins. */
static void vcu_apply_outputs(void) {
    bool has_fault = s_state.fault_bits != VCU_FAULT_NONE;

    if (!vcu_hw_ready()) {
        return;
    }

    vcu_write_pin(s_hw.inverter_enable_port,
                  s_hw.inverter_enable_pin,
                  s_hw.inverter_enable_active_state,
                  s_state.inverter_enable_command);

    vcu_write_pin(s_hw.fault_led_port,
                  s_hw.fault_led_pin,
                  s_hw.indicator_led_active_state,
                  has_fault);

    vcu_write_pin(s_hw.heartbeat_led_port,
                  s_hw.heartbeat_led_pin,
                  s_hw.heartbeat_led_active_state,
                  s_state.heartbeat);
}

/*
 * Read all 1 ms inputs:
 * - Prefer external sampled inputs when callback is provided.
 * - Fall back to direct HAL ADC polling for analog signals.
 * - Read digital safety signals directly from GPIO.
 */
static void vcu_read_inputs_1ms(void) {
    vcu_adc_samples_s samples = { 0 };
    bool used_external_samples = false;

    s_state.adc_read_error = false;

    if (s_hooks.read_adc_samples != NULL) {
        used_external_samples = s_hooks.read_adc_samples(s_hooks.user_ctx, &samples);
    }

    if (used_external_samples) {
        /* APPS channels are mandatory even when external sampling is used. */
        if (samples.apps1_valid) {
            s_state.apps1_raw = samples.apps1_raw;
        } else {
            s_state.adc_read_error = true;
        }

        if (samples.apps2_valid) {
            s_state.apps2_raw = samples.apps2_raw;
        } else {
            s_state.adc_read_error = true;
        }

        if (samples.brake_pressure_valid) {
            s_state.brake_pressure_raw = samples.brake_pressure_raw;
        }
        if (samples.brake_pressure_filtered_valid) {
            s_state.brake_pressure_filtered_raw = samples.brake_pressure_filtered_raw;
        }
        if (samples.motor_current_sense_valid) {
            s_state.motor_current_sense = samples.motor_current_sense;
        } else {
            /* Allow mixed mode: external analog samples + GPIO digital current sense. */
            s_state.motor_current_sense = vcu_bool_from_pin(s_hw.motor_current_sense_port,
                                                            s_hw.motor_current_sense_pin,
                                                            s_hw.motor_current_sense_active_state,
                                                            false);
        }
    } else {
        /* Fallback path for bring-up when DMA/ISR sampling is not wired yet. */
        vcu_read_adc_signal(s_hw.hadc_apps1, &s_state.apps1_raw, true);
        vcu_read_adc_signal(s_hw.hadc_apps2, &s_state.apps2_raw, true);
        vcu_read_adc_signal(s_hw.hadc_brake_pressure, &s_state.brake_pressure_raw, false);
        vcu_read_adc_signal(s_hw.hadc_brake_pressure_filtered,
                            &s_state.brake_pressure_filtered_raw,
                            false);

        s_state.motor_current_sense = vcu_bool_from_pin(s_hw.motor_current_sense_port,
                                                        s_hw.motor_current_sense_pin,
                                                        s_hw.motor_current_sense_active_state,
                                                        false);
    }

    if (used_external_samples) {
        /* If filtered brake is not provided, reuse unfiltered brake sample. */
        if (!samples.brake_pressure_filtered_valid) {
            s_state.brake_pressure_filtered_raw = s_state.brake_pressure_raw;
        }
    } else if (s_hw.hadc_brake_pressure_filtered == NULL) {
        /* Same fallback when no dedicated filtered ADC channel exists. */
        s_state.brake_pressure_filtered_raw = s_state.brake_pressure_raw;
    }

    s_state.brake_gate = vcu_bool_from_pin(s_hw.brake_gate_port,
                                           s_hw.brake_gate_pin,
                                           s_hw.brake_gate_active_state,
                                           false);
    s_state.bspd_5kw = vcu_bool_from_pin(s_hw.bspd_5kw_port,
                                         s_hw.bspd_5kw_pin,
                                         s_hw.bspd_5kw_active_state,
                                         false);
    s_state.ss_bspd_closed = vcu_bool_from_pin(s_hw.ss_bspd_port,
                                               s_hw.ss_bspd_pin,
                                               s_hw.ss_bspd_closed_state,
                                               true);
    s_state.ss_inertia_closed = vcu_bool_from_pin(s_hw.ss_inertia_port,
                                                  s_hw.ss_inertia_pin,
                                                  s_hw.ss_inertia_closed_state,
                                                  true);
}

/*
 * Convert raw inputs to normalized engineering units and derive helper states
 * used by fault and torque logic.
 */
static void vcu_normalize_and_filter_1ms(void) {
    uint16_t apps1_norm = 0u;
    uint16_t apps2_norm = 0u;
    uint16_t brake_pressure_norm = 0u;

    apps1_norm = vcu_scale_counts_to_permille(s_state.apps1_raw,
                                              s_calib.apps1_min_counts,
                                              s_calib.apps1_max_counts,
                                              s_calib.apps1_inverted,
                                              &s_state.apps1_out_of_range);

    apps2_norm = vcu_scale_counts_to_permille(s_state.apps2_raw,
                                              s_calib.apps2_min_counts,
                                              s_calib.apps2_max_counts,
                                              s_calib.apps2_inverted,
                                              &s_state.apps2_out_of_range);

    brake_pressure_norm = vcu_scale_counts_to_permille(s_state.brake_pressure_filtered_raw,
                                                       s_calib.brake_pressure_min_counts,
                                                       s_calib.brake_pressure_max_counts,
                                                       false,
                                                       &s_state.brake_sensor_out_of_range);

    s_state.apps1_permille
        = vcu_iir_filter_permille(s_state.apps1_permille, apps1_norm, s_calib.filter_shift);
    s_state.apps2_permille
        = vcu_iir_filter_permille(s_state.apps2_permille, apps2_norm, s_calib.filter_shift);
    s_state.brake_pressure_permille = vcu_iir_filter_permille(s_state.brake_pressure_permille,
                                                              brake_pressure_norm,
                                                              s_calib.filter_shift);

    s_state.apps_safe_permille = vcu_min_u16(s_state.apps1_permille, s_state.apps2_permille);
    s_state.apps_high_permille = (s_state.apps1_permille > s_state.apps2_permille)
                                     ? s_state.apps1_permille
                                     : s_state.apps2_permille;
    s_state.apps_mismatch = vcu_abs_diff_u16(s_state.apps1_permille, s_state.apps2_permille)
                            > s_calib.apps_mismatch_threshold_permille;

    s_state.hard_brake_detected = s_state.brake_gate
                                  || (s_state.brake_pressure_permille
                                      >= s_calib.hard_brake_threshold_permille);
    /* over_5kw_detected is a digital OR of both 5kW indicators. */
    s_state.over_5kw_detected = s_state.bspd_5kw || s_state.motor_current_sense;
}

/*
 * Build fault bitmask and latching state.
 * Debounce and latch behavior is implemented with ms counters that increment
 * in the 1 ms task.
 */
static void vcu_update_fault_manager_1ms(void) {
    uint32_t fault_bits = VCU_FAULT_NONE;
    bool apps_implausible_now = false;

    apps_implausible_now = s_state.apps1_out_of_range || s_state.apps2_out_of_range
                           || s_state.apps_mismatch;

    if (s_state.apps1_out_of_range) {
        fault_bits |= VCU_FAULT_APPS1_OUT_OF_RANGE;
    }
    if (s_state.apps2_out_of_range) {
        fault_bits |= VCU_FAULT_APPS2_OUT_OF_RANGE;
    }
    if (s_state.apps_mismatch) {
        fault_bits |= VCU_FAULT_APPS_MISMATCH;
    }

    if (apps_implausible_now) {
        s_state.apps_implaus_timer_ms = vcu_increment_sat_u16(s_state.apps_implaus_timer_ms);
        if (s_state.apps_implaus_timer_ms >= s_calib.apps_implausibility_persist_ms) {
            s_state.apps_implaus_latched = true;
        }
    } else {
        s_state.apps_implaus_timer_ms = 0u;
        if (s_state.apps_safe_permille <= s_calib.throttle_zero_clear_permille) {
            s_state.apps_implaus_latched = false;
        }
    }

    if (s_state.apps_implaus_latched) {
        fault_bits |= VCU_FAULT_APPS_TIMEOUT_LATCHED;
    }

    if (s_state.brake_sensor_out_of_range) {
        s_state.brake_sensor_implaus_timer_ms
            = vcu_increment_sat_u16(s_state.brake_sensor_implaus_timer_ms);
    } else {
        s_state.brake_sensor_implaus_timer_ms = 0u;
    }
    if (s_state.brake_sensor_implaus_timer_ms >= s_calib.brake_sensor_implausibility_persist_ms) {
        fault_bits |= VCU_FAULT_BRAKE_SENSOR_TIMEOUT;
    }

    if (s_state.brake_gate
        && (s_state.apps_high_permille >= s_calib.brake_throttle_trigger_permille)) {
        s_state.brake_throttle_implaus_latched = true;
    } else if (s_state.apps_high_permille <= s_calib.brake_throttle_clear_permille) {
        s_state.brake_throttle_implaus_latched = false;
    }
    if (s_state.brake_throttle_implaus_latched) {
        fault_bits |= VCU_FAULT_BRAKE_THROTTLE_IMPLAUS;
    }

    if (s_state.hard_brake_detected && s_state.over_5kw_detected) {
        s_state.bspd_timer_ms = vcu_increment_sat_u16(s_state.bspd_timer_ms);
        if (s_state.bspd_timer_ms >= s_calib.bspd_trigger_delay_ms) {
            s_state.bspd_latched = true;
        }
    } else {
        s_state.bspd_timer_ms = 0u;
    }

    if (!s_state.ss_bspd_closed) {
        fault_bits |= VCU_FAULT_SHUTDOWN_BSPD_OPEN;
        s_state.bspd_latched = true;
    }
    if (!s_state.ss_inertia_closed) {
        fault_bits |= VCU_FAULT_INERTIA_SWITCH_OPEN;
        s_state.bspd_latched = true;
    }
    if (s_state.bspd_latched) {
        fault_bits |= VCU_FAULT_BSPD_POWER_LATCHED;
    }

    if (s_can_inputs.inverter_fault_active) {
        fault_bits |= VCU_FAULT_INVERTER_REPORTED;
    }
    if (s_state.adc_read_error) {
        fault_bits |= VCU_FAULT_INTERNAL_ADC;
    }

    s_state.fault_bits = fault_bits;
    /* Keep only the fault subset that should block torque generation. */
    s_state.blocking_fault_bits = fault_bits & VCU_BLOCKING_FAULT_MASK;
}

/*
 * Generate torque and inverter enable command for the current cycle.
 * Priority is: not ready -> fault -> normal run.
 */
static void vcu_update_torque_command_1ms(void) {
    bool has_fault = s_state.blocking_fault_bits != VCU_FAULT_NONE;
    uint32_t torque = 0u;

    s_state.ready_to_drive = s_can_inputs.ready_to_drive;
    s_state.inverter_fault_active = s_can_inputs.inverter_fault_active;
    s_state.inverter_enable_feedback = s_can_inputs.inverter_enable_feedback;

    if (!s_state.ready_to_drive) {
        s_state.torque_command = 0;
        s_state.inverter_enable_command = false;
        s_state.mode = VCU_MODE_NOT_READY;
        return;
    }

    if (has_fault) {
        s_state.torque_command = 0;
        s_state.inverter_enable_command = !s_calib.disable_inverter_on_fault;
        s_state.mode = VCU_MODE_FAULT;
        return;
    }

    torque = ((uint32_t)s_state.apps_safe_permille * (uint32_t)s_calib.torque_command_max)
             / VCU_PERMILLE_MAX;

    if (s_state.apps_safe_permille <= s_calib.throttle_deadband_permille) {
        torque = 0u;
    }
    if (torque > (uint32_t)s_calib.torque_command_max) {
        torque = (uint32_t)s_calib.torque_command_max;
    }

    s_state.torque_command = (int16_t)torque;
    s_state.inverter_enable_command = true;
    s_state.mode = VCU_MODE_RUN;
}

/* Populate a default calibration for initial bench bring-up. */
void vcu_load_default_calibration(vcu_calib_s* calib) {
    if (calib == NULL) {
        return;
    }

    memset(calib, 0, sizeof(*calib));

    /*
     * TODO: Replace these defaults with on-car calibration data once VCU bring-up
     * is complete.
     */
    calib->apps1_min_counts = 0u;
    calib->apps1_max_counts = VCU_DEFAULT_ADC_MAX_COUNTS;
    calib->apps1_inverted = false;
    calib->apps2_min_counts = 0u;
    calib->apps2_max_counts = VCU_DEFAULT_ADC_MAX_COUNTS;
    calib->apps2_inverted = false;
    calib->brake_pressure_min_counts = 0u;
    calib->brake_pressure_max_counts = VCU_DEFAULT_ADC_MAX_COUNTS;
    calib->apps_mismatch_threshold_permille = VCU_DEFAULT_APPS_MISMATCH_THRESHOLD_PERMILLE;
    calib->apps_implausibility_persist_ms = VCU_DEFAULT_APPS_IMPLAUS_PERSIST_MS;
    calib->brake_sensor_implausibility_persist_ms
        = VCU_DEFAULT_BRAKE_SENSOR_IMPLAUS_PERSIST_MS;

    calib->brake_throttle_trigger_permille = VCU_DEFAULT_BRAKE_THROTTLE_TRIGGER_PERMILLE;
    calib->brake_throttle_clear_permille = VCU_DEFAULT_BRAKE_THROTTLE_CLEAR_PERMILLE;
    calib->hard_brake_threshold_permille = VCU_DEFAULT_HARD_BRAKE_THRESHOLD_PERMILLE;
    calib->bspd_trigger_delay_ms = VCU_DEFAULT_BSPD_TRIGGER_DELAY_MS;

    calib->torque_command_max = VCU_DEFAULT_TORQUE_COMMAND_MAX;
    calib->throttle_deadband_permille = VCU_DEFAULT_THROTTLE_DEADBAND_PERMILLE;
    calib->throttle_zero_clear_permille = VCU_DEFAULT_THROTTLE_ZERO_CLEAR_PERMILLE;

    calib->filter_shift = VCU_DEFAULT_FILTER_SHIFT;
    calib->heartbeat_toggle_ms = VCU_DEFAULT_HEARTBEAT_TOGGLE_MS;
    calib->inverter_command_period_ms = VCU_DEFAULT_INVERTER_COMMAND_PERIOD_MS;
    calib->disable_inverter_on_fault = true;
}

/*
 * Validate user configuration and initialize all module state.
 * Only APPS ADC handles are strictly required by this implementation.
 */
HAL_StatusTypeDef vcu_init(const vcu_hw_s* hw,
                           const vcu_calib_s* calib,
                           const vcu_hooks_s* hooks) {
    if ((hw == NULL) || (calib == NULL)) {
        return HAL_ERROR;
    }
    if ((hw->hadc_apps1 == NULL) || (hw->hadc_apps2 == NULL)) {
        return HAL_ERROR;
    }
    if ((calib->apps1_max_counts <= calib->apps1_min_counts)
        || (calib->apps2_max_counts <= calib->apps2_min_counts)) {
        return HAL_ERROR;
    }
    if (calib->torque_command_max < 0) {
        return HAL_ERROR;
    }

    s_hw = *hw;
    s_hw_initialized = true;
    s_calib = *calib;
    if (s_calib.inverter_command_period_ms == 0u) {
        s_calib.inverter_command_period_ms = 1u;
    }
    if (hooks != NULL) {
        s_hooks = *hooks;
    } else {
        memset(&s_hooks, 0, sizeof(s_hooks));
    }
    memset(&s_state, 0, sizeof(s_state));
    memset(&s_can_inputs, 0, sizeof(s_can_inputs));

    s_tick_1ms = false;
    s_tick_10ms = false;

    s_state.mode = VCU_MODE_INIT;
    s_state.ss_bspd_closed = true;
    s_state.ss_inertia_closed = true;
    s_state.heartbeat = false;
    s_state.inverter_enable_command = false;
    s_state.torque_command = 0;
    s_state.inverter_command_publish_elapsed_ms = s_calib.inverter_command_period_ms;
    s_state.blocking_fault_bits = 0u;

    vcu_apply_outputs();
    return HAL_OK;
}

/* Scheduler hook: request execution of 1 ms task from the main loop. */
void vcu_request_1ms_tick(void) {
    s_tick_1ms = true;
}

/* Scheduler hook: request execution of 10 ms task from the main loop. */
void vcu_request_10ms_tick(void) {
    s_tick_10ms = true;
}

/* Update latest CAN input snapshot. */
void vcu_set_can_inputs(const vcu_can_inputs_s* can_inputs) {
    if (can_inputs == NULL) {
        return;
    }
    s_can_inputs = *can_inputs;
}

/* Clear latchable safety faults and reset associated persistence timers. */
void vcu_clear_latched_faults(void) {
    s_state.apps_implaus_latched = false;
    s_state.brake_throttle_implaus_latched = false;
    s_state.bspd_latched = false;

    s_state.apps_implaus_timer_ms = 0u;
    s_state.brake_sensor_implaus_timer_ms = 0u;
    s_state.bspd_timer_ms = 0u;
    s_state.blocking_fault_bits = 0u;
}

/* Run one 1 ms control cycle. */
HAL_StatusTypeDef vcu_step_1ms(void) {
    if (!vcu_hw_ready()) {
        return HAL_ERROR;
    }

    vcu_read_inputs_1ms();
    vcu_normalize_and_filter_1ms();
    vcu_update_fault_manager_1ms();
    vcu_update_torque_command_1ms();
    vcu_apply_outputs();

    /* Publish inverter command at configured cadence (default 10 ms). */
    s_state.inverter_command_publish_elapsed_ms
        = (uint16_t)(s_state.inverter_command_publish_elapsed_ms + 1u);
    if (s_state.inverter_command_publish_elapsed_ms >= s_calib.inverter_command_period_ms) {
        s_state.inverter_command_publish_elapsed_ms = 0u;
        if (s_hooks.publish_inverter_command != NULL) {
            s_hooks.publish_inverter_command(s_hooks.user_ctx,
                                             s_state.torque_command,
                                             s_state.inverter_enable_command);
        }
    }

    return s_state.adc_read_error ? HAL_ERROR : HAL_OK;
}

/* Run one 10 ms telemetry/heartbeat cycle. */
HAL_StatusTypeDef vcu_step_10ms(void) {
    if (!vcu_hw_ready()) {
        return HAL_ERROR;
    }

    if (s_calib.heartbeat_toggle_ms > 0u) {
        s_state.heartbeat_elapsed_ms = (uint16_t)(s_state.heartbeat_elapsed_ms + 10u);
        if (s_state.heartbeat_elapsed_ms >= s_calib.heartbeat_toggle_ms) {
            s_state.heartbeat = !s_state.heartbeat;
            s_state.heartbeat_elapsed_ms = 0u;
        }
    } else {
        s_state.heartbeat = false;
        s_state.heartbeat_elapsed_ms = 0u;
    }

    if (s_hooks.publish_vcu_status != NULL) {
        s_hooks.publish_vcu_status(s_hooks.user_ctx, &s_state);
    }
    if (s_hooks.publish_throttle_debug != NULL) {
        s_hooks.publish_throttle_debug(s_hooks.user_ctx, &s_state);
    }
    if (s_hooks.publish_bspd_debug != NULL) {
        s_hooks.publish_bspd_debug(s_hooks.user_ctx, &s_state);
    }

    vcu_apply_outputs();
    return HAL_OK;
}

/*
 * Main cooperative scheduler entry point.
 * Services background hooks and executes pending periodic tasks.
 */
HAL_StatusTypeDef vcu_main_loop_iteration(void) {
    HAL_StatusTypeDef status = HAL_OK;
    vcu_can_inputs_s callback_inputs = { 0 };

    if (!vcu_hw_ready()) {
        return HAL_ERROR;
    }

    vcu_background_service();

    if (s_hooks.read_can_inputs != NULL) {
        callback_inputs = s_can_inputs;
        s_hooks.read_can_inputs(s_hooks.user_ctx, &callback_inputs);
        s_can_inputs = callback_inputs;
    }

    if (s_tick_1ms) {
        s_tick_1ms = false;
        if (vcu_step_1ms() != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    if (s_tick_10ms) {
        s_tick_10ms = false;
        if (vcu_step_10ms() != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    return status;
}

/* Expose read-only state pointer for diagnostics/telemetry. */
const vcu_state_s* vcu_get_state(void) {
    return &s_state;
}
