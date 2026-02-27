#ifndef VEHICLE_MKVII_SOFTWARE_LV_VCU_VCU_H_
#define VEHICLE_MKVII_SOFTWARE_LV_VCU_VCU_H_

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/* All normalized pedal/sensor values in this module are represented as 0..1000. */
#define VCU_PERMILLE_MAX (1000u)

/* High-level operating mode of the VCU torque state machine. */
typedef enum {
    VCU_MODE_INIT = 0,
    VCU_MODE_NOT_READY,
    VCU_MODE_FAULT,
    VCU_MODE_RUN,
} vcu_mode_e;

/*
 * Fault bits stored in vcu_state_s::fault_bits.
 * These values are intended for bitwise operations (|, &, ~), not mutually
 * exclusive enum states.
 */
typedef enum {
    VCU_FAULT_NONE = 0u,
    VCU_FAULT_APPS1_OUT_OF_RANGE = (1u << 0),
    VCU_FAULT_APPS2_OUT_OF_RANGE = (1u << 1),
    VCU_FAULT_APPS_MISMATCH = (1u << 2),
    VCU_FAULT_APPS_TIMEOUT_LATCHED = (1u << 3),
    VCU_FAULT_BRAKE_SENSOR_TIMEOUT = (1u << 4),
    VCU_FAULT_BRAKE_THROTTLE_IMPLAUS = (1u << 5),
    VCU_FAULT_BSPD_POWER_LATCHED = (1u << 6),
    VCU_FAULT_SHUTDOWN_BSPD_OPEN = (1u << 7),
    VCU_FAULT_INERTIA_SWITCH_OPEN = (1u << 8),
    VCU_FAULT_INVERTER_REPORTED = (1u << 9),
    VCU_FAULT_INTERNAL_ADC = (1u << 10),
} vcu_fault_bit_e;

/* CAN inputs consumed by the VCU core each control cycle. */
typedef struct {
    bool ready_to_drive;
    bool inverter_fault_active;
    bool inverter_enable_feedback;
} vcu_can_inputs_s;

/*
 * Optional externally provided sensor snapshot.
 * If read_adc_samples callback returns true, these values are used instead of
 * direct HAL polling from this module.
 */
typedef struct {
    uint16_t apps1_raw;
    uint16_t apps2_raw;
    uint16_t brake_pressure_raw;
    uint16_t brake_pressure_filtered_raw;
    bool motor_current_sense;

    bool apps1_valid;
    bool apps2_valid;
    bool brake_pressure_valid;
    bool brake_pressure_filtered_valid;
    bool motor_current_sense_valid;
} vcu_adc_samples_s;

/*
 * Hardware mapping for this module.
 * HAL handles are provided by board/app initialization and passed into vcu_init().
 */
typedef struct {
    ADC_HandleTypeDef* hadc_apps1;
    ADC_HandleTypeDef* hadc_apps2;
    ADC_HandleTypeDef* hadc_brake_pressure;
    ADC_HandleTypeDef* hadc_brake_pressure_filtered;
    IWDG_HandleTypeDef* hiwdg;

    GPIO_TypeDef* motor_current_sense_port;
    uint16_t motor_current_sense_pin;
    GPIO_PinState motor_current_sense_active_state;

    GPIO_TypeDef* brake_gate_port;
    uint16_t brake_gate_pin;
    GPIO_PinState brake_gate_active_state;

    GPIO_TypeDef* bspd_5kw_port;
    uint16_t bspd_5kw_pin;
    GPIO_PinState bspd_5kw_active_state;

    GPIO_TypeDef* ss_bspd_port;
    uint16_t ss_bspd_pin;
    GPIO_PinState ss_bspd_closed_state;

    GPIO_TypeDef* ss_inertia_port;
    uint16_t ss_inertia_pin;
    GPIO_PinState ss_inertia_closed_state;

    GPIO_TypeDef* inverter_enable_port;
    uint16_t inverter_enable_pin;
    GPIO_PinState inverter_enable_active_state;

    GPIO_TypeDef* fault_led_port;
    uint16_t fault_led_pin;
    GPIO_PinState indicator_led_active_state;

    GPIO_TypeDef* heartbeat_led_port;
    uint16_t heartbeat_led_pin;
    GPIO_PinState heartbeat_led_active_state;
} vcu_hw_s;

/* Calibration constants that define thresholds, timing, and scaling behavior. */
typedef struct {
    uint16_t apps1_min_counts;
    uint16_t apps1_max_counts;
    bool apps1_inverted;

    uint16_t apps2_min_counts;
    uint16_t apps2_max_counts;
    bool apps2_inverted;

    uint16_t brake_pressure_min_counts;
    uint16_t brake_pressure_max_counts;

    uint16_t apps_mismatch_threshold_permille;
    uint16_t apps_implausibility_persist_ms;
    uint16_t brake_sensor_implausibility_persist_ms;

    uint16_t brake_throttle_trigger_permille;
    uint16_t brake_throttle_clear_permille;
    uint16_t hard_brake_threshold_permille;
    uint16_t bspd_trigger_delay_ms;

    int16_t torque_command_max;
    uint16_t throttle_deadband_permille;
    uint16_t throttle_zero_clear_permille;

    uint8_t filter_shift;
    uint16_t heartbeat_toggle_ms;
    uint16_t inverter_command_period_ms;

    bool disable_inverter_on_fault;
} vcu_calib_s;

/*
 * Full runtime state of the VCU core.
 * This struct is both internal state and the payload for debug/status publish hooks.
 */
typedef struct {
    uint16_t apps1_raw;
    uint16_t apps2_raw;
    uint16_t brake_pressure_raw;
    uint16_t brake_pressure_filtered_raw;
    bool motor_current_sense;

    uint16_t apps1_permille;
    uint16_t apps2_permille;
    uint16_t apps_safe_permille;
    uint16_t apps_high_permille;
    uint16_t brake_pressure_permille;

    bool brake_gate;
    bool bspd_5kw;
    bool ss_bspd_closed;
    bool ss_inertia_closed;

    bool apps1_out_of_range;
    bool apps2_out_of_range;
    bool apps_mismatch;
    bool brake_sensor_out_of_range;
    bool apps_implaus_latched;
    bool brake_throttle_implaus_latched;
    bool bspd_latched;
    bool hard_brake_detected;
    bool over_5kw_detected;
    bool adc_read_error;

    bool ready_to_drive;
    bool inverter_fault_active;
    bool inverter_enable_feedback;
    bool inverter_enable_command;

    int16_t torque_command;
    bool heartbeat;
    uint16_t heartbeat_elapsed_ms;

    uint16_t apps_implaus_timer_ms;
    uint16_t brake_sensor_implaus_timer_ms;
    uint16_t bspd_timer_ms;

    uint32_t fault_bits;
    uint32_t blocking_fault_bits;
    vcu_mode_e mode;
    uint16_t inverter_command_publish_elapsed_ms;
} vcu_state_s;

/* Optional integration hooks for platform-specific services and I/O paths. */
typedef void (*vcu_watchdog_refresh_f)(void* user_ctx);
typedef void (*vcu_bootloader_service_f)(void* user_ctx);
typedef void (*vcu_read_can_inputs_f)(void* user_ctx, vcu_can_inputs_s* can_inputs);
typedef bool (*vcu_read_adc_samples_f)(void* user_ctx, vcu_adc_samples_s* samples);
typedef void (*vcu_publish_inverter_command_f)(void* user_ctx,
                                               int16_t torque_command,
                                               bool inverter_enable);
typedef void (*vcu_publish_state_f)(void* user_ctx, const vcu_state_s* state);

typedef struct {
    void* user_ctx;

    vcu_watchdog_refresh_f watchdog_refresh;
    vcu_bootloader_service_f bootloader_service;
    vcu_read_can_inputs_f read_can_inputs;
    vcu_read_adc_samples_f read_adc_samples;

    vcu_publish_inverter_command_f publish_inverter_command;
    vcu_publish_state_f publish_vcu_status;
    vcu_publish_state_f publish_throttle_debug;
    vcu_publish_state_f publish_bspd_debug;
} vcu_hooks_s;

/* Fill a calibration object with conservative default values for bring-up. */
void vcu_load_default_calibration(vcu_calib_s* calib);

/*
 * Initialize the module with hardware mapping, calibrations, and optional hooks.
 * Returns HAL_OK on success and HAL_ERROR when required inputs are invalid.
 */
HAL_StatusTypeDef vcu_init(const vcu_hw_s* hw,
                           const vcu_calib_s* calib,
                           const vcu_hooks_s* hooks);

/* Set scheduler flags consumed by vcu_main_loop_iteration(). */
void vcu_request_1ms_tick(void);
void vcu_request_10ms_tick(void);
/* Push latest CAN-derived control inputs into the module. */
void vcu_set_can_inputs(const vcu_can_inputs_s* can_inputs);
/* Clear latchable faults and associated debounce timers. */
void vcu_clear_latched_faults(void);

/* Execute one control step of the 1 ms / 10 ms tasks. */
HAL_StatusTypeDef vcu_step_1ms(void);
HAL_StatusTypeDef vcu_step_10ms(void);
/* Run one main-loop iteration: background services + scheduled tasks. */
HAL_StatusTypeDef vcu_main_loop_iteration(void);

/* Read-only access to current VCU state snapshot. */
const vcu_state_s* vcu_get_state(void);

#endif /* VEHICLE_MKVII_SOFTWARE_LV_VCU_VCU_H_ */
