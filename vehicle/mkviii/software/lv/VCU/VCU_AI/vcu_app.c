#include "vehicle/mkviii/software/lv/VCU/vcu.h"
#include "vehicle/mkviii/software/lv/VCU/vcu_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static ADC_HandleTypeDef s_apps1_adc = { .channel = VCU_APPS1_ADC_CHANNEL };
static ADC_HandleTypeDef s_apps2_adc = { .channel = VCU_APPS2_ADC_CHANNEL };
static ADC_HandleTypeDef s_brake_adc = { .channel = VCU_BRAKE_PRESSURE_ADC_CHANNEL };
static ADC_HandleTypeDef s_brake_filtered_adc = { .channel = VCU_BRAKE_PRESSURE_FILTERED_ADC_CHANNEL };

static GPIO_TypeDef s_motor_current_port = { 0 };
static GPIO_TypeDef s_brake_gate_port = { 0 };
static GPIO_TypeDef s_bspd_port = { 0 };
static GPIO_TypeDef s_fault_led_port = { 0 };
static GPIO_TypeDef s_heartbeat_led_port = { 0 };

static bool vcu_read_adc_samples_hook(void* user_ctx, vcu_adc_samples_s* samples) {
    (void)user_ctx;

    if (samples == NULL) {
        return false;
    }

    /* Placeholder values until board-specific ADC plumbing is restored. */
    samples->apps1_raw = 0u;
    samples->apps2_raw = 0u;
    samples->brake_pressure_raw = 0u;
    samples->brake_pressure_filtered_raw = 0u;
    samples->motor_current_sense = false;

    samples->apps1_valid = true;
    samples->apps2_valid = true;
    samples->brake_pressure_valid = true;
    samples->brake_pressure_filtered_valid = true;
    samples->motor_current_sense_valid = true;

    return true;
}

static void vcu_read_can_inputs_hook(void* user_ctx, vcu_can_inputs_s* can_inputs) {
    (void)user_ctx;

    if (can_inputs == NULL) {
        return;
    }

    /* Keep vehicle disabled by default until CAN integration is reconnected. */
    can_inputs->ready_to_drive = false;
    can_inputs->inverter_fault_active = false;
    can_inputs->inverter_enable_feedback = false;
}

static void vcu_publish_inverter_command_hook(void* user_ctx,
                                              int16_t torque_command,
                                              bool inverter_enable) {
    (void)user_ctx;
    (void)torque_command;
    (void)inverter_enable;
}

static void vcu_publish_state_hook(void* user_ctx, const vcu_state_s* state) {
    (void)user_ctx;
    (void)state;
}

int main(void) {
    vcu_hw_s hw = { 0 };
    vcu_calib_s calib = { 0 };
    vcu_hooks_s hooks = { 0 };
    uint16_t loop_count = 0u;

    hw.hadc_apps1 = &s_apps1_adc;
    hw.hadc_apps2 = &s_apps2_adc;
    hw.hadc_brake_pressure = &s_brake_adc;
    hw.hadc_brake_pressure_filtered = &s_brake_filtered_adc;

    hw.motor_current_sense_port = &s_motor_current_port;
    hw.motor_current_sense_pin = 0x0001u;
    hw.motor_current_sense_active_state = GPIO_PIN_SET;

    hw.brake_gate_port = &s_brake_gate_port;
    hw.brake_gate_pin = 0x0001u;
    hw.brake_gate_active_state = GPIO_PIN_SET;

    hw.bspd_5kw_port = &s_bspd_port;
    hw.bspd_5kw_pin = 0x0001u;
    hw.bspd_5kw_active_state = GPIO_PIN_SET;

    hw.fault_led_port = &s_fault_led_port;
    hw.fault_led_pin = 0x0001u;
    hw.indicator_led_active_state = GPIO_PIN_SET;

    hw.heartbeat_led_port = &s_heartbeat_led_port;
    hw.heartbeat_led_pin = 0x0001u;
    hw.heartbeat_led_active_state = GPIO_PIN_SET;

    vcu_load_default_calibration(&calib);
    calib.apps1_min_counts = VCU_APPS1_MIN_COUNTS;
    calib.apps1_max_counts = VCU_APPS1_MAX_COUNTS;
    calib.apps2_min_counts = VCU_APPS2_MIN_COUNTS;
    calib.apps2_max_counts = VCU_APPS2_MAX_COUNTS;

    hooks.read_can_inputs = vcu_read_can_inputs_hook;
    hooks.read_adc_samples = vcu_read_adc_samples_hook;
    hooks.publish_inverter_command = vcu_publish_inverter_command_hook;
    hooks.publish_vcu_status = vcu_publish_state_hook;
    hooks.publish_throttle_debug = vcu_publish_state_hook;
    hooks.publish_bspd_debug = vcu_publish_state_hook;

    if (vcu_init(&hw, &calib, &hooks) != HAL_OK) {
        for (;;) {
        }
    }

    for (;;) {
        vcu_request_1ms_tick();

        loop_count++;
        if (loop_count >= 10u) {
            loop_count = 0u;
            vcu_request_10ms_tick();
        }

        (void)vcu_main_loop_iteration();
    }
}
