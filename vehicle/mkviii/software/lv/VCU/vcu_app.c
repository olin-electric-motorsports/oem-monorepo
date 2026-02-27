#include "vehicle/mkviii/software/lv/VCU/vcu.h"

#include "libs/adc/api.h"
#include "libs/gpio/api.h"
#include "libs/timer/api.h"
#include "vehicle/mkviii/software/lv/VCU/can_api.h"
#include "vehicle/mkviii/software/lv/VCU/vcu_config.h"

#include <avr/interrupt.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "projects/btldr/btldr_lib.h"
#include "projects/btldr/git_sha.h"
#include "projects/btldr/libs/image/api.h"

image_hdr_t image_hdr __attribute__((section(".image_hdr"))) = {
    .image_magic = IMAGE_MAGIC,
    .git_sha = STABLE_GIT_COMMIT,
};

static ADC_HandleTypeDef s_apps1_adc = { .channel = VCU_APPS1_ADC_CHANNEL };
static ADC_HandleTypeDef s_apps2_adc = { .channel = VCU_APPS2_ADC_CHANNEL };
static ADC_HandleTypeDef s_brake_adc = { .channel = VCU_BRAKE_PRESSURE_ADC_CHANNEL };
static ADC_HandleTypeDef s_brake_filtered_adc = { .channel = VCU_BRAKE_PRESSURE_FILTERED_ADC_CHANNEL };

static GPIO_TypeDef s_gpio_port_b = { .port_reg = 0x05, .pin_reg = 0x03 };
static GPIO_TypeDef s_gpio_port_c = { .port_reg = 0x08, .pin_reg = 0x06 };
static GPIO_TypeDef s_gpio_port_d = { .port_reg = 0x0B, .pin_reg = 0x09 };

static inline uint16_t vcu_pin_mask(gpio_t pin) {
    return (uint16_t)(1u << pin.num);
}

static bool vcu_read_adc_samples_hook(void* user_ctx, vcu_adc_samples_s* samples) {
    (void)user_ctx;

    if (samples == NULL) {
        return false;
    }

    samples->apps1_raw = adc_read(VCU_APPS1_ADC_CHANNEL);
    samples->apps2_raw = adc_read(VCU_APPS2_ADC_CHANNEL);
    samples->brake_pressure_raw = adc_read(VCU_BRAKE_PRESSURE_ADC_CHANNEL);
    samples->brake_pressure_filtered_raw = adc_read(VCU_BRAKE_PRESSURE_FILTERED_ADC_CHANNEL);
    samples->motor_current_sense = !!gpio_get_pin(VCU_MOTOR_CURRENT_SENSE);

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

    if (can_poll_receive_dashboard() == 0) {
        can_receive_dashboard();
    }

    can_inputs->ready_to_drive = dashboard.ready_to_drive;
    can_inputs->inverter_fault_active = false;
    can_inputs->inverter_enable_feedback = false;
}

static uint8_t vcu_to_legacy_throttle_status(const vcu_state_s* state) {
    if (state == NULL) {
        return THROTTLE_IDLE;
    }
    if (!state->ready_to_drive) {
        return THROTTLE_IDLE;
    }
    if (state->brake_throttle_implaus_latched) {
        return THROTTLE_BRAKE_PRESSED;
    }
    if (state->apps1_out_of_range) {
        return THROTTLE_L_OUT_OF_RANGE;
    }
    if (state->apps2_out_of_range) {
        return THROTTLE_R_OUT_OF_RANGE;
    }
    if (state->apps_mismatch || state->apps_implaus_latched) {
        return THROTTLE_POSITION_IMPLAUSIBILITY;
    }
    return THROTTLE_RUN;
}

static int16_t vcu_permille_to_legacy_position(uint16_t permille) {
    uint32_t scaled = ((uint32_t)permille * 255u) / VCU_PERMILLE_MAX;
    if (scaled > 255u) {
        scaled = 255u;
    }
    return (int16_t)scaled;
}

static void vcu_publish_inverter_command_hook(void* user_ctx,
                                              int16_t torque_command,
                                              bool inverter_enable) {
    (void)user_ctx;

    m192_command_message.torque_command = torque_command;
    m192_command_message.inverter_enable = inverter_enable;
    can_send_m192_command_message();
}

static void vcu_publish_vcu_status_hook(void* user_ctx, const vcu_state_s* state) {
    (void)user_ctx;

    if (state == NULL) {
        return;
    }

    throttle.throttle_status = vcu_to_legacy_throttle_status(state);
    throttle.throttle_l_pos = vcu_permille_to_legacy_position(state->apps1_permille);
    throttle.throttle_r_pos = vcu_permille_to_legacy_position(state->apps2_permille);
    throttle.ss_is = state->ss_inertia_closed;
    throttle.heartbeat = state->heartbeat;

    bspd.brake_pressure = state->brake_pressure_raw;
    bspd.brake_pressure_filtered = state->brake_pressure_filtered_raw;
    bspd.opamp_timer_rc_circuit_status = 0u;
    bspd.ss_bspd = state->ss_bspd_closed;
    bspd.heartbeat = state->heartbeat;
    bspd.brake_gate = state->brake_gate;
    bspd.bspd_5kw = state->over_5kw_detected;

    can_send_throttle();
    can_send_bspd();
}

static void vcu_publish_throttle_debug_hook(void* user_ctx, const vcu_state_s* state) {
    (void)user_ctx;

    if (state == NULL) {
        return;
    }

    throttle_debug.throttle_l_raw = (int16_t)state->apps1_raw;
    throttle_debug.throttle_r_raw = (int16_t)state->apps2_raw;
    throttle_debug.throttle_l_out_of_range = state->apps1_out_of_range;
    throttle_debug.throttle_r_out_of_range = state->apps2_out_of_range;
    throttle_debug.throttle_deviation = state->apps_mismatch;
    throttle_debug.throttle_brake_implaus = state->brake_throttle_implaus_latched;
    can_send_throttle_debug();
}

static void vcu_publish_bspd_debug_hook(void* user_ctx, const vcu_state_s* state) {
    (void)user_ctx;
    (void)state;
}

void vcu_timer_10ms_callback(void) {
    vcu_request_10ms_tick();
}

void vcu_timer_1ms_callback(void) {
    vcu_request_1ms_tick();
}

int main(void) {
    vcu_hw_s hw = { 0 };
    vcu_calib_s calib = { 0 };
    vcu_hooks_s hooks = { 0 };

    sei();
    can_init_vcu();
    adc_init();
    updater_init(BTLDR_ID, 5);

    timer_init(&vcu_timer_10ms_cfg);
    timer_init(&vcu_timer_1ms_cfg);

    gpio_set_mode(VCU_BRAKE_GATE_SENSE, INPUT);
    gpio_set_mode(VCU_MOTOR_CURRENT_SENSE, INPUT);
    gpio_set_mode(VCU_SS_BSPD_SENSE, INPUT);

    gpio_set_mode(VCU_FAULT_LED, OUTPUT);
    gpio_set_mode(VCU_HEARTBEAT_LED, OUTPUT);

    hw.hadc_apps1 = &s_apps1_adc;
    hw.hadc_apps2 = &s_apps2_adc;
    hw.hadc_brake_pressure = &s_brake_adc;
    hw.hadc_brake_pressure_filtered = &s_brake_filtered_adc;
    hw.hiwdg = NULL;

    hw.motor_current_sense_port = &s_gpio_port_b;
    hw.motor_current_sense_pin = vcu_pin_mask(VCU_MOTOR_CURRENT_SENSE);
    hw.motor_current_sense_active_state = GPIO_PIN_SET;

    hw.brake_gate_port = &s_gpio_port_b;
    hw.brake_gate_pin = vcu_pin_mask(VCU_BRAKE_GATE_SENSE);
    hw.brake_gate_active_state = GPIO_PIN_SET;

    hw.bspd_5kw_port = &s_gpio_port_b;
    hw.bspd_5kw_pin = vcu_pin_mask(VCU_MOTOR_CURRENT_SENSE);
    hw.bspd_5kw_active_state = GPIO_PIN_SET;

    hw.ss_bspd_port = &s_gpio_port_c;
    hw.ss_bspd_pin = vcu_pin_mask(VCU_SS_BSPD_SENSE);
    hw.ss_bspd_closed_state = GPIO_PIN_RESET;

    hw.ss_inertia_port = NULL;
    hw.ss_inertia_pin = 0u;
    hw.ss_inertia_closed_state = GPIO_PIN_SET;

    hw.inverter_enable_port = NULL;
    hw.inverter_enable_pin = 0u;
    hw.inverter_enable_active_state = GPIO_PIN_SET;

    hw.fault_led_port = &s_gpio_port_d;
    hw.fault_led_pin = vcu_pin_mask(VCU_FAULT_LED);
    hw.indicator_led_active_state = GPIO_PIN_SET;

    hw.heartbeat_led_port = &s_gpio_port_d;
    hw.heartbeat_led_pin = vcu_pin_mask(VCU_HEARTBEAT_LED);
    hw.heartbeat_led_active_state = GPIO_PIN_SET;

    vcu_load_default_calibration(&calib);
    calib.apps1_min_counts = VCU_APPS1_MIN_COUNTS;
    calib.apps1_max_counts = VCU_APPS1_MAX_COUNTS;
    calib.apps2_min_counts = VCU_APPS2_MIN_COUNTS;
    calib.apps2_max_counts = VCU_APPS2_MAX_COUNTS;

    hooks.read_can_inputs = vcu_read_can_inputs_hook;
    hooks.read_adc_samples = vcu_read_adc_samples_hook;
    hooks.publish_inverter_command = vcu_publish_inverter_command_hook;
    hooks.publish_vcu_status = vcu_publish_vcu_status_hook;
    hooks.publish_throttle_debug = vcu_publish_throttle_debug_hook;
    hooks.publish_bspd_debug = vcu_publish_bspd_debug_hook;

    if (vcu_init(&hw, &calib, &hooks) != HAL_OK) {
        for (;;) {
        }
    }

    m192_command_message.direction_command = VCU_MOTOR_ANTICLOCKWISE;
    m192_command_message.inverter_enable = false;
    m192_command_message.torque_command = 0;
    can_send_m192_command_message();
    can_receive_dashboard();

    for (;;) {
        updater_loop();
        (void)vcu_main_loop_iteration();
    }
}
