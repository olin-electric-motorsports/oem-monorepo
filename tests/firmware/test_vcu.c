#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mock_hal.h"
#define main vcu_firmware_main
#include "vehicle/mkviii/software/lv/VCU/Src/vcu.c"
#undef main

static void setup(void) {
    mock_hal_reset();
    assert(vcu_gpio_init() == HAL_OK);
    assert(vcu_adc_init() == HAL_OK);
    assert(can_init_vcu() == 0);
    assert(vcu_init() == HAL_OK);
}
static void reset_tx(unsigned capacity) {
    mock.tx_capacity = capacity;
    mock.tx_queued = mock.tx_attempts = mock.tx_failures = 0;
    memset(mock.tx_ids, 0, sizeof(mock.tx_ids));
}
int main(int argc, char **argv) {
    assert(argc == 2);
    const char *test = argv[1];
    mock_hal_reset();
    if (!strcmp(test, "adc_init_error")) {
        mock.adc_init_status = HAL_ERROR;
        assert(vcu_adc_init() == HAL_ERROR);
    } else if (!strcmp(test, "adc_calibration_error")) {
        mock.calibration_status = HAL_ERROR;
        assert(vcu_adc_init() == HAL_ERROR);
    } else if (!strcmp(test, "adc_retry_calibration")) {
        mock.calibration_status = HAL_ERROR;
        (void)vcu_adc_init();
        unsigned before = mock.calibration_calls;
        mock.calibration_status = HAL_OK;
        assert(vcu_adc_init() == HAL_OK);
        assert(mock.calibration_calls > before);
    } else if (!strncmp(test, "adc_read_", 9)) {
        setup();
        s_state.ready_to_drive_latched = true;
        s_state.throttle_r_raw = 80;
        mock.failing_channel = 2;
        if (!strcmp(test, "adc_read_config_error")) mock.config_status = HAL_ERROR;
        else if (!strcmp(test, "adc_read_start_error")) mock.start_status = HAL_ERROR;
        else if (!strcmp(test, "adc_read_timeout")) mock.poll_status = HAL_TIMEOUT;
        else if (!strcmp(test, "adc_read_stop_error")) mock.stop_status = HAL_ERROR;
        else return 2;
        vcu_step_1ms();
        assert(s_state.fault_bits != VCU_FAULT_NONE);
        assert(!s_state.inverter_enable_command);
        assert(s_state.inverter_torque_command_raw == 0);
        assert(s_state.throttle_r_raw == 80);
    } else if (!strcmp(test, "can_first_disable")) {
        setup();
        assert(mock.tx_ids[0] == 0xc0);
        assert((mock.tx_data[0][5] & 1u) == 0u);
    } else if (!strcmp(test, "can_three_slot_burst")) {
        setup(); reset_tx(3);
        assert(vcu_step_10ms() != HAL_ERROR);
        assert(mock.tx_attempts == 3 && mock.tx_failures == 0);
        assert(mock.tx_ids[0] == 0xc0);
    } else if (!strcmp(test, "can_deferred_status")) {
        setup(); reset_tx(3); vcu_step_10ms();
        mock.tx_queued = 0; vcu_step_1ms();
        assert(mock.tx_attempts == 4 && mock.tx_failures == 0);
        assert(mock.tx_ids[3] == 0x702);
    } else if (!strcmp(test, "can_backpressure")) {
        setup(); reset_tx(3); mock.tx_queued = 3;
        vcu_step_10ms();
        assert(mock.tx_attempts == 0);
        mock.tx_queued = 0; vcu_step_1ms();
        assert(mock.tx_ids[0] == 0xc0);
    } else if (!strcmp(test, "can_submit_error")) {
        setup(); reset_tx(3);
        s_state.ready_to_drive_latched = true; vcu_step_1ms();
        mock.fail_tx = true;
        assert(vcu_step_10ms() == HAL_ERROR);
        assert(s_state.fault_bits != VCU_FAULT_NONE);
        assert(!s_state.inverter_enable_command);
    } else if (!strcmp(test, "can_recovery_disabled")) {
        setup(); reset_tx(3);
        s_state.ready_to_drive_latched = true; vcu_step_1ms();
        mock.fail_tx = true; vcu_step_10ms();
        mock.fail_tx = false; reset_tx(3); vcu_step_1ms();
        assert(mock.tx_ids[0] == 0xc0);
        assert((mock.tx_data[0][5] & 1u) == 0u);
        assert(!s_state.ready_to_drive_latched);
    } else if (!strcmp(test, "can_pending_uses_safe_state")) {
        setup(); reset_tx(3); mock.tx_queued = 3;
        s_state.ready_to_drive_latched = true;
        mock.adc_counts[1] = 1010; mock.adc_counts[2] = 1700;
        vcu_step_1ms(); vcu_step_10ms();
        mock.failing_channel = 2; mock.poll_status = HAL_TIMEOUT;
        mock.tx_queued = 0; vcu_step_1ms();
        assert(mock.tx_ids[0] == 0xc0);
        assert(mock.tx_data[0][0] == 0 && mock.tx_data[0][1] == 0);
        assert((mock.tx_data[0][5] & 1u) == 0u);
    } else if (!strcmp(test, "can_command_deadline")) {
        setup(); reset_tx(3); mock.tx_queued = 3;
        vcu_step_10ms(); mock.now_ms += 10; vcu_step_1ms();
        assert(s_state.fault_bits != VCU_FAULT_NONE);
    } else if (!strcmp(test, "can_bus_off")) {
        setup(); reset_tx(3); mock.bus_off = true;
        s_state.ready_to_drive_latched = true; vcu_step_1ms();
        assert(s_state.fault_bits != VCU_FAULT_NONE);
        assert(!s_state.inverter_enable_command);
        assert(mock.tx_attempts == 0);
    } else if (!strcmp(test, "can_busy_before_deadline")) {
        setup(); reset_tx(3); mock.tx_queued = 3;
        vcu_step_10ms(); mock.now_ms += 9; vcu_step_1ms();
        assert(s_state.fault_bits == VCU_FAULT_NONE);
        assert(mock.tx_attempts == 0);
    } else if (!strcmp(test, "healthy_drive_and_brake")) {
        setup(); mock.brake = true; s_state.start_button_pressed = true;
        vcu_step_1ms(); assert(s_state.ready_to_drive_latched);
        s_state.start_button_pressed = false; mock.brake = false;
        mock.adc_counts[1] = 1010; mock.adc_counts[2] = 1700;
        vcu_step_1ms();
        assert(s_state.inverter_enable_command && s_state.inverter_torque_command_raw == 127);
        reset_tx(3); vcu_step_10ms();
        assert(mock.tx_ids[0] == 0xc0 && mock.tx_data[0][0] == 127);
        assert((mock.tx_data[0][5] & 1u) != 0u);
        mock.brake = true; vcu_step_1ms();
        assert(s_state.inverter_torque_command_raw == 0);
    } else if (!strcmp(test, "can_ack_error")) {
        setup(); reset_tx(3); mock.can_last_error = FDCAN_PROTOCOL_ERROR_ACK;
        s_state.ready_to_drive_latched = true; vcu_step_1ms();
        assert(!s_state.ready_to_drive_latched);
    } else if (!strcmp(test, "apps_elapsed_time")) {
        setup(); mock.adc_counts[1] = 1000; mock.adc_counts[2] = 0;
        vcu_step_1ms();
        mock.now_ms = 99; vcu_step_1ms();
        assert(!s_state.throttle_implaus_latched);
        mock.now_ms = 100; vcu_step_1ms();
        assert(s_state.throttle_implaus_latched);
        assert(s_state.throttle_implaus_timer_ms == 100);
    } else if (!strcmp(test, "apps_no_elapsed_time")) {
        setup(); mock.adc_counts[1] = 1000; mock.adc_counts[2] = 0;
        for (unsigned i = 0; i < 200; i++) vcu_step_1ms();
        assert(!s_state.throttle_implaus_latched);
        assert(s_state.throttle_implaus_timer_ms == 0);
    } else if (!strcmp(test, "apps_clock_wrap")) {
        setup(); mock.adc_counts[1] = 1000; mock.adc_counts[2] = 0;
        mock.now_ms = UINT32_MAX - 30; vcu_step_1ms();
        mock.now_ms += 100; vcu_step_1ms();
        assert(s_state.throttle_implaus_latched);
    } else if (!strcmp(test, "apps_interrupted_fault")) {
        setup(); mock.adc_counts[1] = 1000; mock.adc_counts[2] = 0;
        mock.now_ms = 10; vcu_step_1ms();
        mock.adc_counts[1] = 36; mock.adc_counts[2] = 7;
        mock.now_ms = 60; vcu_step_1ms();
        mock.adc_counts[1] = 1000; mock.adc_counts[2] = 0;
        mock.now_ms = 110; vcu_step_1ms();
        mock.now_ms = 150; vcu_step_1ms();
        assert(!s_state.throttle_implaus_latched);
        mock.now_ms = 210; vcu_step_1ms();
        assert(s_state.throttle_implaus_latched);
    } else if (!strcmp(test, "heartbeat_delayed_task")) {
        setup(); mock.now_ms = 1250; vcu_step_10ms();
        assert(!s_state.heartbeat);
        assert(s_state.heartbeat_elapsed_ms == 250);
    } else if (!strcmp(test, "heartbeat_clock_wrap")) {
        setup(); mock.now_ms = UINT32_MAX - 200;
        assert(vcu_init() == HAL_OK);
        mock.now_ms += 501; vcu_step_10ms();
        assert(s_state.heartbeat);
        assert(s_state.heartbeat_elapsed_ms == 1);
    } else if (!strcmp(test, "temporary_calibration_unchanged")) {
        setup(); mock.adc_counts[2] = 0; vcu_step_1ms();
        assert(!s_state.throttle_r_out_of_range);
        assert(s_state.fault_bits == VCU_FAULT_NONE);
    } else if (!strcmp(test, "apps_latch_unchanged")) {
        setup(); mock.adc_counts[1] = 1000; mock.adc_counts[2] = 0;
        vcu_step_1ms();
        for (unsigned i = 0; i < 100; i++) {mock.now_ms++; vcu_step_1ms();}
        assert(s_state.throttle_implaus_latched);
        mock.adc_counts[1] = 36; mock.adc_counts[2] = 7;
        for (unsigned i = 0; i < 200; i++) {mock.now_ms++; vcu_step_1ms();}
        assert(s_state.throttle_implaus_latched);
    } else if (!strcmp(test, "startup_input_still_required")) {
        setup(); mock.brake = true;
        for (unsigned i = 0; i < 100; i++) {mock.now_ms++; vcu_step_1ms();}
        assert(s_state.fault_bits == VCU_FAULT_NONE);
        assert(!s_state.ready_to_drive_latched);
        assert(!s_state.inverter_enable_command);
    } else return 2;
    printf("PASS %s\n", test);
    return 0;
}
