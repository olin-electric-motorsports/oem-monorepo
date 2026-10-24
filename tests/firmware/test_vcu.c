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
    } else if (!strcmp(test, "temporary_calibration_unchanged")) {
        setup(); mock.adc_counts[2] = 0; vcu_step_1ms();
        assert(!s_state.throttle_r_out_of_range);
        assert(s_state.fault_bits == VCU_FAULT_NONE);
    } else if (!strcmp(test, "apps_latch_unchanged")) {
        setup(); mock.adc_counts[1] = 1000; mock.adc_counts[2] = 0;
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
