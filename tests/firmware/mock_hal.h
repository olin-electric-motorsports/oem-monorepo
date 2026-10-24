#pragma once
#include "stm32g4xx_hal.h"

typedef struct {
    uint32_t adc_counts[18];
    uint32_t now_ms;
    bool brake;
    HAL_StatusTypeDef adc_init_status, calibration_status;
    HAL_StatusTypeDef config_status, start_status, poll_status, stop_status;
    int failing_channel;
    unsigned init_calls, calibration_calls, stop_calls;
    bool fail_tx;
    unsigned tx_capacity, tx_queued, tx_attempts, tx_failures;
    uint32_t tx_ids[32];
} mock_hal_state_t;
extern mock_hal_state_t mock;
void mock_hal_reset(void);
