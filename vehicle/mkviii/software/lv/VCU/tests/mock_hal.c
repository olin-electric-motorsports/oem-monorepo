#include "mock_hal.h"
#include <assert.h>
#include <string.h>

GPIO_TypeDef mock_gpio[3] = {{0}, {1}, {2}};
ADC_TypeDef mock_adc[2] = {{0}, {1}};
mock_hal_state_t mock;

void mock_hal_reset(void) {
    memset(&mock, 0, sizeof(mock));
    mock.adc_counts[1] = 36;
    mock.adc_counts[2] = 7;
    mock.failing_channel = -1;
    mock.tx_capacity = 1024;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    if (port == GPIOB && pin == GPIO_PIN_10) return GPIO_PIN_SET;
    if (port == GPIOA && pin == GPIO_PIN_8) return mock.brake ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return GPIO_PIN_RESET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t n, GPIO_PinState s) {(void)p; (void)n; (void)s;}
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *s) {(void)p; (void)s;}
void HAL_IncTick(void) { mock.now_ms++; }
uint32_t HAL_GetTick(void) { return mock.now_ms; }
HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }
void SystemClockConfig(void) {}
void ErrorHandler(void) { assert(!"unexpected firmware ErrorHandler"); }
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h) {
    (void)h; mock.init_calls++; return mock.adc_init_status;
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h, uint32_t s) {
    (void)h; (void)s; mock.calibration_calls++; return mock.calibration_status;
}
static bool failing(ADC_HandleTypeDef *h) {
    return h->Instance == ADC1 && (int)h->selected_channel == mock.failing_channel;
}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c) {
    h->selected_channel = c->Channel;
    return failing(h) ? mock.config_status : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h) { return failing(h) ? mock.start_status : HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t t) {
    if (failing(h) && mock.poll_status != HAL_OK) {
        mock.now_ms += t;
        return mock.poll_status;
    }
    return HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h) { return mock.adc_counts[h->selected_channel]; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h) {
    mock.stop_calls++;
    return failing(h) ? mock.stop_status : HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *h) {(void)h; return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) {(void)h; return HAL_OK;}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t n) {(void)h; (void)n; return 0;}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t n, FDCAN_RxHeaderTypeDef *r, uint8_t *d) {
    (void)h; (void)n; (void)r; (void)d; return HAL_ERROR;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *t, uint8_t *d) {
    (void)h;
    if (mock.tx_attempts < 32) {
        mock.tx_ids[mock.tx_attempts] = t->Identifier;
        memcpy(mock.tx_data[mock.tx_attempts], d, 8);
    }
    mock.tx_attempts++;
    if (mock.fail_tx || mock.tx_queued >= mock.tx_capacity) {
        mock.tx_failures++;
        return HAL_ERROR;
    }
    mock.tx_queued++;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h) {
    (void)h;
    return mock.tx_queued < mock.tx_capacity ? mock.tx_capacity - mock.tx_queued : 0;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h, FDCAN_ProtocolStatusTypeDef *status) {
    (void)h;
    status->BusOff = mock.bus_off;
    status->LastErrorCode = mock.can_last_error;
    mock.can_last_error = FDCAN_PROTOCOL_ERROR_NO_CHANGE;
    return HAL_OK;
}
