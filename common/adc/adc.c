#include "common/adc/adc.h"

static ADC_HandleTypeDef *adc_handle = NULL;
static void (*interrupt_callback)(void) = NULL;

/*
 * Initialize ADC handle
 */
void adc_init(ADC_HandleTypeDef *hadc) {
    adc_handle = hadc;
}

/*
 * Internal helper for configuring single channel ADC
 */
static HAL_StatusTypeDef adc_configure_pin(adc_pin_e pin) {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = (uint32_t)pin;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; // adjust as needed
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    return HAL_ADC_ConfigChannel(adc_handle, &sConfig);
}


/*
 * Runs self-calibration, must be before ADC is enabled
 *
 */
HAL_StatusTypeDef adc_init(ADC_HandleTypeDef *hadc, adc_pin_e pin) {
    if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}


/*
 * Begins ADC conversion with given pin
 *
 * Currently configures pin too but this could be moved to init
 */
HAL_StatusTypeDef adc_start_convert(ADC_HandleTypeDef *hadc, adc_pin_e pin) {
    if (adc_configure_pin(hadc, pin) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_ADC_Start(hadc);
}



/*
 * Poll for completion, returns poll status
 */
HAL_StatusTypeDef adc_poll_complete(ADC_HandleTypeDef *hadc, uint16_t *result) {
    HAL_StatusTypeDef poll_status = HAL_ADC_PollForConversion(hadc, 100);
    *result = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return poll_status;
}

/*
 * Read result (used in interrupt mode)
 */
void adc_read_results(ADC_HandleTypeDef *hadc, uint16_t *result) {
    *result = (uint16_t)HAL_ADC_GetValue(hadc);
}

/*
 * Configures pin in interrupt mode
 */
void adc_interrupt_enable(ADC_HandleTypeDef *hadc, adc_pin_e pin, void (*callback)(void)) {
    interrupt_callback = callback;
    if (adc_configure_pin(hadc, pin) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_ADC_Start_IT(hadc);
}

/*
 * HAL callback 
 *
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (interrupt_callback != NULL) {
        interrupt_callback();
    }
}

/*
 * Blocking read
 */
uint16_t adc_read(ADC_HandleTypeDef *hadc, adc_pin_e pin) {
    uint16_t result = 0;

    if (adc_start_convert(hadc, pin) != HAL_OK) {
        return 0;
    }

    if (adc_poll_complete(hadc, &result) != HAL_OK) {
        return 0;
    }
    
    HAL_ADC_Stop(hadc);
    
    return result;
}