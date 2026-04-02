#include "common/adc/adc.h"

static void (*interrupt_callback)(void) = NULL;

/*
 * Internal helper for configuring single channel ADC
 */
static HAL_StatusTypeDef adc_configure_pin(ADC_HandleTypeDef *hadc, adc_pin_e pin) {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = (uint32_t)pin;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5; // adjust as needed
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    return HAL_ADC_ConfigChannel(hadc, &sConfig);
}


/*
 * Runs self-calibration, must be before ADC conversion is enabled
 */
HAL_StatusTypeDef adc_init(ADC_HandleTypeDef *hadc) {
    if (HAL_ADC_Init(hadc) != HAL_OK) {  
        return HAL_ERROR;
    }
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
HAL_StatusTypeDef adc_poll_complete(ADC_HandleTypeDef *hadc, uint32_t *result) {
    HAL_StatusTypeDef poll_status = HAL_ADC_PollForConversion(hadc, 1000);
    if (poll_status == HAL_OK) {
        *result = (uint32_t)HAL_ADC_GetValue(hadc);
    }
    HAL_ADC_Stop(hadc);
    return poll_status;
}

/*
 * Read result (used in interrupt mode)
 */
void adc_read_results(ADC_HandleTypeDef *hadc, uint32_t *result) {
    *result = (uint32_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop_IT(hadc);
}

/*
 * Configures pin in interrupt mode
 */
HAL_StatusTypeDef adc_interrupt_enable(ADC_HandleTypeDef *hadc, adc_pin_e pin, void (*callback)(void)) {
    interrupt_callback = callback;
    if (adc_configure_pin(hadc, pin) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_ADC_Start_IT(hadc);
}

/*
 * HAL callback 
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (interrupt_callback != NULL) {
        interrupt_callback();
    }
}

/*
 * Blocking read
 */
uint32_t adc_read(ADC_HandleTypeDef *hadc, adc_pin_e pin) {
    uint32_t result = 0;

    if (adc_start_convert(hadc, pin) != HAL_OK) {
        return 0;
    }

    if (adc_poll_complete(hadc, &result) != HAL_OK) {
        return 0;
    }
    
    return result;
}