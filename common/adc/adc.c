#include "common/adc/adc.h"


/*
 * Configure an ADC pin 
 */
HAL_StatusTypeDef adc_configure_pin(ADC_HandleTypeDef *hadc, adc_pin_e pin, adc_pin_config_t *config) {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = (uint32_t)pin;
    sConfig.Rank = config->rank;
    sConfig.SamplingTime = config->sampling_time;
    sConfig.SingleDiff = config->single_diff;
    sConfig.OffsetNumber = config->offset_number;
    sConfig.Offset = config->offset;
    sConfig.OffsetSign = config->offset_sign;
    sConfig.OffsetSaturation = config->offset_saturation;

    return HAL_ADC_ConfigChannel(hadc, &sConfig);
}


/*
 * Initialize ADC peripheral
 */
HAL_StatusTypeDef adc_init(ADC_HandleTypeDef *hadc) {
    return HAL_ADC_Init(hadc);
}


/*
 * Calibrate the peripheral input mode as either single or differential ended
 */
HAL_StatusTypeDef adc_calibrate(ADC_HandleTypeDef *hadc, uint32_t input_mode) {
    return HAL_ADCEx_Calibration_Start(hadc, input_mode);
}


/*
 * Begin ADC conversion with given pin
 */
HAL_StatusTypeDef adc_start_convert(ADC_HandleTypeDef *hadc, adc_pin_e pin, adc_pin_config_t *config) {
    if (adc_configure_pin(hadc, pin, config) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_ADC_Start(hadc);
}


/*
 * Poll for completion, return poll status
 */
HAL_StatusTypeDef adc_poll_complete(ADC_HandleTypeDef *hadc, uint32_t *result, uint32_t timeout) {
    HAL_StatusTypeDef poll_status = HAL_ADC_PollForConversion(hadc, timeout);
    if (poll_status == HAL_OK) {
        *result = (uint32_t)HAL_ADC_GetValue(hadc);
    }
    HAL_ADC_Stop(hadc);
    return poll_status;
}