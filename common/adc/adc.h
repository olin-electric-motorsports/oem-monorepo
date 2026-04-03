#pragma once
#include "stm32g4xx_hal.h" 

/*
 * Converts raw ADC reading to voltage
 *
 * VDDA = 3.3V, 12-bit resolution (0-4095)
 */
#define AS_VOLTAGE(raw) ((float)(raw) * (3.3f / 4095.0f))

/*
 * Maps pin to ADC channel only (ADC handle specifies the peripheral)
 */
typedef enum {
    ADC_PA0_IN1  = ADC_CHANNEL_1,   /* ADC1 or ADC2 */
    ADC_PA1_IN2  = ADC_CHANNEL_2,   /* ADC1 or ADC2 */
    ADC_PA2_IN3  = ADC_CHANNEL_3,   /* ADC1 only */
    ADC_PA3_IN4  = ADC_CHANNEL_4,   /* ADC1 only */
    ADC_PA4_IN17 = ADC_CHANNEL_17,  /* ADC2 only */
    ADC_PA5_IN13 = ADC_CHANNEL_13,  /* ADC2 only */
    ADC_PA6_IN3  = ADC_CHANNEL_3,   /* ADC2 only — same channel as PA2 on ADC1 */
    ADC_PA7_IN4  = ADC_CHANNEL_4,   /* ADC2 only — same channel as PA3 on ADC1 */
    ADC_PB0_IN15 = ADC_CHANNEL_15,  /* ADC1 only */
} adc_pin_e;


typedef struct {
    uint32_t rank;
    uint32_t sampling_time;
    uint32_t single_diff;
    uint32_t offset_number;
    uint32_t offset;
    uint32_t offset_sign;        
    uint32_t offset_saturation;
} adc_pin_config_t;


/*
 * adc_configure_pin
 *
 * Configures ADC pin from a struct
 *
 * Parameters
 * - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 * - (adc_pin_e) pin: The ADC pin to use
 * - (adc_pin_config_t *): Struct with pin config parameters
 *
 * Returns (HAL_StatusTypeDef)
 * - HAL status code 
 */
HAL_StatusTypeDef adc_configure_pin(ADC_HandleTypeDef *hadc, adc_pin_e pin, adc_pin_config_t *config);


/*
 * adc_init
 *
 * Initializes ADC peripheral (1 or 2). 
 *
 * Parameters
 * - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *
 * Returns (HAL_StatusTypeDef)
 * - HAL status code 
 */
HAL_StatusTypeDef adc_init(ADC_HandleTypeDef *hadc);

/*
 * adc_calibrate
 *
 * Calibrates input mode of ADC peripheral (1 or 2) to support
 * either a single or differential analog signal. This should only be
 * called once before adc_start_convert
 *
 * Parameters
 * - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 * - (uint32_t) input_mode: Type of ADC input
 *       - pass input as ADC_SINGLE_ENDED or ADC_DIFFERENTIAL_ENDED
 *
 * Returns (HAL_StatusTypeDef)
 * - HAL status code 
 */
HAL_StatusTypeDef adc_calibrate(ADC_HandleTypeDef *hadc, uint32_t input_mode);

/*
 * adc_start_convert
 *
 * Begins ADC conversion with given ADC pin. 
 *
 * Note: This automatically configures the pin before starting conversion,
 *
 * Parameters
 *   - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *   - (adc_pin_e) pin: The ADC pin to use
 *   - (adc_pin_config_t *): Struct with pin config parameters
 *
 * Returns (HAL_StatusTypeDef)
 * - HAL status code 
 */
HAL_StatusTypeDef adc_start_convert(ADC_HandleTypeDef *hadc, adc_pin_e pin, adc_pin_config_t *config);

/*
 * adc_poll_complete
 *
 * Polls for completion of conversion, and, if successful, stores the value of
 * the ADC.
 *
 * Parameters
 *   - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *   - (uint32_t *) result: Pointer to where the value will be stored
 *   - (uint32_t) timeout: Polling timeout, in milliseconds
 *
 * Returns (HAL_StatusTypeDef)
 * - HAL status code 
 */
HAL_StatusTypeDef adc_poll_complete(ADC_HandleTypeDef *hadc, uint32_t *result, uint32_t timeout);