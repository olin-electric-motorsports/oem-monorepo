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

/*
 * adc_init
 *
 * Initializes ADC peripheral (1 or 2) using the handle. Must be called 
 * before ADC conversion is enabled.
 *
 * Parameters
 * - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *
 * Returns (HAL_StatusTypeDef)
 * - HAL status code 
 */
HAL_StatusTypeDef adc_init(ADC_HandleTypeDef *hadc)

/*
 * adc_start_convert
 *
 * Begins ADC conversion with given ADC
 *
 * Parameters
 *   - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *   - (adc_pin_e) pin: The ADC pin to use
 *
 * Returns (HAL_StatusTypeDef)
 * - HAL status code 
 */
HAL_StatusTypeDef adc_start_convert(ADC_HandleTypeDef *hadc, adc_pin_e pin)

/*
 * adc_poll_complete
 *
 * Polls for completion of conversion, and, if successful, stores the value of
 * the ADC.
 *
 * Parameters
 *   - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *   - (uint16_t *) result: Pointer to where the value will be stored
 *
 * Returns (HAL_StatusTypeDef)
 * - HAL status code 
 */
HAL_StatusTypeDef adc_poll_complete(ADC_HandleTypeDef *hadc, uint16_t *result)

/*
 * adc_read_results
 *
 * Reads the value of the ADC register. Used as an alternative to
 * adc_poll_complete when using interrupts.
 *
 * Parameters
 *   - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *   - (uint16_t *) result: Pointer to where the value will be stored
 *
 * Returns (void)
 */
void adc_read_results(ADC_HandleTypeDef *hadc, uint16_t *result)

/*
 * adc_interrupt_enable
 *
 * Enables interrupt when ADC data is complete and registers a callback function
 * to be called in HAL Complete Callback (HAL_ADC_ConvCpltCallback).
 *
 * Parameters:
 *   - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *   - (adc_pin_e) pin: The ADC pin to use
 *   - (function ptr) callback: Function to be called upon end of ADC conversion
 *
 * Returns (void)
 */
void adc_interrupt_enable(ADC_HandleTypeDef *hadc, adc_pin_e pin, void (*callback)(void))

/*
 * adc_read
 *
 * Function to read an ADC value. Wraps other functions and is provided as a
 * convenience.
 *
 * WARNING: This function is blocking and won't return until the conversion is
 * completed or timed out.
 *
 * Parameters:
 *   - (ADC_HandleTypeDef *) hadc: Pointer to ADC handle
 *   - (adc_pin_e) pin: The ADC pin to use
 *
 * Returns (uint16_t)
 *   - Value of the ADC conversion
 */
uint16_t adc_read(ADC_HandleTypeDef *hadc, adc_pin_e pin)