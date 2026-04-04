#include "adc.h"
#include <stdbool.h>

// Global handles for both hardware ADCs
static ADC_HandleTypeDef hadc1 = {.Instance = ADC1};
static ADC_HandleTypeDef hadc2 = {.Instance = ADC2};
static bool is_adc1_calibrated = false;
static bool is_adc2_calibrated = false;

static void enable_clocks(GPIO_TypeDef* port, ADC_TypeDef* adc) {
    // STM32G4 uses a shared clock bus for both ADC1 and ADC2
    if (adc == ADC1 || adc == ADC2) __HAL_RCC_ADC12_CLK_ENABLE(); 
    
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
}

void oem_adc_init(oem_adc_config_t* config) {
    enable_clocks(config->port, config->adc_instance);

    // Put the pin in Analog Mode
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = config->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(config->port, &GPIO_InitStruct);

    // Boot up ADC1 if requested
    if (config->adc_instance == ADC1 && !is_adc1_calibrated) {
        hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
        hadc1.Init.Resolution = ADC_RESOLUTION_12B;
        hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
        HAL_ADC_Init(&hadc1);
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        is_adc1_calibrated = true;
    }
    
    // Boot up ADC2 if requested
    if (config->adc_instance == ADC2 && !is_adc2_calibrated) {
        hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
        hadc2.Init.Resolution = ADC_RESOLUTION_12B;
        hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
        HAL_ADC_Init(&hadc2);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
        is_adc2_calibrated = true;
    }
}

uint16_t oem_adc_read(oem_adc_config_t* config) {
    // Figure out which hardware engine to talk to based on the struct
    ADC_HandleTypeDef* active_hadc;
    if (config->adc_instance == ADC1) active_hadc = &hadc1;
    else if (config->adc_instance == ADC2) active_hadc = &hadc2;
    else return 0; 

    // Configure (based off of example code)
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = config->channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = config->sample_time;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    
    HAL_ADC_ConfigChannel(active_hadc, &sConfig);

    // Take the reading
    HAL_ADC_Start(active_hadc);
    if (HAL_ADC_PollForConversion(active_hadc, 5) == HAL_OK) {
        return HAL_ADC_GetValue(active_hadc);
    }
    
    return 0;
}




