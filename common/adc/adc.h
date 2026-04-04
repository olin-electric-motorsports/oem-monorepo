#pragma once
#include "stm32g4xx_hal.h"


//*
//  * Converts raw ADC reading to voltage
//  *
//  * VDDA = 3.3V, 12-bit resolution (0-4095)
//  */
// #define AS_VOLTAGE(raw) ((float)(raw) * (3.3f / 4095.0f))


// holds hardware mapping
typedef struct {
    ADC_TypeDef* adc_instance;   
    GPIO_TypeDef* port;          
    uint16_t pin;                
    uint32_t channel;            
    uint32_t sample_time;        
} oem_adc_config_t;

void oem_adc_init(oem_adc_config_t* config);
uint16_t oem_adc_read(oem_adc_config_t* config);



