#pragma once
#include "stm32g4xx_hal.h"
#include "common/adc/adc.h"


/*
* GPIO macros
*/

#define BLINKY_LED_GPIO_PORT          GPIOA
#define BLINKY_LED_PIN                GPIO_PIN_1

// extern ADC_HandleTypeDef hadc1; // Shared variable
// extern adc_pin_config_t PA4_config;

/*
* Function prototypes - config
*/
void GpioInit(void);
extern oem_adc_config_t throttle_sensor;
/*
* Function prototypes - main
*/
void SysTick_Handler(void);
void SystemClockConfig(void);
