#pragma once
#include "stm32g4xx_hal.h"

/*
* GPIO macros
*/
#define ANALOG_TEST_INPUT_GPIO_PORT   GPIOA
#define ANALOG_TEST_INPUT_PIN         GPIO_PIN_3
#define ANALOG_TEST_INPUT_CHANNEL     ADC_PA3_IN4
#define BLINKY_LED_GPIO_PORT          GPIOA
#define BLINKY_LED_PIN                GPIO_PIN_1

extern ADC_HandleTypeDef hadc; // Shared variable

/*
* Function prototypes - config
*/
void GpioInit(void);

/*
* Function prototypes - main
*/
void SysTick_Handler(void);
void SystemClockConfig(void); // try deleting this, may be unneccesary
