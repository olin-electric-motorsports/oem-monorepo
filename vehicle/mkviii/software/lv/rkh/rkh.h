#pragma once
#include "stm32g4xx_hal.h"
#include "can_api.h"

#define PB5_GPIO_Port   GPIOB
#define PB5_Pin         GPIO_PIN_5

#define PB6_GPIO_Port   GPIOB
#define PB6_Pin         GPIO_PIN_6

#define PB7_GPIO_Port   GPIOB
#define PB7_Pin         GPIO_PIN_7

#define PA4_GPIO_Port   GPIOA
#define PA4_Pin         GPIO_PIN_4

void SystemClockConfig();

// Function Prototypes - declares inputs and outputs of function
void GpioInit(void);
void SysTick_Handler(void);
void SystemClockConfig(void);
 // header file for c language
 // 