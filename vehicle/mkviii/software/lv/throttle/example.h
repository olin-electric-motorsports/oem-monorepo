#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "stm32g4xx_hal.h"

#define PA0_GPIO_Port   GPIOA
#define PA0_Pin         GPIO_PIN_0

#define PA1_GPIO_Port   GPIOA
#define PA1_Pin         GPIO_PIN_1

// Function Prototypes
static void GpioInit(void);

void ErrorHandler(void);

void SystemClockConfig(void);

void SysTick_Handler(void);


#endif // THROTTLE_H