#ifndef CAN_EXAMPLE_H
#define CAN_EXAMPLE_H

#include "stm32g4xx_hal.h"

#define PA1_GPIO_Port   GPIOA
#define PA1_Pin         GPIO_PIN_1






void SysTick_Handler(void) {
  HAL_IncTick();
}

#endif // CAN_EXAMPLE_H