#pragma once
#include "stm32g4xx_hal.h"

#define PB6_GPIO_Port   GPIOB
#define PB6_Pin         GPIO_PIN_6

void SystemClockConfig();

// Function Prototypes
void GpioInit(void);
void SysTick_Handler(void);
void SystemClockConfig(void);
// #ifndef RKH_H
// #define RKH_H

// #include "stm32g4xx_hal.h"

// #define PB6_GPIO_Port   GPIOB
// #define PB6_Pin         GPIO_PIN_6


// // Function Prototypes
// // static void GpioInit(void) {
// //   __HAL_RCC_GPIOF_CLK_ENABLE();
// //   __HAL_RCC_GPIOB_CLK_ENABLE();

// //   GPIO_InitTypeDef GPIO_InitStruct = {0};

// //   // Uses PB6_... from example.h
// //   HAL_GPIO_WritePin(PB6_GPIO_Port, PB6_Pin, GPIO_PIN_RESET);

// //   GPIO_InitStruct.Pin = PB6_Pin;
// //   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
// //   GPIO_InitStruct.Pull = GPIO_NOPULL;
// //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
// //   HAL_GPIO_Init(PB6_GPIO_Port, &GPIO_InitStruct);
// // }



// void SysTick_Handler(void) {
//   HAL_IncTick();
// }




// #endif // RKH_H