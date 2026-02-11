#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "stm32g4xx_hal.h"

#define PA0_GPIO_Port   GPIOA
#define PA0_Pin         GPIO_PIN_0

#define PA1_GPIO_Port   GPIOA
#define PA1_Pin         GPIO_PIN_1

// Function Prototypes
static void GpioInit(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Uses PA1_... from throttle.h
  HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = PA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA1_GPIO_Port, &GPIO_InitStruct);
}