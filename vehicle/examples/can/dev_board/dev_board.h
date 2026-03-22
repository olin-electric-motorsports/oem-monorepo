#ifndef DEV_BOARD_H
#define DEV_BOARD_H

#include "stm32g4xx_hal.h"

#define PA1_GPIO_Port   GPIOA
#define PA1_Pin         GPIO_PIN_1


void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan) {
    if (hfdcan->Instance == FDCAN1) {
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // Configure PA11  and PA12
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

//it uses the OEM dev board for STM32G441
// Function Prototypes
static void GpioInit(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Uses PA1_... from dev_board.h
  HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = PA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA1_GPIO_Port, &GPIO_InitStruct);
}


void SysTick_Handler(void) {
  HAL_IncTick();
}

#endif // DEV_BOARD_H