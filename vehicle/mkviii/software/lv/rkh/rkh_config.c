#include "rkh.h"
#include "rkh.yml"

// Function Implementations
void GpioInit(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO_InitTypeDef is the type. the struct is an oem variable
  HAL_GPIO_WritePin(PB5_GPIO_Port, PB5_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PB5_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO_InitTypeDef is the type. the struct is an oem variable
  HAL_GPIO_WritePin(PB6_GPIO_Port, PB6_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PB6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PB6_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO_InitTypeDef is the type. the struct is an oem variable
  HAL_GPIO_WritePin(PB7_GPIO_Port, PB7_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PB7_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO_InitTypeDef is the type. the struct is an oem variable
  GPIO_InitStruct.Pin = PA4_Pin;  // motor current input
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // defines PA4 as an input
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // confirm in office hours
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // same speed as VCU
  HAL_GPIO_Init(PA4_GPIO_Port, &GPIO_InitStruct);
}


void SysTick_Handler(void) {
  HAL_IncTick();
}