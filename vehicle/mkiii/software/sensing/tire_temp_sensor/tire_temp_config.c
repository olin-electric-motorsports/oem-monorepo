#include "tire_temp.h"


void SysTick_Handler(void) {
  HAL_IncTick();
}
static void GpioInit(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*// Uses PA1_... from throttle.h
  HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);*/
  //defining heartbeat pin
  HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = HEARTBEAT_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEARTBEAT_LED_PORT, &GPIO_InitStruct);
  
  //define debug LED
  
  HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = DEBUG_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DEBUG_LED_PORT, &GPIO_InitStruct);

}
