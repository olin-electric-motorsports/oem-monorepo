#include "tire_temp.h"


void SysTick_Handler(void) {
  HAL_IncTick();
}
void GpioInit(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // SDA pin
  HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;//alternate function open drain
  GPIO_InitStruct.Pull = GPIO_PULLUP;//enable pullup
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;  // AF for I2C1
  HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
  
  //SCL pin
  HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;//alternate function open drain
  GPIO_InitStruct.Pull = GPIO_PULLUP;//enable pullup
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);

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

  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

}
