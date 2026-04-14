#include "brake_temp.h"


void GpioInit(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Uses PA1_... from throttle.h
  HAL_GPIO_WritePin(GPIOA, PA1_Pin, GPIO_PIN_RESET);

  // Heartbeat LED assignments
  GPIO_InitStruct.Pin = HEARTBEAT_LED;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Debug LED assignemnts
  GPIO_InitStruct.Pin = DEBUG_LED;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // IR Output assignments
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

  // Configure ADC Pin
oem_adc_config_t brake_temp_sensor = {
    .adc_instance = ADC1,
    .port = GPIOA,
    .pin = GPIO_PIN_0,
    .channel = ADC_CHANNEL_1,
    .sample_time = ADC_SAMPLETIME_47CYCLES_5
  };
    

void SysTick_Handler(void) {
  HAL_IncTick();
}