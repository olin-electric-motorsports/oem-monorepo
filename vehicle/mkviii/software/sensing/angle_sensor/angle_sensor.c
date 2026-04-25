/*
This file is specifically meant for the STM32G441 dev board that was created and can be found in mkviii/hardware/example
*/

#include "angle_sensor.h"
#include <stdio.h>

TIM_HandleTypeDef htim2;

uint32_t frequency, duty_cycle;
uint32_t capture_value;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if (capture_value == 0) {
      frequency = SystemCoreClock / (capture_value);
      duty_cycle = 10000 * HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / capture_value;
    }
  }
}
//static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    printf("we");
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    printf("we");
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
    printf("we");
  }
  
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) {
    printf("we");
  }


  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
    printf("we");
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
    printf("we");
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    printf("we");
  }

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  MX_TIM2_Init();

  //int placeHolderStatusInput = 1;

  int i = 0;

  HAL_GPIO_WritePin(GPIOA, Debug_LED_Pin, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, Heartbeat_LED_Pin, GPIO_PIN_RESET); 

  while (1) {
    if (i < 1000000){
    //   // Heartbeat LED
      //HAL_Delay(10000);

    //   // Debug/status signal
    //   // HAL_GPIO_ReadPin(GPIOA, Status_Signal_Pin)
    //   // if (placeHolderStatusInput == 1){
    //   //   HAL_GPIO_WritePin(GPIOA, Debug_LED_Pin, GPIO_PIN_SET); 
    //   // }
    // //i = i + 1;
    //   // Temporary test code
    //   // if (placeHolderStatusInput == 1){
    //   //   HAL_GPIO_WritePin(GPIOA, Debug_LED_Pin, GPIO_PIN_SET); 
    //   // }
    i = i + 1;
    } 
    else {
      HAL_GPIO_TogglePin(GPIOA, Heartbeat_LED_Pin);
      i = 0;
    }
    // PWM_IN

    //int capture_value = HAL_GPIO_ReadPin(GPIOA, PWM_Input_Pin);
    if (frequency > 0) {
      HAL_GPIO_WritePin(GPIOA, Debug_LED_Pin, GPIO_PIN_SET);
    }
    //else if (capture_value == 0) {
      //HAL_GPIO_WritePin(GPIOA, Debug_LED_Pin, GPIO_PIN_SET);
    //}
    
    
  }
  return 0;


}