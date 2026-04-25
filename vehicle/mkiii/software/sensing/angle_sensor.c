/*
This file is specifically meant for the STM32G441 dev board that was created and can be found in mkviii/hardware/example
*/

#include "angle_sensor.h"
#include <stdio.h>

TIM_HandleTypeDef htim2;

//static void MX_GPIO_Init(void);
//static void MX_TIM2_Init(void);

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();

  //int placeHolderStatusInput = 1;

  //int i = 0;

  HAL_GPIO_WritePin(GPIOA, Debug_LED_Pin, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, Heartbeat_LED_Pin, GPIO_PIN_RESET); 

  while (1) {
    // if (i < 1000000){
    //   // Heartbeat LED
    //  HAL_Delay(10000000);

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
    // i = i + 1;
    // } 
    // else {
    //   HAL_GPIO_TogglePin(GPIOA, Debug_LED_Pin);
    //   i = 0;
    // }

    // // PWM_IN
    // int PWM_Val = HAL_GPIO_ReadPin(GPIOA, PWM_Input_Pin);
    // if (PWM_Val == 1) {
    //   //HAL_GPIO_WritePin(GPIOA, Debug_LED_Pin, GPIO_PIN_SET);
    // }
    // else if (PWM_Val == 0) {
    //   HAL_GPIO_WritePin(GPIOA, Debug_LED_Pin, GPIO_PIN_RESET);
    // }
    
    
  }
  return 0;


}