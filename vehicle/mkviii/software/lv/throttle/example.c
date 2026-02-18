#include "example.h"
#include <stdbool.h>

volatile bool is_heart_beating = true;

static void GpioInit(void) {
  // 1. Enable GPIO Clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE(); // Likely unnecessary unless using HSE on Port F

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* --- Configure PA0 (Interrupt Input) --- */
  GPIO_InitStruct.Pin = PA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PA0_GPIO_Port, &GPIO_InitStruct);

  // CRITICAL FIX: Enable NVIC for PA0 (EXTI Line 0)
  // Refer to UM2570 Section 3.11.2 GPIOs -> EXTI Mode
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0); // Set priority (adjust as needed)
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);         // Enable the Interrupt

  /* --- Configure PA1 (Output) --- */
  // Good Practice: Set level BEFORE configuring as output to prevent glitches
  HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = PA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA1_GPIO_Port, &GPIO_InitStruct);
}

void ErrorHandler(void) { // this will only happen if error on board
  __disable_irq();
  HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_SET);
  while (1) {
  }
}

void SystemClockConfig(void) { // if you are messing with this please consult someone
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    ErrorHandler();
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void EXTI0_IRQHandler(void)
{
    // Tell the HAL library to handle the interrupt logic
    // It will clear the flag and call the callback function below
    HAL_GPIO_EXTI_IRQHandler(PA0_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    is_heart_beating = !is_heart_beating;
}

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  
  while (1) {
    if (is_heart_beating){
      HAL_GPIO_TogglePin(PA1_GPIO_Port, PA1_Pin);
      HAL_Delay(500);
    } else {
      HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
    }
  }
  return 0;
}

