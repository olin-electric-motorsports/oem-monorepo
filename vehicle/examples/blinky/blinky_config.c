#include "blinky.h"

TIM_HandleTypeDef htim2;

void GpioInit(void) {
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = PA1_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PA1_GPIO_Port, &GPIO_InitStruct);
}

void TimerInit(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = (tim_clk / 10000) - 1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;   // 10,000 ticks @ 10kHz = 1000ms
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; //Makes sure it doesn't change period during a cycle
    HAL_TIM_Base_Init(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
   HAL_TIM_IRQHandler(&htim2);
 }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(PA1_GPIO_Port, PA1_Pin);
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}