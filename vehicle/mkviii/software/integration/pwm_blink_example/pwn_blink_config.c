#include "pwm_blink.h"

// This handle acts as the "remote control" for Timer 1
TIM_HandleTypeDef htim1;

void GpioInit(void) {
    // 1. Enable the clock for Port A (GPIOs need power to work)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 2. Configure PA8 for "Alternate Function" 
    // This tells the pin: "Don't listen to the CPU, listen to Timer 1"
    GPIO_InitStruct.Pin = PWM_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      // Alternate Function, Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // No internal resistors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // PWM is relatively slow
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;   // AF6 connects PA8 to TIM1
    HAL_GPIO_Init(PWM_PORT, &GPIO_InitStruct);
}

void PwmInit(void) {
    // 3. Enable power to the Timer 1 peripheral
    __HAL_RCC_TIM1_CLK_ENABLE();

    // 4. Timer Math: Create a 1kHz frequency
    // Assuming 16MHz system clock: (16MHz / 16) = 1MHz internal tick rate
    htim1.Instance = PWM_TIMER;
    htim1.Init.Prescaler = 16 - 1;           
    htim1.Init.Period = 1000 - 1;            // Count 0 to 999 (1000 steps)
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&htim1);

    // 5. Channel Configuration: Define how the pulses look
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;      // Standard PWM mode
    sConfigOC.Pulse = 0;                     // Start with 0% duty cycle (OFF)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, PWM_CHANNEL);

    // 6. Start the hardware
    HAL_TIM_PWM_Start(&htim1, PWM_CHANNEL);
}

void SysTick_Handler(void) {
    HAL_IncTick(); // Required for HAL_Delay() to work
}