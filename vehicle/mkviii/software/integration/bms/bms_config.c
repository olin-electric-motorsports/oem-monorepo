#include "bms.h"

// Function Implementations
void GpioInit(void) {
    // 1. Enable GPIO Clocks
    //  enable Port A because the Heartbeat (PA7) is on Port A
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 2. Initialize Heartbeat Pin
    // Ensure the pin starts in a known state (Low/Reset)
    HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = HEARTBEAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // Standard Push-Pull Output
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // No internal pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Low speed is fine for a blinker
    
    // Apply settings to the hardware
    HAL_GPIO_Init(HEARTBEAT_GPIO_Port, &GPIO_InitStruct);
}

void SysTick_Handler(void) {
    // This increments the global tick counter every 1ms
    // This is required for HAL_Delay() to work
    HAL_IncTick();
}