#include "pwm_blink.h"

// holds the configuration settings for Timer 1
TIM_HandleTypeDef htim1;

void PWM_Init(void) {
    /* STEP 1: Power on the hardware */
    __HAL_RCC_TIM1_CLK_ENABLE();  // Turn on the clock for Timer 1
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Turn on the clock for Port A 

    /* STEP 2: Configure the Physical Pin */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PWM_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // tells the pin to listen to a Timer, not just high/low code
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // No internal resistor needed
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // PWM at 1kHz is slow so low speed is fine
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;    // Link this specific pin to the Timer 1 hardware
    HAL_GPIO_Init(PWM_PORT, &GPIO_InitStruct);    // Write these settings to the hardware registers

    /* STEP 3: Set the heartbeat of the Timer */
    htim1.Instance = PWM_TIMER;
    // Prescaler: 16MHz clock / 16 = 1MHz internal timer speed (1 tick per microsecond)
    htim1.Init.Prescaler = 16 - 1; 
    // Period: Count up to 1000 before resetting. 1MHz / 1000 = 1kHz PWM frequency.
    htim1.Init.Period = 1000 - 1; 
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&htim1);

    /* STEP 4: Configure the PWM Channel */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;           // PWM Mode 1: Pin is HIGH while counter < Pulse value
    sConfigOC.Pulse = 0;                          // Start at 0 (Off)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;   // "Active" means 3.3V
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, PWM_CHANNEL);

    /* STEP 5: Start the hardware */
    HAL_TIM_PWM_Start(&htim1, PWM_CHANNEL);       // Tell the timer to start counting and outputting signal
}

/**
 * Sets the brightness/duty cycle.
 * @param duty: A value from 0 (Always Off) to 1000 (Always On)
 */
void PWM_SetDutyCycle(uint32_t duty) {
    // This macro writes directly to the Compare Register (CCR1)
    // When the timer count is less than 'duty', the pin stays HIGH.
    __HAL_TIM_SET_COMPARE(&htim1, PWM_CHANNEL, duty);
}