/*
 * PWM Blinky for STM32G441 dev board.
 * Targets PA8 (External Signal 1).
 */
#include "pwm_blink.h"

// Variable to control duty cycle (0 = off, 1000 = full bright)
extern TIM_HandleTypeDef htim1;

int main(void) {
    HAL_Init();               // Standard HAL boot
    SystemClockConfig();      // Setup internal clocks
    GpioInit();               // Setup PA8 pin
    PwmInit();                // Setup Timer 1

    uint32_t dutyCycle = 0;   // Start at zero brightness
    int increment = 25;       // How much to change brightness each step

    while (1) {
        // Update the PWM duty cycle
        // This is the "PWM version" of writing a pin High or Low
        __HAL_TIM_SET_COMPARE(&htim1, PWM_CHANNEL, dutyCycle);

        // Logic to fade up and down
        dutyCycle += increment;

        // If we hit the top (1000) or bottom (0), reverse the fade
        if (dutyCycle <= 0 || dutyCycle >= 1000) {
            increment = -increment; 
        }

        // Delay 30ms so we can see the smooth transition
        HAL_Delay(30); 
    }
    return 0;
}