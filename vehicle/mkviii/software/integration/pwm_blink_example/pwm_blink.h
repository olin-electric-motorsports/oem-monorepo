#pragma once
#include "stm32g4xx_hal.h"

// Hardware Mapping: PA8 is Timer 1 Channel 1 on the G441
#define PWM_PORT      GPIOA
#define PWM_PIN       GPIO_PIN_8
#define PWM_TIMER     TIM1
#define PWM_CHANNEL   TIM_CHANNEL_1

// Function Prototypes - matches your blinky.h style
void GpioInit(void);          // Sets up the physical pin
void PwmInit(void);           // Sets up the timer math
void SystemClockConfig(void); 
void SysTick_Handler(void);