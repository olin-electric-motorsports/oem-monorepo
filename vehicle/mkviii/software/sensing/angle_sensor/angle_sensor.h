#pragma once
#include "stm32g4xx_hal.h"

#define PWM_Input_Pin     GPIO_PIN_0
#define Status_Signal_Pin      GPIO_PIN_1
#define Debug_LED_Pin      GPIO_PIN_8
#define Heartbeat_LED_Pin      GPIO_PIN_9
#define CAN_RX_Pin      GPIO_PIN_11
#define CAN_TX_Pin      GPIO_PIN_12

void SystemClockConfig();

// Function Prototypes
void GpioInit(void);
void SysTick_Handler(void);
void SystemClockConfig(void);
