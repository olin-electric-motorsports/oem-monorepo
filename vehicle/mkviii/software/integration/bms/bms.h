#pragma once
#include "stm32g4xx_hal.h"

// Hardware Mapping for BMS Heartbeat
// PA7 corresponds to Pin 12 on BMS MICRO schematic
#define HEARTBEAT_GPIO_Port GPIOA
#define HEARTBEAT_Pin       GPIO_PIN_7

// Function Prototypes
void GpioInit(void);
void SystemClockConfig(void);
void SysTick_Handler(void);