#pragma once
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "common/spi/spi.h"

// Function Prototypes
void SystemClockConfig(void);
void GpioInit(void);
void SysTick_Handler(void);
