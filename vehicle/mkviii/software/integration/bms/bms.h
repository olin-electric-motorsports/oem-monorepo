#pragma once
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "common/spi/spi.h"

// Hardware Mapping for BMS Heartbeat
// PA7 corresponds to Pin 12 on BMS MICRO schematic
#define HEARTBEAT_GPIO_Port GPIOA
#define HEARTBEAT_Pin       GPIO_PIN_7

// Function Prototypes
void SystemClockConfig(void);
void GpioInit(void);
void SysTick_Handler(void);