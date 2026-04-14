#pragma once
#include "stm32g4xx_hal.h"

#define PA1_GPIO_Port   GPIOA
#define PA1_Pin         GPIO_PIN_1

void SystemClockConfig();

// Function Prototypes
void GpioInit(void);
void SysTick_Handler(void);
void SystemClockConfig(void);


void oem_spi_init(void);

// Pulls the CS pin (PA15) LOW 
void oem_spi_select(void);

// Pulls the CS pin (PA15) HIGH 
void oem_spi_deselect(void);


int oem_spi_transmit_receive(uint8_t *txData, uint8_t *rxData, uint16_t size);