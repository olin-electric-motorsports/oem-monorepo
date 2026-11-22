#pragma once
#include "stm32g4xx_hal.h"

#include "common/spi/spi.h"

#define PA1_GPIO_Port   GPIOA
#define PA1_Pin         GPIO_PIN_1

void SystemClockConfig();

// Function Prototypes
void GpioInit(void);
void SysTick_Handler(void);
void SystemClockConfig(void);


extern oem_spi_config_t bms_spi;


void oem_spi_init(oem_spi_config_t* config);
void oem_spi_select(oem_spi_config_t* config);
void oem_spi_deselect(oem_spi_config_t* config);
int oem_spi_transmit_receive(oem_spi_config_t* config, uint8_t *txData, uint8_t *rxData, uint16_t size);