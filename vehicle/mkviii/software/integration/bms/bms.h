#pragma once
//#include "stm32g4xx_hal.h"
# include "stdint.h"

// Hardware Mapping for BMS Heartbeat
// PA7 corresponds to Pin 12 on BMS MICRO schematic
#define HEARTBEAT_GPIO_Port GPIOA
#define HEARTBEAT_Pin       GPIO_PIN_7

// Function Prototypes
void GpioInit(void);
void SystemClockConfig(void);
void SysTick_Handler(void);

// SPI Function Prototypes 04/18/2026
void oem_spi_init(oem_spi_config_t *config);
void oem_spi_select(oem_spi_config_t *config);
void oem_spi_deselect(oem_spi_config_t *config);
int oem_spi_transmit_receive(oem_spi_config_t *config, uint8_t *txData, uint8_t *rxData, uint16_t size);