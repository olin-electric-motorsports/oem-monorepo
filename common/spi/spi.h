#pragma once
#include "stm32g4xx_hal.h"

typedef struct {
    SPI_TypeDef* spi_instance; // SPI1
    GPIO_TypeDef* cs_port; // GPIOA
    uint16_t cs_pin; // GPIO_PIN_15
    uint32_t baud_prescaler; // SPI_BAUDRATEPRESCALER_64
} oem_spi_config_t;

// The abstracted functions
void oem_spi_init(oem_spi_config_t* config);
void oem_spi_select(oem_spi_config_t* config);
void oem_spi_deselect(oem_spi_config_t* config);
int oem_spi_transmit_receive(oem_spi_config_t* config, uint8_t *txData, uint8_t *rxData, uint16_t size);