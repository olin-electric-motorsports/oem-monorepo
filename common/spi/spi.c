#include "spi.h"
#include <stdbool.h>

// HAL handles for SPI
static SPI_HandleTypeDef hspi1;
static bool is_spi1_initialized = false;

void oem_spi_init(oem_spi_config_t* config) {
    // Turn on the clocks for SPI1, the CS port, and the SCK/MISO/MOSI port
    if (config->spi_instance == SPI1) __HAL_RCC_SPI1_CLK_ENABLE();
    
    if (config->cs_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (config->cs_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (config->cs_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    
    __HAL_RCC_GPIOB_CLK_ENABLE(); // PB3, PB4, PB5 are standard for SPI1 on your board

    // Configure the custom Chip Select Pin passed via the struct
    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET); // Default to HIGH
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = config->cs_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(config->cs_port, &GPIO_InitStruct);

    // PB3=SCK, PB4=MISO, PB5=MOSI
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; 
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Boot up the SPI 
    if (config->spi_instance == SPI1 && !is_spi1_initialized) {
        hspi1.Instance = SPI1;
        hspi1.Init.Mode = SPI_MODE_MASTER;
        hspi1.Init.Direction = SPI_DIRECTION_2LINES;
        hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi1.Init.NSS = SPI_NSS_SOFT;
        hspi1.Init.BaudRatePrescaler = config->baud_prescaler; 
        hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; 
        hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        
        HAL_SPI_Init(&hspi1);
        is_spi1_initialized = true;
    }
}

void oem_spi_select(oem_spi_config_t* config) {
    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_RESET);
}

void oem_spi_deselect(oem_spi_config_t* config) {
    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);
}

int oem_spi_transmit_receive(oem_spi_config_t* config, uint8_t *txData, uint8_t *rxData, uint16_t size) {
    // Determine which hardware to use
    SPI_HandleTypeDef* active_hspi;
    if (config->spi_instance == SPI1) active_hspi = &hspi1;
    else return -1; // Add SPI2 later 

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(active_hspi, txData, rxData, size, 100);
    return (status == HAL_OK) ? 0 : -1;
}