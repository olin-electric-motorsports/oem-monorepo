#include "spi.h"

// Function Implementations
#include "spi.h"

// Global SPI handle
SPI_HandleTypeDef hspi1;

// Part 1: Hardware-Level Initialization (Clocks and Pins)
// The HAL_SPI_Init() function will automatically call this.
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hspi->Instance == SPI1) {
        // 1. Enable Clocks
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        // 2. Configure PA15 as Software CS (Chip Select)
        // Set it HIGH by default so the SPI device is not accidentally selected
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // 3. Configure PB3 (SCK), PB4 (MISO), and PB5 (MOSI)
        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate Function Push-Pull
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // Needed for fast SPI
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1; // AF5 routes these pins to SPI1
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void Spi_Init(void) {
    hspi1.Instance = SPI1;
    
    // Configure SPI Mode 
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    
    // SPI Clock Polarity and Phase
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    
    // Chip Select
    hspi1.Init.NSS = SPI_NSS_SOFT;
    
    //  system clock is 170MHz, a prescaler of 64 gives a ~2.6MHz SPI clock.
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; 
    
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; // Most devices expect MSB first
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        while(1) {}
    }
}

// Pulls PA15 LOW to select the device
void Spi_Select(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

// Pulls PA15 HIGH to deselect the device
void Spi_Deselect(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

// Wrapper to transmit and receive data at the same time
int Spi_TransmitReceive(uint8_t *txData, uint8_t *rxData, uint16_t size) {
  
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, txData, rxData, size, 100);
    
    if (status == HAL_OK) {
        return 0; // Success
    }
    return -1; // Error or Timeout
}