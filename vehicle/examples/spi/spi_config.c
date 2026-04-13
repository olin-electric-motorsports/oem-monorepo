#include "spi_example.h"

// Single Source of Truth for the hardware routing
oem_spi_config_t bms_spi = {
    .spi_instance = SPI1, // Using SPI1 refer to datasheet for pinout
    .cs_port = GPIOA, // Chip Select on GPIOA
    .cs_pin = GPIO_PIN_15, // CS pin number
    .baud_prescaler = SPI_BAUDRATEPRESCALER_64 // Adjust as needed for your sensor's max SPI speed
};