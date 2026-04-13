#include "spi_example.h"

int main(void) {
    HAL_Init();
    SystemClockConfig();

    // Initialize once at startup
    oem_spi_init(&bms_spi); 

    uint8_t tx_buffer[2] = {0x80, 0x00}; 
    uint8_t rx_buffer[2] = {0};          
    uint16_t size = 1; // We want to read 1 byte, but we send 2 bytes 

    while (1) {
        // the transaction 
        oem_spi_select(&bms_spi);

        // configure, transmit, receive, size
        oem_spi_transmit_receive(&bms_spi, tx_buffer, rx_buffer, size); 

        // Deselect the device after the transaction
        oem_spi_deselect(&bms_spi);

        // The response is now sitting in rx_buffer[1]
        
        HAL_Delay(10);
    }
    
    return 0;
}