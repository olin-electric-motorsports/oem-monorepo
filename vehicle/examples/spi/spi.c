#include "spi.h"
#include <stdbool.h>

void adbms1818_wakeup(void) {
    uint8_t dummy_tx[1] = {0xFF};
    uint8_t dummy_rx[1] = {0};
    
    // Tap 1: Wakes the isoSPI transceiver
    oem_spi_select(&bms_spi);
    oem_spi_transmit_receive(&bms_spi, dummy_tx, dummy_rx, 1);
    oem_spi_deselect(&bms_spi);
    
    HAL_Delay(2); // Wait for the receiver to power up
    
    // Tap 2: Wakes the internal digital core
    oem_spi_select(&bms_spi);
    oem_spi_transmit_receive(&bms_spi, dummy_tx, dummy_rx, 1);
    oem_spi_deselect(&bms_spi);
    
    HAL_Delay(3); // Give the core time to boot
}

int main(void) {
    HAL_Init();
    SystemClockConfig();
    
    // FIXED: Added &bms_spi
    oem_spi_init(&bms_spi); 

    // RDCVA Command: 0x00 0x04. Pre-calculated PEC15: 0x07 0xC2
    uint8_t cmd_tx[4] = {0x00, 0x04, 0x07, 0xC2}; 
    uint8_t cmd_rx[4] = {0}; 
    
    // We expect 8 bytes back: 6 bytes of voltage data (3 cells * 2 bytes) + 2 bytes of PEC
    // We send 0xFF to keep the clock ticking while we listen
    uint8_t data_tx[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
    uint8_t data_rx[8] = {0}; 

    while (1) {
        adbms1818_wakeup();

        oem_spi_select(&bms_spi);
        
        // 1. Send the command (This uses cmd_tx and cmd_rx!)
        oem_spi_transmit_receive(&bms_spi, cmd_tx, cmd_rx, 4);
        
        // 2. Read the data and trap the status
        volatile int status = oem_spi_transmit_receive(&bms_spi, data_tx, data_rx, 8);
        
        oem_spi_deselect(&bms_spi);

        // Mute the compiler warnings for our debug variables
        (void)status; 
        (void)data_rx[0]; 

        HAL_Delay(100); 
    }
    
    return 0;
}