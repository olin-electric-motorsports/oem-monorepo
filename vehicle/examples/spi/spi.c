/*
This file is specifically meant for the STM32G441 dev board that was created and can be found in mkviii/hardware/example
*/

#include "spi.h"

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();

  while (1) {
    // 1. Initialize once at startup
  Spi_Init(); 

  // 2. Prepare your data
  uint8_t tx_buffer[2] = {0x80, 0x00}; // Example: Read from register 0x00
  uint8_t rx_buffer[2] = {0};          // To hold the response

  // 3. Execute the transaction
  Spi_Select();                                         // Pull PA15 Low
  Spi_TransmitReceive(tx_buffer, rx_buffer, 2);         // Clock out 2 bytes
  Spi_Deselect();                                       // Pull PA15 High

  // The sensor's response is now sitting in rx_buffer[1] !
  }
  return 0;
}

