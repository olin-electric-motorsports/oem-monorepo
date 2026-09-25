/*!
  ADBMS181x hardware library
@verbatim
  This library contains all of the hardware dependant functions used by the bms
  code edited by Jacob Likins for the STM32G441
@endverbatim

Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright 2017 Linear Technology Corp. (LTC)
*/
#include "bms_hardware.h"
#include "common/spi/spi.h"
#include "stm32g4xx_hal.h"
#include <string.h>

// Pull the hardware config from spi_config.c
extern oem_spi_config_t bms_spi;
extern uint32_t SystemCoreClock; 

// Pull the hardware config from spi_config.c
extern oem_spi_config_t bms_spi;

void cs_low(uint8_t pin) {
    oem_spi_select(&bms_spi);
    delay_u(2);
}

void cs_high(uint8_t pin) {
    oem_spi_deselect(&bms_spi);
    delay_u(2);
}

//microsecond delay based on CPU cycles cause HAL doesn't have this
void delay_u(uint16_t micro) {
    // Calculate cycles needed (Clock speed / 1 million gives cycles per microsecond)
    // Divide by 4 because the while loop takes roughly 4 instructions per iteration
    volatile uint32_t delay_cycles = (SystemCoreClock / 1000000) * micro / 4;
    
    while (delay_cycles--) {
        __NOP(); // No-operation
    }
}

void delay_m(uint16_t milli) {
    HAL_Delay(milli);
}

void spi_write_array(uint8_t len, uint8_t data[]) {
    if (len > 0) {
        uint8_t dummy_rx[32]; 
        oem_spi_transmit_receive(&bms_spi, data, dummy_rx, len);
    }
}

void spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len) {
    __disable_irq();
    // end the command 
    if (tx_len > 0) {
        uint8_t dummy_rx_cmd[tx_len];
        oem_spi_transmit_receive(&bms_spi, tx_Data, dummy_rx_cmd, tx_len);
    }
    
    // Read the response 
    if (rx_len > 0) {
        uint8_t dummy_tx_data[rx_len];
        memset(dummy_tx_data, 0xFF, rx_len);
        oem_spi_transmit_receive(&bms_spi, dummy_tx_data, rx_data, rx_len);
    }
    __enable_irq();
}

uint8_t spi_read_byte(uint8_t tx_dat) {
    uint8_t rx_data;
    oem_spi_transmit_receive(&bms_spi, &tx_dat, &rx_data, 1);
    return rx_data;
}