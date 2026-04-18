/*
 * BMS Heartbeat firmware for STM32G441KBT6.
 * Specifically targets the PA7 Heartbeat pin on the BMS Micro board.
 */
#include "bms.h"
#include "bms_config.h" //added to access SPI 04/18/26
#include "common/spi/api.h"

int main(void) {
    // 1. Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // 2. Configure the system clock (usually defined in a common library)
    SystemClockConfig();

    // 3. Initialize the Heartbeat GPIO pin
    GpioInit();

    // 4. Initialize SPI 04/18/26
    oem_spi_init(&bms_spi);

    // buffers for LTC6811 communication
    uint8_t tx_buffer[2] = {0x00, 0x01}; //example command
    uint8_t rx_buffer[2] = {0};


    // Infinite Loop
    while (1) {
        // Toggle the state of the Heartbeat pin (High -> Low or Low -> High)
        HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);

        // SPI transaction example 04/18/26
        // 1. Select the Subnode (Pull CS Low)
        oem_spi_select(&bms_spi);

        // 2. Transmit/Receive data
        oem_spi_transmit_receive(&bms_spi, tx_buffer, rx_buffer, 2);

        // 3. Deselect the Subnode (Pull CS High)
        oem_spi_deselect(&bms_spi);

        // Wait for 500 milliseconds (0.5 seconds)
        // Adjust this value to change the blink speed
        HAL_Delay(500); 
    }

    return 0;
}