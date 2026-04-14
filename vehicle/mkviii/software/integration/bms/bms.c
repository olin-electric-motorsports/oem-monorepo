/*
 * BMS Heartbeat firmware for STM32G441KBT6.
 * Specifically targets the PA7 Heartbeat pin on the BMS Micro board.
 */
#include "bms.h"

int main(void) {
    // 1. Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // 2. Configure the system clock (usually defined in a common library)
    SystemClockConfig();

    // 3. Initialize the Heartbeat GPIO pin
    GpioInit();

    // Infinite Loop
    while (1) {
        // Toggle the state of the Heartbeat pin (High -> Low or Low -> High)
        HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);

        // Wait for 500 milliseconds (0.5 seconds)
        // Adjust this value to change the blink speed
        HAL_Delay(500); 
    }

    return 0;
}