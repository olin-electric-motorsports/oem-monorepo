/*
This file is specifically meant for the STM32G441 dev board that was created and can be found in mkviii/hardware/example
*/

#include "blinky.h"

int main(void) {
    HAL_Init();
    SystemClockConfig();
    GpioInit();
    TimerInit();

    while (1) {
        // everything driven by TIM6 interrupt
    }
    return 0;
}

