#include "rkh.h"
// This file is specifically meant for the STM32G441 dev board that was created and can be found in mkviii/hardware/example
// static void GpioInit(void);
void SystemClockConfig(void);

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();

  while (1) {
    HAL_Delay(100); // Fast
    HAL_GPIO_TogglePin(PB6_GPIO_Port, PB6_Pin);
  }
  return 0;
}

