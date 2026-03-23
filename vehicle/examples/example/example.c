#include "example.h"
// This file is specifically meant for the STM32G441 dev board that was created and can be found in mkviii/hardware/example
static void GpioInit(void);
void SystemClockConfig(void);

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();

  while (1) {
    HAL_Delay(100); // Fast
    HAL_GPIO_TogglePin(PA1_GPIO_Port, PA1_Pin);
  }
  return 0;
}

