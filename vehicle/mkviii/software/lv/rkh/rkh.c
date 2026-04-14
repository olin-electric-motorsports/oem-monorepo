#include "rkh.h"
// This file is specifically meant for the STM32G441 dev board that was created and can be found in mkviii/hardware/example
// static void GpioInit(void);
void SystemClockConfig(void);

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  oem_adc_init(&throttle_right);
  while (1) {
    HAL_Delay(100); // Fast
    HAL_GPIO_TogglePin(PB5_GPIO_Port, PB5_Pin); 
    HAL_GPIO_TogglePin(PB6_GPIO_Port, PB6_Pin);
    HAL_GPIO_TogglePin(PB7_GPIO_Port, PB7_Pin);
  }
  return 0;
}

