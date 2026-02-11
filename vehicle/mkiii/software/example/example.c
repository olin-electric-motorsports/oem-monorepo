#include "example.h"

static void GpioInit(void);

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();

  while (1) {
    HAL_Delay(1000); // Fast
    HAL_GPIO_TogglePin(PA1_GPIO_Port, PA1_Pin);
  }
  return 0;
}