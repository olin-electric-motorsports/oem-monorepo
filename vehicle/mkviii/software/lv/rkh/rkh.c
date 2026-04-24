#include "rkh.h"
#include "can_api.h"
// This file is specifically meant for the STM32G441 dev board that was created and can be found in mkviii/hardware/example
// static void GpioInit(void);

uint64_t my_var = 0;

void SystemClockConfig(void);

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();

  // oem_adc_init(&throttle_right);
  if (can_init_rkh() != 0){
    while(1);
  }

  while (1) {
    HAL_Delay(100); // Fast
    HAL_GPIO_TogglePin(PB5_GPIO_Port, PB5_Pin); 
    HAL_GPIO_TogglePin(PB6_GPIO_Port, PB6_Pin);
    HAL_GPIO_TogglePin(PB7_GPIO_Port, PB7_Pin);

    // can_poll_recieve_all(); // recieves messages
    // my_var = message_1.dummy_signal; // builds a struct

    message_1.dummy_signal = my_var;
    can_send_message_1();
  }
  return 0;
}

