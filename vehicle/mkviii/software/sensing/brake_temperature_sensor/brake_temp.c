#include "brake_temp.h"

static void GpioInit(void);
void SystemClockConfig(void);
// void initialize(void) :
 
  //  gpio_set_mode(HEARTBEAT_LED, OUTPUT); // Heartbeat LED
  //  


//void get_temperature(void)
  // Voltage = analogRead(IR_Output)


// Blinky
int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();

  while(1)
  {
    count = count + 1;
    if count >= 500:
      HAL_GPIO_TogglePin(PA1_GPIO_Port,PA8_Pin);
      count = 0;
  }

}

