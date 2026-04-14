#include "brake_temp.h"



// Initialize Variables and Functions
// Variable Declarations
float bit_voltage = 0;
float measured_voltage = 0;
int temperature = 0;
int delayval=500;


// Blinky


void init_peripherals(void) {
 HAL_Init();
 SystemClockConfig();
 GpioInit();
 oem_adc_init(&brake_temp_sensor);
 }

// Trigger Debug LED
void update_debug(void) {
 HAL_GPIO_TogglePin(GPIOA, DEBUG_LED);
 delayval = 100;
}


void get_temperature(void) {
  //bit_voltage = oem_adc_read(&brake_temp_sensor);
  bit_voltage = 3000;
  HAL_Delay(10);
  measured_voltage = (bit_voltage / 4095.0) * 3.3;
  temperature = (measured_voltage - 0.5) * 200;
}


int main(void) {
 init_peripherals();
 while (1)
 {
   HAL_Delay(delayval); // Fast"
  HAL_GPIO_TogglePin(GPIOA, HEARTBEAT_LED);
  get_temperature();
  delayval = 500;
  if (measured_voltage >= 3.0) {
    update_debug();
  }
 }
 return 0;
}







