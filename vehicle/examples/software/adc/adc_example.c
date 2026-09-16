#include "adc_example.h"



int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  oem_adc_init(&throttle_sensor);

  volatile uint16_t throttle_val = 0;

    while (1) {
        //Read the data from the ADC pin
        throttle_val = oem_adc_read(&throttle_sensor);


       
        if (throttle_val > 200) { // 200 approx 160mV
            HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_SET);
        } 
        else {
            HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_RESET);
        }


        HAL_Delay(10);
    }
  

    
  return 0;
}
