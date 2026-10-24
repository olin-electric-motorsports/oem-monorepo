#include "adc_example.h"



int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  if (oem_adc_init(&throttle_sensor) != HAL_OK) {
    ErrorHandler();
  }

  volatile uint16_t throttle_val = 0;

    while (1) {
        //Read the data from the ADC pin
        uint16_t sample;
        if (oem_adc_read(&throttle_sensor, &sample) != HAL_OK) {
            ErrorHandler();
        }
        throttle_val = sample;


       
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
