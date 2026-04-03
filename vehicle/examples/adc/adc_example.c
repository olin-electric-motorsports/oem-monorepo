#include "vehicle/examples/adc/adc_example.h"
#include "common/adc/adc.h"



int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  AdcInit();
  

  uint32_t adc_result = 0;
  uint32_t poll_timeout = 100;

  while (1) {
    if (adc_start_convert(&hadc1, ANALOG_TEST_INPUT_CHANNEL, &PA3_config) != HAL_OK){
        // Handle error, maybe toggle a debug LED
    }
    
    if (adc_poll_complete(&hadc1, &adc_result, poll_timeout) != HAL_OK){
        adc_result = 0; // Reset result
    }
  
    
    if (adc_result > 200) { // 200 --> 160mV
        HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_SET);
    } 
    else {
        HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_RESET);
    }
    
  }
    
  return 0;
}
