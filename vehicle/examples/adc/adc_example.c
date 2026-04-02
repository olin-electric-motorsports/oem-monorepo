#include "vehicle/examples/adc/adc_example.h"
#include "common/adc/adc.h"
#include <stdio.h>

uint32_t interrupt_result = 0;
uint8_t adc_done = 0;

/*
void adc_callback(void) {
    adc_read_results(&hadc, (uint32_t *)&interrupt_result);
    HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_SET);
    adc_done = 1;
}
*/

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  

  if (adc_init(&hadc) != HAL_OK){
    // Handle error
    //ErrorHandler();
      HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_SET); // Clear pin
  }

  /*
  if (adc_configure_pin(&hadc, ANALOG_TEST_INPUT_PIN) != HAL_OK){
    // Handle error
    ErrorHandler();
  }
  */

  uint32_t adc_result = 0;
  //adc_interrupt_enable(&hadc, ANALOG_TEST_INPUT_CHANNEL, adc_callback);
  while (1) {
    /*
    HAL_Delay(1000); // Fast
    HAL_GPIO_TogglePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN);
    */
    
    
    if (adc_start_convert(&hadc, ANALOG_TEST_INPUT_CHANNEL) != HAL_OK){
      // Handle error
      //ErrorHandler();
      HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_SET);
    }
  
    
    if (adc_poll_complete(&hadc, &adc_result) != HAL_OK){
        // Handle error
        adc_result = 0; // Reset result
        HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_SET); // Clear pin
    }
  
    
    if (adc_result > 200) {           // 200 counts ~ 160mV, well above noise floor
        HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(100);
    } else {
        HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_SET);
    }
    

    /*
    if (adc_done) {
        adc_done = 0;

        if (interrupt_result > 0) {
            HAL_GPIO_TogglePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN);
            HAL_Delay(1000);
            HAL_GPIO_TogglePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN);
            HAL_Delay(1000);
        }

        adc_interrupt_enable(&hadc, ANALOG_TEST_INPUT_CHANNEL, adc_callback);
    }
    */
  }
    
  return 0;
}
