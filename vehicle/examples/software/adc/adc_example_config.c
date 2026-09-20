#include "adc_example.h"


/* 
* GPIO Setup 
*
* Setting up pin PA4 as an anaolog input
*/
void GpioInit(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIO port A
  

  // Blinky LED pin
  HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_RESET); // Clear pin
  GPIO_InitTypeDef GPIO_InitStruct_Blinky_LED = {
    .Pin = BLINKY_LED_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(BLINKY_LED_GPIO_PORT, &GPIO_InitStruct_Blinky_LED);
}

// notice this is outside of GPIO init because it also sets up the ADC peripheral, not just the pin
oem_adc_config_t throttle_sensor = {
    .adc_instance = ADC2, // either ADC1 or ADC2
    .port = GPIOA,  // GPIO port for the pin
    .pin = GPIO_PIN_4, // pin number
    .channel = ADC_CHANNEL_17, // ADC channel corresponding to the pin (check datasheet)
    .sample_time = ADC_SAMPLETIME_47CYCLES_5 //  sampling time (experiment with this for better results)
};

/*So what is hidden in this function that is in the lib

- Clock management for the ADC peripheral and GPIO port
- Setting the GPIO pin to analog mode
- ADC Resolution (12-bit)
- the Clock Prescaler
- Data Alignment (Right-aligned)
- Trigger Mode (Software start)
- Silicon Calibration. STM32 made a function for this
- the hardware routing logic, wheiter we talk to ADC1 or ADC2, since they are separate hardware engines

This is more as an FYI than anything else, but the ADC peripheral is pretty complicated and has a lot of 
configuration options that can be tweaked for better performance 
*/ 



void SysTick_Handler(void) {
  HAL_IncTick();
}