#include "vehicle/examples/adc/adc_example.h"
#include "common/adc/adc.h"

oem_adc_config_t throttle_sensor = {
    .adc_instance = ADC2,
    .port = GPIOA, 
    .pin = GPIO_PIN_4,
    .channel = ADC_CHANNEL_17,
    .sample_time = ADC_SAMPLETIME_47CYCLES_5 
};
// adc_pin_config_t brake_temp_sensor = {
//     .adc_instance = ADC1, 
//     .port = GPIOB, 
//     .pin = GPIO_PIN_11, 
//     .channel = ADC_CHANNEL_14, 
//     .sample_time = ADC_SAMPLETIME_247CYCLES_5 // Slower sample for weak thermistor
// };

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  oem_adc_init(&throttle_sensor);
//   oem_adc_init(&brake_temp_sensor);

  volatile uint16_t throttle_val = 0;

    while (1) {
        // 3. Read the data effortlessly
        throttle_val = oem_adc_read(&throttle_sensor);


        (void)throttle_val;

        HAL_Delay(10);
    }
  

//   uint32_t adc_result = 0;
//   uint32_t poll_timeout = 100;

//   while (1) {
//     if (adc_start_convert(&hadc1, ANALOG_TEST_INPUT_CHANNEL, &PA3_config) != HAL_OK){
//         // Handle error, maybe toggle a debug LED
//     }
    
//     if (adc_poll_complete(&hadc1, &adc_result, poll_timeout) != HAL_OK){
//         adc_result = 0; // Reset result
//     }
  
    
//     if (adc_result > 200) { // 200 --> 160mV
//         HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_SET);
//     } 
//     else {
//         HAL_GPIO_WritePin(BLINKY_LED_GPIO_PORT, BLINKY_LED_PIN, GPIO_PIN_RESET);
//     }
    
//   }
    
  return 0;
}
