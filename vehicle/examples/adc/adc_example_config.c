#include "vehicle/examples/adc/adc_example.h"
#include "common/adc/adc.h"

/* 
* GPIO Setup 
*
* Setting up pin PA3 as an anaolog input
*/
void GpioInit(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIO port A
  

  // Analog test input pin
  HAL_GPIO_WritePin(ANALOG_TEST_INPUT_GPIO_PORT, ANALOG_TEST_INPUT_PIN, GPIO_PIN_RESET); // Clear pin
  GPIO_InitTypeDef GPIO_InitStruct_Analog_Input = {
    .Pin = ANALOG_TEST_INPUT_PIN,
    .Mode = GPIO_MODE_ANALOG,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(ANALOG_TEST_INPUT_GPIO_PORT, &GPIO_InitStruct_Analog_Input);

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


/* 
* ADC Setup 
*
* - Setup clock for ADC
* - Create the config and handle structs for the ADC1 peripheral
* - Create config for PA3
*/
void AdcInit() {
  // Clock setup for ADC1 and ADC2
  __HAL_RCC_ADC12_CLK_ENABLE(); 

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;

  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  // initialize peripheral and calibrate to single ended input
  adc_init(&hadc1);
  adc_calibrate(&hadc1, ADC_SINGLE_ENDED);
}


ADC_HandleTypeDef hadc1 = {
  .Instance = ADC1, // hadc1 is the ADC1 handle
  .Init =  {
    .ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1, 
    .Resolution = ADC_RESOLUTION_12B, 
    .DataAlign = ADC_DATAALIGN_RIGHT,
    .GainCompensation = 0,
    .ScanConvMode = ADC_SCAN_DISABLE,
    .EOCSelection = ADC_EOC_SINGLE_CONV,
    .LowPowerAutoWait = DISABLE,
    .ContinuousConvMode = DISABLE,
    .NbrOfConversion = 1,
    .DiscontinuousConvMode = DISABLE,
    .ExternalTrigConv = ADC_SOFTWARE_START,
    .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
    .DMAContinuousRequests = DISABLE,
    .Overrun = ADC_OVR_DATA_PRESERVED,
    .OversamplingMode = DISABLE,
  }
};


adc_pin_config_t PA3_config = {
  .rank = ADC_REGULAR_RANK_1,
  .sampling_time = ADC_SAMPLETIME_247CYCLES_5,
  .single_diff = ADC_SINGLE_ENDED,
  .offset_number = ADC_OFFSET_NONE,
  .offset = 0,
  .offset_sign = 0,
  .offset_saturation = 0, 
};




void SysTick_Handler(void) {
  HAL_IncTick();
}