#include "adc.h"

ADC_HandleTypeDef hadc1;

HAL_StatusTypeDef vcu_adc_init(void)
{
  // How to initialize hadc1?

  if (adc_init(&hadc1) != HAL_OK){
    return HAL_ERROR;
  }

  s_hw.hadc_throttle_l = &hadc1;
  s_hw.throttle_l_channel = VCU_THROTTLE_L_ADC_CHANNEL;

  s_hw.hadc_throttle_r = &hadc1;
  s_hw.throttle_r_channel = VCU_THROTTLE_R_ADC_CHANNEL;

  return HAL_OK;
}
