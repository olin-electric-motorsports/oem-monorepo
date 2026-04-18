#include "adc.h"
#include "vcu.h"

static oem_adc_config_t s_throttle_l_adc = {
  .adc_instance = VCU_THROTTLE_L_ADC_INSTANCE,
  .port = VCU_THROTTLE_L_ADC_PORT,
  .pin = VCU_THROTTLE_L_ADC_PIN,
  .channel = VCU_THROTTLE_L_ADC_CHANNEL,
  .sample_time = VCU_THROTTLE_ADC_SAMPLE_TIME,
};

static oem_adc_config_t s_throttle_r_adc = {
  .adc_instance = VCU_THROTTLE_R_ADC_INSTANCE,
  .port = VCU_THROTTLE_R_ADC_PORT,
  .pin = VCU_THROTTLE_R_ADC_PIN,
  .channel = VCU_THROTTLE_R_ADC_CHANNEL,
  .sample_time = VCU_THROTTLE_ADC_SAMPLE_TIME,
};

HAL_StatusTypeDef vcu_adc_init(void)
{
  oem_adc_init(&s_throttle_l_adc);
  oem_adc_init(&s_throttle_r_adc);

  s_hw.hadc_throttle_l = &s_throttle_l_adc;
  s_hw.hadc_throttle_r = &s_throttle_r_adc;

  return HAL_OK;
}
