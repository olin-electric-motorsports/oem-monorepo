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

static oem_adc_config_t s_brake_press_sense_adc = {
  .adc_instance = VCU_BRAKE_PRESSURE_SENSE_ADC_INSTANCE,
  .port = VCU_BRAKE_PRESSURE_SENSE_ADC_PORT,
  .pin = VCU_BRAKE_PRESSURE_SENSE_ADC_PIN,
  .channel = VCU_BRAKE_PRESSURE_SENSE_ADC_CHANNEL,
  .sample_time = VCU_THROTTLE_ADC_SAMPLE_TIME,
};

static oem_adc_config_t s_brake_press_sense_ftr_adc = {
  .adc_instance = VCU_BRAKE_PRESSURE_SENSE_FILTERED_ADC_INSTANCE,
  .port = VCU_BRAKE_PRESSURE_SENSE_FILTERED_ADC_PORT,
  .pin = VCU_BRAKE_PRESSURE_SENSE_FILTERED_ADC_PIN,
  .channel = VCU_BRAKE_PRESSURE_SENSE_FILTERED_ADC_CHANNEL,
  .sample_time = VCU_THROTTLE_ADC_SAMPLE_TIME,
};

static oem_adc_config_t s_rc_timer_status_adc = {
  .adc_instance = VCU_RC_TIMER_STATUS_ADC_INSTANCE,
  .port = VCU_RC_TIMER_STATUS_ADC_PORT,
  .pin = VCU_RC_TIMER_STATUS_ADC_PIN,
  .channel = VCU_RC_TIMER_STATUS_ADC_CHANNEL,
  .sample_time = VCU_THROTTLE_ADC_SAMPLE_TIME,
};

HAL_StatusTypeDef vcu_adc_init(void)
{
  // Throttle
  oem_adc_init(&s_throttle_l_adc);
  oem_adc_init(&s_throttle_r_adc);

  s_hw.hadc_throttle_l = &s_throttle_l_adc;
  s_hw.hadc_throttle_r = &s_throttle_r_adc;

  // BSPD
  oem_adc_init(&s_brake_press_sense_adc);
  oem_adc_init(&s_brake_press_sense_ftr_adc);
  oem_adc_init(&s_rc_timer_status_adc);

  s_hw.hadc_brake_press_sense = &s_brake_press_sense_adc;
  s_hw.hadc_brake_press_sense_ftr = &s_brake_press_sense_ftr_adc;
  s_hw.hadc_rc_timer_status = &s_rc_timer_status_adc;

  return HAL_OK;
}
