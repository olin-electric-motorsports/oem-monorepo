#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common/adc/adc.h"
#include "stm32g441xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"


/*
 * Maps pin to ADC channel only (ADC handle specifies the peripheral)
 */
typedef enum {
    ADC_PA0_IN1  = ADC_CHANNEL_1,   /* ADC1 or ADC2 */
    ADC_PA1_IN2  = ADC_CHANNEL_2,   /* ADC1 or ADC2 */
    ADC_PA2_IN3  = ADC_CHANNEL_3,   /* ADC1 only */
    ADC_PA3_IN4  = ADC_CHANNEL_4,   /* ADC1 only */
    ADC_PA4_IN17 = ADC_CHANNEL_17,  /* ADC2 only */
    ADC_PA5_IN13 = ADC_CHANNEL_13,  /* ADC2 only */
    ADC_PA6_IN3  = ADC_CHANNEL_3,   /* ADC2 only — same channel as PA2 on ADC1 */
    ADC_PA7_IN4  = ADC_CHANNEL_4,   /* ADC2 only — same channel as PA3 on ADC1 */
    ADC_PB0_IN15 = ADC_CHANNEL_15,  /* ADC1 only */
} adc_pin_e;

HAL_StatusTypeDef vcu_adc_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

