#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common/adc/adc.h"
#include "vcu_config.h"
#include "stm32g441xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"

HAL_StatusTypeDef vcu_adc_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

