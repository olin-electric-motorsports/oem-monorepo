#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "vcu.h"
#include <stddef.h>
#include "vcu_config.h"
#include "stm32g441xx.h"

HAL_StatusTypeDef vcu_gpio_init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

