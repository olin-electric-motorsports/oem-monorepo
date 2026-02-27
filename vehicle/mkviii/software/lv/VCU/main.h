#ifndef VEHICLE_MKVII_SOFTWARE_LV_VCU_MAIN_H_
#define VEHICLE_MKVII_SOFTWARE_LV_VCU_MAIN_H_

#include "libs/adc/api.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
} HAL_StatusTypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1,
} GPIO_PinState;

typedef struct {
    uint8_t port_reg;
    uint8_t pin_reg;
} GPIO_TypeDef;

typedef struct {
    adc_pin_e channel;
    uint16_t sample;
    bool started;
} ADC_HandleTypeDef;

typedef struct {
    uint8_t reserved;
} IWDG_HandleTypeDef;

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t timeout_ms);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef* hiwdg);

#endif /* VEHICLE_MKVII_SOFTWARE_LV_VCU_MAIN_H_ */
