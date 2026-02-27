#include "vehicle/mkviii/software/lv/VCU/main.h"

#include <avr/io.h>

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc) {
    if (hadc == NULL) {
        return HAL_ERROR;
    }

    hadc->sample = adc_read(hadc->channel);
    hadc->started = true;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t timeout_ms) {
    (void)timeout_ms;
    if ((hadc == NULL) || !hadc->started) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc) {
    if (hadc == NULL) {
        return 0u;
    }
    return hadc->sample;
}

HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc) {
    if (hadc == NULL) {
        return HAL_ERROR;
    }
    hadc->started = false;
    return HAL_OK;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin) {
    if ((port == NULL) || (pin == 0u)) {
        return GPIO_PIN_RESET;
    }

    return (_SFR_IO8(port->pin_reg) & (uint8_t)pin) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state) {
    if ((port == NULL) || (pin == 0u)) {
        return;
    }

    if (state == GPIO_PIN_SET) {
        _SFR_IO8(port->port_reg) |= (uint8_t)pin;
    } else {
        _SFR_IO8(port->port_reg) &= (uint8_t)~pin;
    }
}

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef* hiwdg) {
    (void)hiwdg;
    return HAL_OK;
}
