/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "vcu_config.h"
#include "stm32g441xx.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure SS_IS*/
  vcu_gpio_init(
    VCU_SS_IS_GPIO_PORT,
    VCU_SS_IS_GPIO_PIN,
    GPIO_MODE_INPUT,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );

  /*Configure ERROR_LED*/
  vcu_gpio_init(
    VCU_ERROR_LED_GPIO_PORT,
    VCU_ERROR_LED_GPIO_PIN, 
    GPIO_MODE_OUTPUT_PP,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );

  /*Configure ERROR_LED*/
  vcu_gpio_init(
    VCU_HEARTBEAT_LED_GPIO_PORT,
    VCU_HEARTBEAT_LED_GPIO_PIN, 
    GPIO_MODE_OUTPUT_PP,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

// Helper functions

/* A GPIO is considered usable only if both port and pin are configured. */
static inline bool vcu_gpio_is_configured(GPIO_TypeDef* port, uint16_t pin) {
    return (port != NULL) && (pin != 0u); // Can port be valid when equal to 0u????
}

static inline void vcu_gpio_init(GPIO_TypeDef* port, 
                                    uint16_t pin, 
                                    uint32_t mode, 
                                    uint32_t pull, 
                                    uint32_t speed) {
    if (port == NULL || pin == 0) {
        return;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = speed;

    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // Add code for handler
}

static inline bool vcu_bool_from_pin(uint16_t port,
                                     uint16_t pin,
                                     GPIO_PinState active_state,
                                     bool default_if_unconfigured) {
    if (!vcu_gpio_is_configured(port, pin)) {
        return default_if_unconfigured;
    }

    return HAL_GPIO_ReadPin(port, pin) == active_state;
}

/*
 * Write a logical state to a GPIO with active-high/active-low support.
 * "active" is the logical command; active_state defines the physical level.
 */
static inline void vcu_write_pin(uint16_t port,
                                 uint16_t pin,
                                 GPIO_PinState active_state,
                                 bool active) {
    GPIO_PinState pin_state = GPIO_PIN_RESET;
    if (!vcu_gpio_is_configured(port, pin)) {
        return;
    }

    pin_state = active ? active_state
                       : (active_state == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(port, pin, pin_state);
}