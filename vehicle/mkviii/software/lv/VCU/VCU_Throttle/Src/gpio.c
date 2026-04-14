#include "gpio.h"

HAL_StatusTypeDef vcu_gpio_init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure SS_IS*/
  gpio_init(
    VCU_SS_IS_GPIO_PORT,
    VCU_SS_IS_GPIO_PIN,
    GPIO_MODE_INPUT,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.ss_is_port = VCU_SS_IS_GPIO_PORT;
  s_hw.ss_is_pin = VCU_SS_IS_GPIO_PIN;

  /*Configure ERROR_LED*/
  gpio_init(
    VCU_ERROR_LED_GPIO_PORT,
    VCU_ERROR_LED_GPIO_PIN, 
    GPIO_MODE_OUTPUT_PP,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.error_led_port = VCU_ERROR_LED_GPIO_PORT;
  s_hw.error_led_pin = VCU_ERROR_LED_GPIO_PIN;

  /*Configure HEARTBEAT_LED*/
  gpio_init(
    VCU_HEARTBEAT_LED_GPIO_PORT,
    VCU_HEARTBEAT_LED_GPIO_PIN, 
    GPIO_MODE_OUTPUT_PP,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.heartbeat_led_port = VCU_HEARTBEAT_LED_GPIO_PORT;
  s_hw.heartbeat_led_pin = VCU_HEARTBEAT_LED_GPIO_PIN;
  
  return HAL_OK;
}

// Helper functions

static void gpio_init(GPIO_TypeDef* port,
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
}
