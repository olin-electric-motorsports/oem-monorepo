#include "gpio.h"

static void gpio_init(GPIO_TypeDef* port,
                      uint16_t pin,
                      uint32_t mode,
                      uint32_t pull,
                      uint32_t speed);

HAL_StatusTypeDef vcu_gpio_init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure BRAKE_LL_LED*/
  gpio_init(
    VCU_BRAKE_LL_LED_GPIO_PORT,
    VCU_BRAKE_LL_LED_GPIO_PIN,
    GPIO_MODE_OUTPUT_PP,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.brake_ll_led_port = VCU_BRAKE_LL_LED_GPIO_PORT;
  s_hw.brake_ll_led_pin = VCU_BRAKE_LL_LED_GPIO_PIN;

  /*Configure MOTOR_5KW_LED*/
  gpio_init(
    VCU_MOTOR_5KW_LED_GPIO_PORT,
    VCU_MOTOR_5KW_LED_GPIO_PIN,
    GPIO_MODE_OUTPUT_PP,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.motor_5kw_led_port = VCU_MOTOR_5KW_LED_GPIO_PORT;
  s_hw.motor_5kw_led_pin = VCU_MOTOR_5KW_LED_GPIO_PIN;

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

  /*Configure BSPD_LL*/
  gpio_init(
    VCU_BSPD_LL_GPIO_PORT,
    VCU_BSPD_LL_GPIO_PIN, 
    GPIO_MODE_INPUT,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.bspd_ll_port = VCU_BSPD_LL_GPIO_PORT;
  s_hw.bspd_ll_pin = VCU_BSPD_LL_GPIO_PIN;

  /*Configure BRAKELIGHT_LL*/
  gpio_init(
    VCU_BRAKELIGHT_LL_GPIO_PORT,
    VCU_BRAKELIGHT_LL_GPIO_PIN, 
    GPIO_MODE_INPUT,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.brakelight_ll_port = VCU_BRAKELIGHT_LL_GPIO_PORT;
  s_hw.brakelight_ll_pin = VCU_BRAKELIGHT_LL_GPIO_PIN;

  /*Configure MOTOR_CURRENT_SENSE*/
  gpio_init(
    VCU_MOTOR_CURRENT_SENSE_GPIO_PORT,
    VCU_MOTOR_CURRENT_SENSE_GPIO_PIN, 
    GPIO_MODE_INPUT,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.motor_current_sense_port = VCU_MOTOR_CURRENT_SENSE_GPIO_PORT;
  s_hw.motor_current_sense_pin = VCU_MOTOR_CURRENT_SENSE_GPIO_PIN;

  /*Configure BSPD_SHUTDOWN_SENSE*/
  gpio_init(
    VCU_BSPD_SHUTDOWN_SENSE_GPIO_PORT,
    VCU_BSPD_SHUTDOWN_SENSE_GPIO_PIN, 
    GPIO_MODE_INPUT,
    GPIO_NOPULL,
    GPIO_SPEED_FREQ_LOW
  );
  s_hw.bspd_shutdown_sense_port = VCU_BSPD_SHUTDOWN_SENSE_GPIO_PORT;
  s_hw.bspd_shutdown_sense_pin = VCU_BSPD_SHUTDOWN_SENSE_GPIO_PIN;
  
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
