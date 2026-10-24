#include "stm32g4xx_hal.h"

/* Board-specific setup called by HAL_FDCAN_Init() before CAN is configured. */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan) {
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    /* The VCU's 16 MHz HSE gives 500 kbit/s with the generated 32-TQ timing. */
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_HSE);
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA11 is CAN RX and PA12 is CAN TX, connected to the onboard transceiver. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
}
