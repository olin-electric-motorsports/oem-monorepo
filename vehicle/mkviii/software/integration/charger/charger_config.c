#include "charger.h"

/*
 * =========================
 * Timer handle
 * =========================
 */
TIM_HandleTypeDef htim2;

/*
 * =========================
 * SPI configs
 * =========================
 *
 * LCD:
 *   CS   = PA15
 *   SCK  = PB3
 *   MISO = PB4
 *   MOSI = PB5
 *
 * Charger CAN controller:
 *   CS   = PB12
 *   SCK  = PB13
 *   MISO = PB14
 *   MOSI = PB15
 */
oem_spi_config_t lcd_spi = {
    .spi_instance = SPI1,
    .cs_port = LCD_SPI_CS_GPIO_Port,
    .cs_pin = LCD_SPI_CS_Pin,
    .baud_prescaler = SPI_BAUDRATEPRESCALER_16
};

oem_spi_config_t charger_spi = {
    .spi_instance = SPI2,
    .cs_port = CAN_CTRLR_SPI_CS_GPIO_Port,
    .cs_pin = CAN_CTRLR_SPI_CS_Pin,
    .baud_prescaler = SPI_BAUDRATEPRESCALER_64
};

/*
 * If your STM32 MCP25625 wrapper needs this object, uncomment and adapt as needed.
 *
MCP25625 charger_CAN_converter = {
    .speed = 250000,
    .spi_struct = &charger_spi,
    .rst = &(gpio_t){0},
    .stby = &(gpio_t){0}
};
*/

void GpioInit(void) {
    /*
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    */

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*
     * =========================
     * Status / control output pins
     * =========================
     */
    HAL_GPIO_WritePin(SPI_STATUS_GPIO_Port, SPI_STATUS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CHARGING_STATUS_GPIO_Port, CHARGING_STATUS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FULLY_CHARGED_STATUS_GPIO_Port, FULLY_CHARGED_STATUS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ELCON_READY_GPIO_Port, ELCON_READY_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAR_READY_GPIO_Port, CAR_READY_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CHARGING_ON_OFF_STATUS_GPIO_Port, CHARGING_ON_OFF_STATUS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAN_STATUS_GPIO_Port, CAN_STATUS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_RESET);

    /*
     * Port A outputs
     */
    GPIO_InitStruct.Pin = SPI_STATUS_Pin |
                          CHARGING_STATUS_Pin |
                          FULLY_CHARGED_STATUS_Pin |
                          ELCON_READY_Pin |
                          CAR_READY_Pin |
                          CHARGING_ON_OFF_STATUS_Pin |
                          EVSE_PWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*
     * Port C outputs
     */
    GPIO_InitStruct.Pin = CAN_STATUS_Pin | HEARTBEAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*
     * =========================
     * LCD control pins
     * CS = PA15
     * RST = PB6
     * BLK = PB7
     * =========================
     */


     /*
    HAL_GPIO_WritePin(LCD_SPI_CS_GPIO_Port, LCD_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET);
    */


    /*
     * LCD CS on PA15
     */
    GPIO_InitStruct.Pin = LCD_SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_RST_Pin | LCD_BLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(CAN_CTRLR_SPI_CS_GPIO_Port, CAN_CTRLR_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN_CTRLR_RST_GPIO_Port, CAN_CTRLR_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CHARGER_STBY_GPIO_Port, CHARGER_STBY_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = CAN_CTRLR_RST_Pin | CAN_CTRLR_SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CHARGER_STBY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_SPI_SCK_Pin | LCD_SPI_MISO_Pin | LCD_SPI_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN_CTRLR_SPI_SCK_Pin |
                          CAN_CTRLR_SPI_MISO_Pin |
                          CAN_CTRLR_SPI_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN_RX_Pin | CAN_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void TimerInit(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = CHARGER_MAIN_TIM_INSTANCE;
    htim2.Init.Prescaler = 15999;      /* 16 MHz / 16000 = 1 kHz */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9;             /* 1 kHz / 10 = 100 Hz => 10 ms */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Base_Start_IT(&htim2);
}

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

void SysTick_Handler(void) {
    HAL_IncTick();
}