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
 */
oem_spi_config_t lcd_spi = {
    .spi_instance = SPI1,
    .cs_port = LCD_CS_GPIO_Port,
    .cs_pin = LCD_CS_Pin,
    .baud_prescaler = SPI_BAUDRATEPRESCALER_16
};

oem_spi_config_t charger_spi = {
    .spi_instance = SPI2,
    .cs_port = CHARGER_CAN_CS_GPIO_Port,
    .cs_pin = CHARGER_CAN_CS_Pin,
    .baud_prescaler = SPI_BAUDRATEPRESCALER_64
};

/*
 * NOTE:
 * If your STM32 MCP25625 wrapper still requires specific gpio_t objects for
 * rst/stby, replace these placeholders with your real wrapper-compatible values.
 */
MCP25625 charger_CAN_converter = {
    .speed = 250000,
    .spi_struct = &charger_spi,
    .rst = &(gpio_t){0},
    .stby = &(gpio_t){0}
};

void GpioInit(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*
     * LEDs
     */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*
     * LCD control pins
     */
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = LCD_CS_Pin | LCD_DC_Pin | LCD_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_BL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

    /*
     * MCP25625 control pins
     */
    HAL_GPIO_WritePin(CHARGER_CAN_CS_GPIO_Port, CHARGER_CAN_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CHARGER_CAN_RST_GPIO_Port, CHARGER_CAN_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CHARGER_CAN_STBY_GPIO_Port, CHARGER_CAN_STBY_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = CHARGER_CAN_CS_Pin | CHARGER_CAN_RST_Pin | CHARGER_CAN_STBY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*
     * SPI1 pins for LCD
     */
    GPIO_InitStruct.Pin = LCD_SPI_SCK_Pin | LCD_SPI_MISO_Pin | LCD_SPI_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*
     * SPI2 pins for charger CAN controller
     */
    GPIO_InitStruct.Pin = CHG_SPI_SCK_Pin | CHG_SPI_MISO_Pin | CHG_SPI_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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