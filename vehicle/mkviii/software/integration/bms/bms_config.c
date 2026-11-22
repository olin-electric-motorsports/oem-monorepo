#include "bms_config.h"
#include "stm32g4xx_hal_tim.h"

TIM_HandleTypeDef htim2; // Fan PWM

/*
 * PIN DEFINITIONS
 */
// GPIO Setup
void GpioInit(void){
    // enable clocks for ports we are using
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // generic structure
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // port A outputs: relay drive and LEDs
    HAL_GPIO_WritePin(GPIOA, BMS_RELAY_DRIVE_PIN | HEARTBEAT_PIN | ERROR_LED_PIN | CSC_COMMS_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = BMS_RELAY_DRIVE_PIN | HEARTBEAT_PIN | ERROR_LED_PIN | CSC_COMMS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // configure port A inputs
    // BSPD_CURRENT_THRESH (PA9), driven by U7 so no pull needed
    GPIO_InitStruct.Pin = BSPD_CURRENT_THRESH_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSPD_CURRENT_THRESH_PORT, &GPIO_InitStruct);

    // Fan PWM (PA0) is TIM2_CH1
    GPIO_InitStruct.Pin = FAN_PWM_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(FAN_PWM_PORT, &GPIO_InitStruct);

    // ISO_CS (PA15) and the SPI1 pins (PB3-PB5) are set up by oem_spi_init()
}

// timer initialization
void TimerInit(void) {
    // --- Timer 2: 25kHz fan PWM on CH1 ---
    // The 10ms main loop runs off HAL_GetTick() instead of a timer interrupt
    // because common/startup.c's vector table has no peripheral IRQ entries.
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (HAL_RCC_GetPCLK1Freq() / FAN_PWM_FREQ_HZ) - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
}

// FDCAN1 on PA11 (RX) / PA12 (TX). Called by HAL_FDCAN_Init() from the
// generated can_init_bms(). The generated bit timing assumes the FDCAN kernel
// clock is the 16MHz HSE, which is the reset default.
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan) {
    if (hfdcan->Instance == FDCAN1) {
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

// Defined in the generated can_api.c
extern FDCAN_HandleTypeDef hfdcan1;

void can_wait_for_tx_slot(void) {
    uint32_t start = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0
           && (HAL_GetTick() - start) < 2) {
    }
}

// interupt handlers
void SysTick_Handler(void) {
    HAL_IncTick(); // Critical for HAL_Delay() and HAL_GetTick() to work!
}

// ISO_CS is PA15, SPI1 is PB3 (SCK), PB4 (MISO), PB5 (MOSI).
// The LTC6820 has POL = PHA = 1 (SPI mode 3, matches common/spi) and SLOW = 0,
// so SCK must stay at or below 1MHz: 64MHz / 128 = 500kHz.
oem_spi_config_t bms_spi = {
    .spi_instance = SPI1,
    .cs_port = ISO_CS_PORT,
    .cs_pin = ISO_CS_PIN,
    .baud_prescaler = SPI_BAUDRATEPRESCALER_128
};

// ADC CONFIGURATION

// Current sensor output, divided down by R2/R4, on PA4 (ADC2_IN17)
oem_adc_config_t current_sense_vout = {
    .adc_instance = ADC2,
    .port = VOUT_CURRENT_SENSE_PORT,
    .pin = VOUT_CURRENT_SENSE_PIN,
    .channel = ADC_CHANNEL_17,
    .sample_time = ADC_SAMPLETIME_47CYCLES_5
};

cell_data_s cell_data = {0};
