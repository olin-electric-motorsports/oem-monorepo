#include "bms_config.h"

#include "common/adc/api.h"
#include "common/gpio/api.h"

// #include "libs/adc/api.h"
// #include "libs/gpio/api.h"
// #include "libs/gpio/pin_defs.h"

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


    HAL_GPIO_WritePin(GPIOA, FAN_PWM_PIN | BMS_RELAY_DRIVE_PIN | HEARTBEAT_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = FAN_PWM_PIN | BMS_RELAY_DRIVE_PIN | HEARTBEAT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // configure port A inputs
    // BSPD_CURRENT_THRESH (PA9) 
    GPIO_InitStruct.Pin = BSPD_CURRENT_THRESH_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
    GPIO_InitStruct.Pull = GPIO_PULLUP; 
    HAL_GPIO_Init(BSPD_CURRENT_THRESH_PORT, &GPIO_InitStruct);

    // port B outputs
    HAL_GPIO_WritePin(GPIOB, ERROR_LED_PIN | CSC_COMMS_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = ERROR_LED_PIN | CSC_COMMS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 04/13/26
    // port B spi
    // ISO_CS (PB3) - Needs to start HIGH
    HAL_GPIO_WritePin(ISO_CS_PORT, ISO_CS_PIN, GPIO_PIN_SET); 
    GPIO_InitStruct.Pin = ISO_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(ISO_CS_PORT, &GPIO_InitStruct);

    // port B spi peripheral pins
    // handles clock and data lines
    // ISO_SCK (PB3), ISO_MISO (PB4), ISO_MOSI (PB5)
    // must be GPIO_MODE_AF_PP for the SPI hardware to work 
    GPIO_InitStruct.Pin = ISO_SCK_PIN | ISO_MISO_PIN | ISO_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; 
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


// Based on schematic: ISO_CS is PB3, SPI1 is on Port B 
oem_spi_config_t bms_spi = {
    .spi_instance = SPI1, 
    .cs_port = ISO_CS_PORT,               
    .cs_pin = ISO_CS_PIN,           
    .baud_prescaler = SPI_BAUDRATEPRESCALER_64 
};

// ADC CONFIGURATION

// Pre-charge/Discharge temp readings are on PA1 (ADC1_IN2) and PA2 (ADC1_IN3) 
oem_adc_config_t pre_dis_temp_1 = {
    .adc_instance = ADC1,
    .port = PRE_DIS_TEMP_1_PORT,
    .pin = PRE_DIS_TEMP_1_PIN,
    .channel = ADC_CHANNEL_2,
    .sample_time = ADC_SAMPLETIME_47CYCLES_5
};

oem_adc_config_t pre_dis_temp_2 = {
    .adc_instance = ADC1,
    .port = PRE_DIS_TEMP_2_PORT,
    .pin = PRE_DIS_TEMP_2_PIN,
    .channel = ADC_CHANNEL_3,
    .sample_time = ADC_SAMPLETIME_47CYCLES_5
};

oem_adc_config_t current_sense_vout = {
    .adc_instance = ADC2,
    .port = VOUT_CURRENT_SENSE_PORT,
    .pin = VOUT_CURRENT_SENSE_PIN,
    .channel = ADC_CHANNEL_13,
    .sample_time = ADC_SAMPLETIME_47CYCLES_5
};

cell_data_s cell_data = {0};

// new SPI different SPI ?!?!?!

// OLD bms_config.c CODE!!
// // Outputs
// gpio_t BMS_RELAY_LSD = PC7;
// gpio_t COOLING_PUMP_LSD = PC0;
// gpio_t COOLING_PUMP_PWM = PC1;
// gpio_t SPI_CS = PB6;
// gpio_t CHARGE_ENABLE_IN = PB4;
// gpio_t CHARGE_ENABLE_OUT = PB3;

// gpio_t DEBUG_LED_1 = PD6;
// gpio_t DEBUG_LED_2 = PD7;

// // Inputs
// gpio_t BSPD_CURRENT_THRESH = PB2;
// adc_pin_e PRE_DIS_TEMP_1 = ADC8;
// adc_pin_e PRE_DIS_TEMP_2 = ADC9;
// adc_pin_e PRE_DIS_TEMP_3 = ADC10;
// adc_pin_e CURRENT_SENSE_VOUT = ADC2;

// /*
//  * TIMERS
//  */

// // 10ms main loop timer
// timer_cfg_s timer0_cfg = {
//     .timer = TIMER0,
//     .timer0_mode = TIMER0_MODE_CTC,
//     .prescalar = CLKIO_DIV_1024,
//     .channel_a = {
//         .channel = CHANNEL_A,
//         .output_compare_match = 0x27,
//         .pin_behavior = DISCONNECTED,
//         .interrupt_enable = true,
//         .interrupt_callback = timer0_isr,
//     },
// };

// // Fan PWM (25kHz) TODO: may need to update value
// timer_cfg_s timer1_cfg = {
//     .timer = TIMER1,
//     .timer1_mode = TIMER1_MODE_PHASE_CORRECT_PWM_10_BIT,
//     .prescalar = CLKIO_DIV_8,
//     .channel_a = {
//         .output_compare_match = 0x50,
//     },
//     .channel_b = {
//         .channel = CHANNEL_B,
//         .pin_behavior = CLEAR,
//         .output_compare_match = 0,
//         .interrupt_enable = false,
//     },
// };

// /*
//  * SPI
//  */
// spi_cfg_s spi_cfg = {
//     .interrupt_enable = false,
//     .data_order = MSB,
//     .mode = MAIN,
//     .polarity = FALLING_RISING,
//     .phase = SETUP_SAMPLE,
//     .clock_rate = F_OSC_DIV_4,
//     .cs_pin = &SPI_CS,
// };




// BLINKY CODE :3
// #include "bms.h"

// // Function Implementations
// void GpioInit(void) {
//     // 1. Enable GPIO Clocks
//     //  enable Port A because the Heartbeat (PA7) is on Port A
//     __HAL_RCC_GPIOA_CLK_ENABLE();

//     GPIO_InitTypeDef GPIO_InitStruct = {0};

//     // 2. Initialize Heartbeat Pin
//     // Ensure the pin starts in a known state (Low/Reset)
//     HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_RESET);

//     GPIO_InitStruct.Pin = HEARTBEAT_Pin;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // Standard Push-Pull Output
//     GPIO_InitStruct.Pull = GPIO_NOPULL;           // No internal pull-up/down
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Low speed is fine for a blinker
    
//     // Apply settings to the hardware
//     HAL_GPIO_Init(HEARTBEAT_GPIO_Port, &GPIO_InitStruct);
// }

// void SysTick_Handler(void) {
//     // This increments the global tick counter every 1ms
//     // This is required for HAL_Delay() to work
//     HAL_IncTick();
// }