#pragma once

// #include "common/adc/api.h"
// #include "common/gpio/api.h"
// #include "common/gpio/pin_defs.h"
// #include "common/spi/api.h"
// #include "common/timer/api.h"
// #include "vehicle/common/ltc6811/ltc681x.h"

#include "common/adc/adc.h"
#include "common/gpio/adc.h"
#include "common/spi/api.h"
#include "common/timer/api.h"
#include "vehicle/common/ltc6811/ltc681x.h"

// Pin Defs
// Port A
#define FAN_PWM_PIN              GPIO_PIN_0
#define FAN_PWM_PORT             GPIOA
#define PRE_DIS_TEMP_1_PIN       GPIO_PIN_1
#define PRE_DIS_TEMP_1_PORT      GPIOA
#define PRE_DIS_TEMP_2_PIN       GPIO_PIN_2
#define PRE_DIS_TEMP_2_PORT      GPIOA
#define VOUT_CURRENT_SENSE_PIN   GPIO_PIN_5
#define VOUT_CURRENT_SENSE_PORT  GPIOA
#define HEARTBEAT_PIN            GPIO_PIN_7 
#define HEARTBEAT_PORT           GPIOA
#define BMS_RELAY_DRIVE_PIN      GPIO_PIN_8
#define BMS_RELAY_DRIVE_PORT     GPIOA
#define BSPD_CURRENT_THRESH_PIN  GPIO_PIN_9
#define BSPD_CURRENT_THRESH_PORT GPIOA

// Port B
#define ERROR_LED_PIN            GPIO_PIN_0
#define ERROR_LED_PORT           GPIOB
#define CSC_COMMS_PIN            GPIO_PIN_1
#define CSC_COMMS_PORT           GPIOB
#define ISO_CS_PIN               GPIO_PIN_3
#define ISO_CS_PORT              GPIOB
#define ISO_SCK_PIN              GPIO_PIN_13 // Schematic says SCK is PB13
#define ISO_MISO_PIN             GPIO_PIN_14 // Schematic says MISO is PB14
#define ISO_MOSI_PIN             GPIO_PIN_15 // Schematic says MOSI is PB15

/*
 * Macros
 */
#define NUM_ICS          1

#define MAX_EXTRANEOUS_TEMPERATURES 3 // causing isssssueeeess... maybe set to higher value - ian W 9/14/2024
#define MAX_PEC_ERROR_COUNT \
    32 // copied from mkv where it still says arbitrary...

// copied from MKV - need to be updated
#define FAKE_DA_FIRE_BODGE                 ((int16_t)700) // 140 degC
#define OVERTEMPERATURE_THRESHOLD          ((int16_t)5725) // 60 degC
#define SOFT_OVERTEMPERATURE_THRESHOLD     ((int16_t)8892) // 45 degC
#define SOFT_OVERTEMPERATURE_THRESHOLD_LOW ((int16_t)11708) // 35 degC
#define UNDERTEMPERATURE_THRESHOLD         ((int16_t)27605) // -20 degC

#define OVERVOLTAGE_THRESHOLD          (42000) // 3.95V (max pack voltage (402.9V / [17 * 6] cells))
#define UNDERVOLTAGE_THRESHOLD         (25000) // 2.5V (Li-ion chemistry minimum)
#define SEGMENT_OVERVOLTAGE_THRESHOLD  (714000) // 71.4V (4.2v * 17 cells)
#define SEGMENT_UNDERVOLTAGE_THRESHOLD (442000) // 44.2V (4.6 * 17 cells)

#define CURRENT_THRESH (12000) // 120A (peak current) * 100cA/A = 12,000 centiAmps

// Cell balancing config
// Insert here

#define DIE_OVERTEMPERATURE_THRESHOLD (0)

// Number (out of 18) voltage channels not used. Unused channels are bridged
// and read as 0V
#define NUM_UNUSED_VOLTAGE_CHANNELS 1

/*
 * PIN & PERIPHERAL DEFINITIONS
 */

// Outputs
// extern gpio_t BMS_RELAY_LSD;
// extern gpio_t COOLING_PUMP_LSD;
// extern gpio_t COOLING_PUMP_PWM;
// extern gpio_t SPI_CS;
// extern gpio_t CHARGE_ENABLE_IN;
// extern gpio_t CHARGE_ENABLE_OUT;

// extern gpio_t DEBUG_LED_1;
// extern gpio_t DEBUG_LED_2;

// // Inputs
// extern gpio_t BSPD_CURRENT_THRESH;
// extern adc_pin_e PRE_DIS_TEMP_1;
// extern adc_pin_e PRE_DIS_TEMP_2;
// extern adc_pin_e PRE_DIS_TEMP_3;
// extern adc_pin_e CURRENT_SENSE_VOUT;
void GpioInit(void);

// 04/13/26
//SPI Peripheral Configuration
extern oem_spi_config_t bms_spi;

/** 04/18/26
 * ADC Configurations
 * Mapped to the STM32G4 ADC
 */
extern oem_adc_config_t pre_dis_temp_1;      // Mapped to PA1
extern oem_adc_config_t pre_dis_temp_2;      // Mapped to PA2
extern oem_adc_config_t current_sense_vout; // Mapped to PA5

void timer0_isr(void);
extern timer_cfg_s timer0_cfg;
extern timer_cfg_s timer1_cfg;
extern spi_cfg_s spi_cfg;

// A little bit of a hacky, truncated version of the cell_asic
// struct defined by the LTC681X library
typedef struct {
    ic_register config;
    cv cells;
} cell_asic_trunc;

typedef struct {
    cell_asic_trunc cells[NUM_ICS];
} cell_data_s;

extern cell_data_s cell_data;
