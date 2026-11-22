#pragma once

#include <stdbool.h>       // For bool, true, false
#include <stdint.h>        // For uint8_t, int16_t, etc.
#include "stm32g4xx_hal.h" // The master STM32 HAL header

#include "common/adc/adc.h"
#include "common/spi/spi.h"

#include "vehicle/common/adbms1818/ADBMS181x.h"

// Pin Defs
// Taken from the U10 pads in hardware/integration/bms_core/bms_core.kicad_pcb
// (STM32G441KBT6, LQFP-32). PA1-PA3, PA10, PB0 and PB6-PB8 are unconnected.
// Port A
#define FAN_PWM_PIN              GPIO_PIN_0  // pin 5,  TIM2_CH1 (AF1)
#define FAN_PWM_PORT             GPIOA
#define VOUT_CURRENT_SENSE_PIN   GPIO_PIN_4  // pin 9,  ADC2_IN17, via 5.1k/10k divider
#define VOUT_CURRENT_SENSE_PORT  GPIOA
#define ERROR_LED_PIN            GPIO_PIN_5  // pin 10, red LED D3
#define ERROR_LED_PORT           GPIOA
#define CSC_COMMS_PIN            GPIO_PIN_6  // pin 11, green LED D2
#define CSC_COMMS_PORT           GPIOA
#define HEARTBEAT_PIN            GPIO_PIN_7  // pin 12, amber LED D1
#define HEARTBEAT_PORT           GPIOA
#define BMS_RELAY_DRIVE_PIN      GPIO_PIN_8  // pin 18, gate of Q1
#define BMS_RELAY_DRIVE_PORT     GPIOA
#define BSPD_CURRENT_THRESH_PIN  GPIO_PIN_9  // pin 19, output of U7
#define BSPD_CURRENT_THRESH_PORT GPIOA
#define ISO_CS_PIN               GPIO_PIN_15 // pin 25, LTC6820 ~CS
#define ISO_CS_PORT              GPIOA

// Port B
// ISO_SCK (PB3), ISO_MISO (PB4), ISO_MOSI (PB5) are SPI1 and are configured by
// oem_spi_init() in common/spi. Do not use PB3 as a GPIO.

extern TIM_HandleTypeDef htim2; // Fan PWM
#define FAN_PWM_FREQ_HZ (25000)

void TimerInit(void); // Function prototype to set them up

/*
 * Wait (up to 2ms) for a free slot in the FDCAN TX FIFO. The FIFO only holds 3
 * frames, so call this before each can_send_*() when sending several messages
 * back to back.
 */
void can_wait_for_tx_slot(void);

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

// Cell voltage codes are in units of 100uV
#define OVERVOLTAGE_THRESHOLD          (42000) // 4.2V
#define UNDERVOLTAGE_THRESHOLD         (25000) // 2.5V (Li-ion chemistry minimum)
#define SEGMENT_OVERVOLTAGE_THRESHOLD  (714000) // 71.4V (4.2v * 17 cells)
#define SEGMENT_UNDERVOLTAGE_THRESHOLD (442000) // 44.2V (4.6 * 17 cells)

#define CURRENT_THRESH (12000) // 120A (peak current) * 100cA/A = 12,000 centiAmps

/*
 * Current sensor calibration. The sensor output (0-5V) goes through a
 * 5.1k/10k divider into PA4, so the sensor voltage is V_pin * 15.1 / 10.
 * CURRENT_SENSE_ZERO_MV is the sensor output at 0A. 2773mV is the zero point
 * MKVII used (raw 568 on a 10-bit, 5V ADC) - measure it on this car with no
 * current flowing and update.
 */
#define CURRENT_SENSE_ZERO_MV  (2773)
#define CURRENT_SENSE_MV_PER_A (20)

// Open wire: a cell is open if (pull-up reading - pull-down reading) is above
// this, in 100uV units. 4000 = 400mV, from the ADBMS181x reference driver.
#define OPEN_WIRE_THRESHOLD (4000)

// Cell balancing config
// Balance any cell more than this above the lowest cell (100uV units)
#define BALANCE_THRESHOLD (100) // 10mV
// Discharge timeout written to DCTO in CFGRA. Balancing switches turn off on
// their own if the BMS stops refreshing the config (0x1 = 0.5 minutes)
#define BALANCE_DCTO (0x1)

// ADBMS1818 internal die temperature, ITMP code in 100uV units.
// T(degC) = ITMP * 100uV / 7.6mV - 276, so 85degC -> (85 + 276) * 76 = 27436
#define DIE_OVERTEMPERATURE_THRESHOLD (27436)

// Number (out of 18) voltage channels not used. Unused channels are bridged
// and read as 0V
#define NUM_UNUSED_VOLTAGE_CHANNELS 1

/*
 * PIN & PERIPHERAL DEFINITIONS
 */
void GpioInit(void);

// 04/13/26
//SPI Peripheral Configuration
extern oem_spi_config_t bms_spi;

/** 04/18/26
 * ADC Configurations
 * Mapped to the STM32G4 ADC
 */
extern oem_adc_config_t current_sense_vout; // Mapped to PA4

// A little bit of a hacky, truncated version of the cell_asic
// struct defined by the LTC681X library. Filled by voltage_task() and used for
// cell balancing.
typedef struct {
    ic_register config;
    cv cells;
} cell_asic_trunc;

typedef struct {
    cell_asic_trunc cells[NUM_ICS];
} cell_data_s;

extern cell_data_s cell_data;
