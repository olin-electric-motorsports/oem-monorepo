/*
 * BMS Heartbeat firmware for STM32G441KBT6.
 * Specifically targets the PA7 Heartbeat pin on the BMS Micro board.
 */
#include "bms.h"
#include "bms_config.h"
#include "tasks.h"
#include "common/spi/api.h"
#include <string.h>
#include <stdbool.h>

// Flag to tell the main loop 10ms has passed
volatile bool main_loop_ready = false;

#define MAX_TEMPERATURE_FAN (50)
#define MIN_TEMPERATURE_FAN (20)

void cooling_fan_control(uint16_t max_temp) {
    uint32_t duty_cycle;
    uint16_t range = MAX_TEMPERATURE_FAN - MIN_TEMPERATURE_FAN;

    if (max_temp >= MAX_TEMPERATURE_FAN) {
        duty_cycle = 1023; 
    } else if (max_temp < MIN_TEMPERATURE_FAN) {
        duty_cycle = 0;   
    } else {
        duty_cycle = ((uint32_t)(max_temp - MIN_TEMPERATURE_FAN) * 1023) / range;
    }
    // Set PWM duty cycle on Timer 15
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, duty_cycle);
}

int main(void) {
    // 1. Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // 2. Configure the system clock 
    SystemClockConfig();

    // 3. Initialize the Heartbeat GPIO pin
    GpioInit();
    //initialize stm32 timer
    TimerInit();
    // 4. Initialize SPI 04/18/26
    oem_spi_init(&bms_spi);

    

    // Initializing ADCs
    oem_adc_init(&pre_dis_temp_1);
    oem_adc_init(&pre_dis_temp_2);
    oem_adc_init(&current_sense_vout);

    // Start PWM for Fan
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

    // Initial SPI Wakeup
    // This sends a dummy byte to wake up the LTC6811s
    wakeup_sleep(NUM_ICS);

    // Safety Tracking Variables (Used by tasks.h)
    uint16_t pack_voltage = 0;
    uint32_t ov = 0;
    uint32_t uv = 0;
    uint32_t ot = 0;
    uint32_t ut = 0;
    uint16_t min_temp = 0;
    uint16_t max_temp = UINT16_MAX;
    uint16_t pec_errors = 0;
    int16_t current = 0;

    uint8_t loop_counter = 0;

    // Infinite Loop
    while (1) {
        if (main_loop_ready) {
            main_loop_ready = false; // Reset flag

            // Run Safety Tasks (From tasks)
            voltage_task(&pack_voltage, &ov, &uv, &pec_errors);
            temperature_task(&ot, &ut, &min_temp, &max_temp, &pec_errors);
            current_task(&current);
            // openwire_task is implemented as returning an int or taking a pointer 
            // based on file; standard usage:
            // openwire_task();

            // Fan Control based on latest max_temp
            cooling_fan_control(max_temp);

            // Relay Logic (Safety Shutdown)
            // If any safety flags are set, open the relay (PA8)
            if (ov > 0 || uv > (NUM_UNUSED_VOLTAGE_CHANNELS * NUM_ICS) || 
                ot > MAX_EXTRANEOUS_TEMPERATURES || current > CURRENT_THRESH) {
                HAL_GPIO_WritePin(BMS_RELAY_DRIVE_PORT, BMS_RELAY_DRIVE_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_SET);
            } else {
                // No faults, keep relay closed
                HAL_GPIO_WritePin(BMS_RELAY_DRIVE_PORT, BMS_RELAY_DRIVE_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_RESET);
            }


            // Read Safety Sensors (ADC)
            // uint16_t raw_current = oem_adc_read(&current_sense_vout);
            // uint16_t temp1 = oem_adc_read(&pre_dis_temp_1);

            // Heartbeat logic (Toggle every 500ms = 50 loops)
            static uint8_t blink_counter = 0;
            if (++blink_counter >= 50) {
                HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
                blink_counter = 0;
            }
            
            // do i need this?
            // SPI Communication with Battery Chips
            //oem_spi_select(&bms_spi);
            //oem_spi_deselect(&bms_spi);
        }
    }

    return 0;
}

// callback is triggered by the Timer 2 interrupt every 10ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        main_loop_ready = true;
    }
}