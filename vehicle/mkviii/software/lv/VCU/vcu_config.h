#ifndef VEHICLE_MKVII_SOFTWARE_LV_VCU_VCU_CONFIG_H_
#define VEHICLE_MKVII_SOFTWARE_LV_VCU_VCU_CONFIG_H_

#include "libs/adc/api.h"
#include "libs/gpio/api.h"
#include "libs/gpio/pin_defs.h"
#include "libs/timer/api.h"

#define VCU_MOTOR_ANTICLOCKWISE (0)

#define VCU_APPS1_ADC_CHANNEL (ADC5)
#define VCU_APPS2_ADC_CHANNEL (ADC6)
#define VCU_BRAKE_PRESSURE_ADC_CHANNEL (ADC7)
#define VCU_BRAKE_PRESSURE_FILTERED_ADC_CHANNEL (ADC4)

#define VCU_APPS1_MIN_COUNTS (30u)
#define VCU_APPS1_MAX_COUNTS (1023u)
#define VCU_APPS2_MIN_COUNTS (20u)
#define VCU_APPS2_MAX_COUNTS (667u)

static gpio_t VCU_BRAKE_GATE_SENSE = PB4;
static gpio_t VCU_MOTOR_CURRENT_SENSE = PB5;
static gpio_t VCU_SS_BSPD_SENSE = PC0;
static gpio_t VCU_FAULT_LED = PD5;
static gpio_t VCU_HEARTBEAT_LED = PD6;

void vcu_timer_10ms_callback(void);
void vcu_timer_1ms_callback(void);

/*
 * 10 ms timer (100 Hz)
 * F_CPU / prescalar / output_compare_match
 * 4000000 / 1024 / 39 = 100
 */
static timer_cfg_s vcu_timer_10ms_cfg = {
    .timer = TIMER0,
    .timer0_mode = TIMER0_MODE_CTC,
    .prescalar = CLKIO_DIV_1024,
    .channel_a = {
        .channel = CHANNEL_A,
        .output_compare_match = 0x27,
        .pin_behavior = DISCONNECTED,
        .interrupt_enable = true,
        .interrupt_callback = vcu_timer_10ms_callback,
    },
};

/*
 * 1 ms timer (1 kHz)
 * F_CPU / prescalar / output_compare_match
 * 4000000 / 1 / 4000 = 1000
 */
static timer_cfg_s vcu_timer_1ms_cfg = {
    .timer = TIMER1,
    .timer1_mode = TIMER1_MODE_CTC,
    .prescalar = CLKIO_DIV_1,
    .channel_a = {
        .channel = CHANNEL_A,
        .output_compare_match = 4000,
        .pin_behavior = DISCONNECTED,
        .interrupt_enable = true,
        .interrupt_callback = vcu_timer_1ms_callback,
    },
};

#endif /* VEHICLE_MKVII_SOFTWARE_LV_VCU_VCU_CONFIG_H_ */
