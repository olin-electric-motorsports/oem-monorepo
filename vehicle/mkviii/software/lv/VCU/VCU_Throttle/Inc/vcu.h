// #include "libs/adc/api.h"
// #include "libs/gpio/api.h"
// #include "libs/gpio/pin_defs.h"
// #include "libs/timer/api.h"
#ifndef VCU_H
#define VCU_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* High-level operating mode of the VCU torque state machine. */
typedef enum {
    VCU_MODE_INIT = 0,
    VCU_MODE_NOT_READY,
    VCU_MODE_FAULT,
    VCU_MODE_RUN,
} vcu_mode_e;

// // all possible throttle states
// enum State {
//     THROTTLE_IDLE,
//     THROTTLE_RUN,
//     THROTTLE_L_OUT_OF_RANGE,
//     THROTTLE_R_OUT_OF_RANGE,
//     THROTTLE_POSITION_IMPLAUSIBILITY,
//     THROTTLE_BRAKE_PRESSED
// };

/*
 * Fault bits stored in vcu_state_s::fault_bits.
 * These values are intended for bitwise operations (|, &, ~), not mutually
 * exclusive enum states.
 */
typedef enum {
    VCU_FAULT_NONE = 0u,
    VCU_FAULT_APPS1_OUT_OF_RANGE = (1u << 0),
    VCU_FAULT_APPS2_OUT_OF_RANGE = (1u << 1),
    VCU_FAULT_APPS_MISMATCH = (1u << 2),
    VCU_FAULT_APPS_TIMEOUT_LATCHED = (1u << 3),
    VCU_FAULT_INERTIA_SWITCH_OPEN = (1u << 4),
} vcu_fault_bit_e;

/*
 * Hardware mapping for this module.
 * HAL handles are provided by board/app initialization and passed into vcu_init().
 */
typedef struct {
    ADC_HandleTypeDef* hadc_apps1;
    ADC_HandleTypeDef* hadc_apps2;

    GPIO_PinState ss_inertia_closed_state;
    GPIO_PinState indicator_led_active_state;
    GPIO_PinState heartbeat_led_active_state;
} vcu_state_s;

// PA0 or ADC1_IN1?


/* Configure the system clock */
void SystemClock_Config(void);

void ErrorHandler(void);

void SystemClockConfig(void);

void SysTick_Handler(void);

#endif // VCU_H