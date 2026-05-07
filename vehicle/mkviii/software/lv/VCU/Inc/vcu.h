#ifndef VCU_H
#define VCU_H

#include "common/adc/adc.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* High-level operating mode of the VCU torque state machine. */
typedef enum {
    VCU_MODE_INIT = 0,
    VCU_MODE_NOT_READY,
    VCU_MODE_FAULT,
    VCU_MODE_BRAKING,
    VCU_MODE_RUN,
} vcu_mode_e;

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
    VCU_FAULT_BSPD_POWER_LATCHED = (1u << 5),
    VCU_FAULT_BRAKE_THROTTLE_IMPLAUS = (1u << 6),
    VCU_FAULT_SHUTDOWN_BSPD_OPEN = (1u << 7)
} vcu_fault_bit_e;

/*
 * Hardware mapping for this module.
 * HAL handles are provided by board/app initialization and passed into vcu_init().
 */
typedef struct {
    // Throttle 
    oem_adc_config_t* hadc_throttle_l;
    oem_adc_config_t* hadc_throttle_r;
    
    GPIO_TypeDef* ss_is_port;
    uint16_t ss_is_pin;

    GPIO_TypeDef* heartbeat_led_port;
    uint16_t heartbeat_led_pin;

    GPIO_TypeDef* error_led_port;
    uint16_t error_led_pin;

    // BSPD
    GPIO_TypeDef* brake_ll_led_port;
    uint16_t brake_ll_led_pin;

    GPIO_TypeDef* motor_5kw_led_port;
    uint16_t motor_5kw_led_pin;

    GPIO_TypeDef* bspd_ll_port;
    uint16_t bspd_ll_pin;

    GPIO_TypeDef* brakelight_ll_port;
    uint16_t brakelight_ll_pin;

    GPIO_TypeDef* motor_current_sense_port;
    uint16_t motor_current_sense_pin;

    GPIO_TypeDef* bspd_shutdown_sense_port;
    uint16_t bspd_shutdown_sense_pin;

    oem_adc_config_t* hadc_brake_press_sense;
    oem_adc_config_t* hadc_brake_press_sense_ftr;
    oem_adc_config_t* hadc_rc_timer_status;
} vcu_hw_s;


/*
 * Full runtime state of the VCU core.
 * This struct is both internal state and the payload for debug/status publish hooks.
 */
typedef struct {
    // Throttle
    int16_t throttle_l_raw;
    int16_t throttle_r_raw;

    int16_t throttle_l_scaled;
    int16_t throttle_r_scaled;

    bool throttle_range_invalid;
    bool throttle_l_out_of_range;
    bool throttle_r_out_of_range;
    bool throttles_mismatch;
    bool throttle_implaus_latched;
    bool brake_throttle_implaus_latched;

    uint16_t throttle_implaus_timer_ms;
    int16_t torque_command;

    // BSPD
    uint16_t brake_press_sense;
    uint16_t brake_press_sense_ftr;
    uint16_t rc_timer_status;

    bool brake_gate;
    bool bspd_5kw;
    bool ss_bspd_closed;
    bool ss_inertia_closed;
    bool bspd_latched;

    // Shared
    bool heartbeat;
    uint16_t heartbeat_elapsed_ms;

    uint16_t fault_bits;
    uint16_t blocking_fault_bits;
    vcu_mode_e mode;
    uint16_t inverter_command_publish_elapsed_ms;
} vcu_state_s;

extern vcu_state_s s_state;
extern vcu_hw_s s_hw;

/* Configure the system clock */
void ErrorHandler(void);
void SysTick_Handler(void);
void SystemClockConfig(void);
HAL_StatusTypeDef vcu_init(void);
HAL_StatusTypeDef vcu_step_1ms(void);
HAL_StatusTypeDef vcu_step_10ms(void);

#define Error_Handler ErrorHandler
#define SystemClock_Config SystemClockConfig

#endif // VCU_H
