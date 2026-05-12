#ifndef VCU_H
#define VCU_H

#include "common/adc/adc.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Public interface for the MKVIII low-voltage Vehicle Control Unit.
 *
 * The VCU owns pedal plausibility checks, BSPD/shutdown-chain monitoring,
 * torque-command limiting, status LEDs, and periodic CAN status publishing.
 */

/* High-level operating mode of the VCU torque state machine. */
typedef enum {
    /* Startup/default value before peripheral and runtime initialization. */
    VCU_MODE_INIT = 0,
    /* Hardware is initialized but the ready-to-drive gate is not yet satisfied. */
    VCU_MODE_NOT_READY,
    /* A blocking fault is active and torque output must remain zero. */
    VCU_MODE_FAULT,
    /* Brake signal is active, so torque output is intentionally suppressed. */
    VCU_MODE_BRAKING,
    /* No blocking fault or brake gate is active; torque can follow pedal input. */
    VCU_MODE_RUN,
} vcu_mode_e;

/*
 * Fault bits stored in vcu_state_s::fault_bits.
 * These values are intended for bitwise operations (|, &, ~), not mutually
 * exclusive enum states.
 */
typedef enum {
    VCU_FAULT_NONE = 0u,
    /* Left APPS reading was clipped outside its calibrated pedal range. */
    VCU_FAULT_APPS1_OUT_OF_RANGE = (1u << 0),
    /* Right APPS reading was clipped outside its calibrated pedal range. */
    VCU_FAULT_APPS2_OUT_OF_RANGE = (1u << 1),
    /* The two APPS scaled values differ by more than the allowed deviation. */
    VCU_FAULT_APPS_MISMATCH = (1u << 2),
    /* APPS implausibility persisted past the configured timeout. */
    VCU_FAULT_APPS_TIMEOUT_LATCHED = (1u << 3),
    /* Inertia switch sense indicates an open shutdown-chain segment. */
    VCU_FAULT_INERTIA_SWITCH_OPEN = (1u << 4),
    /* BSPD logic-level monitor indicates that the BSPD output is latched. */
    VCU_FAULT_BSPD_POWER_LATCHED = (1u << 5),
    /* Brake and accelerator were applied together past the allowed threshold. */
    VCU_FAULT_BRAKE_THROTTLE_IMPLAUS = (1u << 6),
    /* BSPD shutdown-chain sense indicates an open shutdown-chain segment. */
    VCU_FAULT_SHUTDOWN_BSPD_OPEN = (1u << 7)
} vcu_fault_bit_e;

/*
 * Hardware mapping for this module.
 * HAL handles are provided by board/app initialization and passed into vcu_init().
 */
typedef struct {
    // Throttle 
    /* ADC channel for the left accelerator pedal position sensor. */
    oem_adc_config_t* hadc_throttle_l;
    /* ADC channel for the right accelerator pedal position sensor. */
    oem_adc_config_t* hadc_throttle_r;
    
    /* Inertia switch shutdown-chain sense input. */
    GPIO_TypeDef* ss_is_port;
    uint16_t ss_is_pin;

    /* Heartbeat LED output used to show that the scheduler is running. */
    GPIO_TypeDef* heartbeat_led_port;
    uint16_t heartbeat_led_pin;

    /* Fault LED output asserted whenever any fault bit is active. */
    GPIO_TypeDef* error_led_port;
    uint16_t error_led_pin;

    // BSPD
    /* LED output mirroring the brake-light logic-level state. */
    GPIO_TypeDef* brake_ll_led_port;
    uint16_t brake_ll_led_pin;

    /* LED output mirroring the BSPD 5 kW motor-power indication. */
    GPIO_TypeDef* motor_5kw_led_port;
    uint16_t motor_5kw_led_pin;

    /* BSPD logic-level monitor input; latched faults are active-low in software. */
    GPIO_TypeDef* bspd_ll_port;
    uint16_t bspd_ll_pin;

    /* Brake-light low-side-driver logic-level monitor input. */
    GPIO_TypeDef* brakelight_ll_port;
    uint16_t brakelight_ll_pin;

    /* Digital motor current sense input used by the BSPD path. */
    GPIO_TypeDef* motor_current_sense_port;
    uint16_t motor_current_sense_pin;

    /* Shutdown-chain sense input for the BSPD segment. */
    GPIO_TypeDef* bspd_shutdown_sense_port;
    uint16_t bspd_shutdown_sense_pin;

    /* Raw brake pressure sense ADC channel. */
    oem_adc_config_t* hadc_brake_press_sense;
    /* Filtered brake pressure sense ADC channel. */
    oem_adc_config_t* hadc_brake_press_sense_ftr;
    /* ADC channel exposing the BSPD RC timing-node voltage. */
    oem_adc_config_t* hadc_rc_timer_status;
} vcu_hw_s;


/*
 * Full runtime state of the VCU core.
 * This struct is both internal state and the payload for debug/status publish hooks.
 */
typedef struct {
    // Throttle
    /* Raw ADC counts before right-shifting or calibration scaling. */
    int16_t throttle_l_raw;
    int16_t throttle_r_raw;

    /* Pedal travel scaled to MIN_THROTTLE_POS..MAX_THROTTLE_POS. */
    int16_t throttle_l_scaled;
    int16_t throttle_r_scaled;

    /* Calibration guard set when a configured min/max range is invalid. */
    bool throttle_range_invalid;
    /* Per-channel range flags set when scaled pedal values are clipped. */
    bool throttle_l_out_of_range;
    bool throttle_r_out_of_range;
    /* True when the two APPS channels differ beyond the plausibility limit. */
    bool throttles_mismatch;
    /* Latches once APPS implausibility survives the required debounce period. */
    bool throttle_implaus_latched;
    /* Latches brake/throttle overlap until pedal travel returns below reset threshold. */
    bool brake_throttle_implaus_latched;

    /* Millisecond counter for continuous APPS implausibility timing. */
    uint16_t throttle_implaus_timer_ms;
    /* Final torque request after fault, brake, idle, and scaling limits. */
    int16_t torque_command;

    // BSPD
    /* Raw ADC monitor values published for BSPD diagnostics. */
    uint16_t brake_press_sense;
    uint16_t brake_press_sense_ftr;
    uint16_t rc_timer_status;

    /* Digital BSPD and shutdown-chain states sampled by the 1 ms loop. */
    bool brake_gate;
    bool bspd_5kw;
    bool ss_bspd_closed;
    bool ss_inertia_closed;
    bool bspd_latched;

    // Shared
    /* Heartbeat state toggled by the 10 ms loop and reflected on CAN/LED. */
    bool heartbeat;
    uint16_t heartbeat_elapsed_ms;

    /* Fault bitmasks used for local mode selection and CAN status reporting. */
    uint16_t fault_bits;
    uint16_t blocking_fault_bits;
    /* Current high-level VCU operating mode. */
    vcu_mode_e mode;
    /* Reserved timing state for inverter-command publish cadence. */
    uint16_t inverter_command_publish_elapsed_ms;
} vcu_state_s;

extern vcu_state_s s_state;
extern vcu_hw_s s_hw;

/* Configure the system clock */
void ErrorHandler(void);
void SysTick_Handler(void);
void SystemClockConfig(void);
/* Initialize runtime state after GPIO, ADC, and CAN peripheral setup. */
HAL_StatusTypeDef vcu_init(void);
/* Execute the 1 ms control task: input sampling, fault updates, and outputs. */
HAL_StatusTypeDef vcu_step_1ms(void);
/* Execute the 10 ms task: heartbeat maintenance and CAN publishing. */
HAL_StatusTypeDef vcu_step_10ms(void);

#define Error_Handler ErrorHandler
#define SystemClock_Config SystemClockConfig

#endif // VCU_H
