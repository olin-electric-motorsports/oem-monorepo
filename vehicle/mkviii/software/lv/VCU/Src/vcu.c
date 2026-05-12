/*
 * MKVIII VCU runtime.
 *
 * The fast 1 ms path samples hardware inputs, evaluates plausibility and
 * shutdown-chain faults, selects the VCU mode, and applies local outputs. The
 * 10 ms path maintains slower status outputs such as heartbeat and CAN frames.
 */

#include "vcu.h"
#include "vcu_config.h"
#include "gpio.h"
#include "adc.h"
#include "can_api.h"
#include "stm32g441xx.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_pwr_ex.h"
#include "stm32g4xx_hal_flash.h"

/* Global runtime state published through CAN and used by each control task. */
vcu_state_s s_state = {0};
/* Hardware mapping populated by GPIO/ADC initialization before vcu_init(). */
vcu_hw_s s_hw = {0};

/* Scheduler flags set by SysTick_Handler and consumed by the main loop. */
static volatile bool s_tick_1ms = false;
static volatile bool s_tick_10ms = false;

/* Input sampling and state update helpers for the fast control loop. */
static void vcu_read_inputs_1ms(void);
static void vcu_read_bspd_1ms(void);
static void vcu_read_throttle_1ms(void);
static void vcu_read_shutdown_chain_1ms(void);
static void vcu_update_fault_manager_1ms(void);
static void vcu_check_brake_throttle_implaus_1ms(void);
/* Output and communication helpers that consume the already-updated state. */
static void vcu_apply_outputs(void);
static void vcu_send_can_10ms(void);
/* Small arithmetic helpers kept local to avoid depending on lib helpers. */
static int16_t vcu_abs_diff_16(int16_t a, int16_t b);
static int16_t vcu_min_16(int16_t a, int16_t b);

/*
 * System tick interrupt hook.
 *
 * HAL timekeeping runs every millisecond. The VCU also derives a 10 ms flag
 * from this interrupt so the main loop can execute both periodic tasks without
 * doing control work directly inside the interrupt context.
 */
void SysTick_Handler(void) {
    static uint8_t tick_10ms_div = 0;

    HAL_IncTick();
    s_tick_1ms = true;

    tick_10ms_div++;
    if (tick_10ms_div >= 10u) {
        tick_10ms_div = 0u;
        s_tick_10ms = true;
    }
}

/*
    Read value from throttle potentiometer, maps it to value between 0 and
    255 where 255 = 100% throttle
    Takes in bool for whether the read throttle is the left or not for
    debugging purposes
    Returns an int16_t representing pedal travel
*/
/*
 * Run one fast VCU control iteration.
 *
 * This function is intended to be called from the foreground loop whenever the
 * 1 ms scheduler flag is set. It leaves all externally visible results in
 * s_state and on the configured GPIO outputs.
 */
HAL_StatusTypeDef vcu_step_1ms(void) {
    vcu_read_inputs_1ms();
    vcu_check_brake_throttle_implaus_1ms();
    vcu_update_fault_manager_1ms();
    vcu_apply_outputs();
    return HAL_OK;
}

/*
 * Sample every input source needed by the fast control path.
 *
 * ADC and GPIO data are copied into s_state first, then the CAN receive queue
 * is drained so any subscribed status can be incorporated by later logic.
 */
static void vcu_read_inputs_1ms(void) {
    vcu_read_bspd_1ms();
    vcu_read_throttle_1ms();
    vcu_read_shutdown_chain_1ms();

    // Read message from CAN
    can_poll_receive_all(); 
}

/*
 * Sample BSPD-related analog and digital monitors.
 *
 * The latched BSPD logic input is inverted in software because a low readback
 * indicates the hardware latch/output is active.
 */
static void vcu_read_bspd_1ms(void) {
    s_state.brake_press_sense = oem_adc_read(s_hw.hadc_brake_press_sense);
    s_state.brake_press_sense_ftr = oem_adc_read(s_hw.hadc_brake_press_sense_ftr);
    s_state.rc_timer_status = oem_adc_read(s_hw.hadc_rc_timer_status);
    
    s_state.brake_gate = !!HAL_GPIO_ReadPin(s_hw.brakelight_ll_port, s_hw.brakelight_ll_pin);
    s_state.bspd_5kw = !!HAL_GPIO_ReadPin(s_hw.motor_current_sense_port, s_hw.motor_current_sense_pin);

    s_state.bspd_latched = !HAL_GPIO_ReadPin(s_hw.bspd_ll_port, s_hw.bspd_ll_pin);
}

/*
 * Read and scale both APPS channels.
 *
 * Raw ADC readings are shifted down before applying the calibrated min/max
 * counts. The scaled values are clipped into the VCU pedal range while separate
 * flags retain whether clipping or sensor mismatch occurred.
 */
static void vcu_read_throttle_1ms(void) {
    // Read raw data from potentiometers
    // TODO: Are we still using int16 instead of uint16?
    int16_t throttle_l_raw = (int16_t)oem_adc_read(s_hw.hadc_throttle_l);
    s_state.throttle_l_raw = throttle_l_raw;
    int16_t throttle_r_raw = (int16_t)oem_adc_read(s_hw.hadc_throttle_r);
    s_state.throttle_r_raw = throttle_r_raw;

    int16_t raw_l = (int16_t)(throttle_l_raw >> 2);
    int16_t raw_r = (int16_t)(throttle_r_raw >> 2);

    int16_t range_l = THROTTLE_L_MAX_COUNTS - THROTTLE_L_MIN_COUNTS;
    int16_t range_r = THROTTLE_R_MAX_COUNTS - THROTTLE_R_MIN_COUNTS;

    /* Record invalid calibration constants for status reporting. */
    if (range_l <= 0 || range_r <= 0){
        s_state.throttle_range_invalid = true;
    } else {
        s_state.throttle_range_invalid = false;
    }

    int32_t scaled_l = (int32_t)(raw_l - THROTTLE_L_MIN_COUNTS) * MAX_THROTTLE_POS;
    int32_t scaled_r = (int32_t)(raw_r - THROTTLE_R_MIN_COUNTS) * MAX_THROTTLE_POS;

    /*
     * Preserve negative below-range values until the clipping checks so the
     * out-of-range flags still describe the original sensor reading.
     */
    if (scaled_l < 0) {
        scaled_l = -(int32_t)((-scaled_l + range_l - 1) / range_l);
    } else {
        scaled_l = scaled_l / range_l;
    }

    if (scaled_r < 0) {
        scaled_r = -(int32_t)((-scaled_r + range_r - 1) / range_r);
    } else {
        scaled_r = scaled_r / range_r;
    }

    // Check if out of range
    if (scaled_l > MAX_THROTTLE_POS) {
        scaled_l = MAX_THROTTLE_POS;
        s_state.throttle_l_out_of_range = true;
    } else if (scaled_l < MIN_THROTTLE_POS) {
        scaled_l = MIN_THROTTLE_POS;
        s_state.throttle_l_out_of_range = true;
    } else {
        s_state.throttle_l_out_of_range = false;
    }

    if (scaled_r > MAX_THROTTLE_POS) {
        scaled_r = MAX_THROTTLE_POS;
        s_state.throttle_r_out_of_range = true;
    } else if (scaled_r < MIN_THROTTLE_POS) {
        scaled_r = MIN_THROTTLE_POS;
        s_state.throttle_r_out_of_range = true;
    } else {
        s_state.throttle_r_out_of_range = false;
    }

    s_state.throttle_l_scaled = scaled_l;
    s_state.throttle_r_scaled = scaled_r;

    // Check if mismatch
    int16_t throttle_diff = vcu_abs_diff_16((int16_t)scaled_l, (int16_t)scaled_r);
    s_state.throttles_mismatch = throttle_diff > APPS_IMPLAUSIBILITY_DEVIATION_THRESHOLD;
}

/*
 * Read shutdown-chain continuity inputs.
 *
 * Both monitored shutdown-chain signals are active-low at the MCU, so a reset
 * pin state is interpreted as a closed segment.
 */
static void vcu_read_shutdown_chain_1ms(void) {
    // BSPD_SHUTDOWN_SENSE is inverted before reaching the MCU.
    s_state.ss_bspd_closed =
        HAL_GPIO_ReadPin(s_hw.bspd_shutdown_sense_port, s_hw.bspd_shutdown_sense_pin)
        == GPIO_PIN_RESET;
    s_state.ss_inertia_closed =
        HAL_GPIO_ReadPin(s_hw.ss_is_port, s_hw.ss_is_pin) 
        == GPIO_PIN_RESET;
}

/*
 * Build fault bitmasks and select the high-level operating mode.
 *
 * Fault bits are recomputed from current sampled state on every 1 ms tick,
 * while specific plausibility conditions latch in s_state until their own
 * reset criteria are met.
 */
static void vcu_update_fault_manager_1ms() {
    uint32_t fault_bits = VCU_FAULT_NONE;

    // BSPD
    if (s_state.bspd_latched) {
        fault_bits |= VCU_FAULT_BSPD_POWER_LATCHED;
    }
    if (!s_state.ss_bspd_closed) {
        fault_bits |= VCU_FAULT_SHUTDOWN_BSPD_OPEN;
    }
    if (!s_state.ss_inertia_closed) {
        fault_bits |= VCU_FAULT_INERTIA_SWITCH_OPEN;
    }

    // Throttle
    bool throttle_implausible_now = false;
    throttle_implausible_now = 
        s_state.throttle_l_out_of_range || 
        s_state.throttle_r_out_of_range || 
        s_state.throttles_mismatch;

    /* Report immediate APPS fault causes separately from the timeout latch. */
    if (s_state.throttle_l_out_of_range) {
        fault_bits |= VCU_FAULT_APPS1_OUT_OF_RANGE;
    }
    if (s_state.throttle_r_out_of_range) {
        fault_bits |= VCU_FAULT_APPS2_OUT_OF_RANGE;
    }
    if (s_state.throttles_mismatch) {
        fault_bits |= VCU_FAULT_APPS_MISMATCH;
    }

    if (throttle_implausible_now) {
        s_state.throttle_implaus_timer_ms++;
        if (s_state.throttle_implaus_timer_ms >= IMPLAUSIBILITY_TIME_LIMIT) {
            s_state.throttle_implaus_latched = true;
        }
    } else {
        s_state.throttle_implaus_timer_ms = 0;
    }

    if (s_state.throttle_implaus_latched) {
        fault_bits |= VCU_FAULT_APPS_TIMEOUT_LATCHED;
    }

    // Shared
    /* Brake/throttle implausibility is evaluated outside the APPS timer. */
    if (s_state.brake_throttle_implaus_latched) {
        fault_bits |= VCU_FAULT_BRAKE_THROTTLE_IMPLAUS;
    }

    // Update fault bits
    s_state.fault_bits = fault_bits;
    s_state.blocking_fault_bits = fault_bits;

    // Update VCU mode
    if (s_state.fault_bits != 0u) {
        s_state.mode = VCU_MODE_FAULT;
    // TODO: Dashboard CAN subscription required
    // } else if (!dashboard.ready_to_drive) {
    //     s_state.mode = VCU_MODE_NOT_READY;
    //     s_state.torque_command = 0;
    } else if (s_state.brake_gate) {
        s_state.mode = VCU_MODE_BRAKING;
    } else {
        s_state.mode = VCU_MODE_RUN;
    }
}

/*
 * Enforce the brake/throttle plausibility rule.
 *
 * The latch is set when braking overlaps with more than the high APPS
 * threshold, then clears only after pedal travel drops below the low threshold.
 */
static void vcu_check_brake_throttle_implaus_1ms(void) {
    int16_t throttle_scaled_min =
        vcu_min_16(s_state.throttle_l_scaled, s_state.throttle_r_scaled);

    /* Use hysteresis so the latch does not chatter near the threshold. */
    if (s_state.brake_throttle_implaus_latched) {
        if (throttle_scaled_min <= APPS_BRAKE_IMPLAUSIBILITY_THRESHOLD_LOW) {
            s_state.brake_throttle_implaus_latched = false;
        }
    } else if (s_state.brake_gate &&
               throttle_scaled_min >= APPS_BRAKE_IMPLAUSIBILITY_THRESHOLD) {
        s_state.brake_throttle_implaus_latched = true;
    }
}

/*
 * Apply the selected mode to torque and GPIO outputs.
 *
 * Torque is forced to zero outside RUN mode. In RUN mode the lower of the two
 * redundant pedal readings is used so a single high sensor cannot command more
 * torque than the other channel agrees with.
 */
static void vcu_apply_outputs(void){
    if (s_state.mode != VCU_MODE_RUN) {
        s_state.torque_command = 0;
    } else {
        int16_t torque_raw = vcu_min_16(s_state.throttle_l_scaled, s_state.throttle_r_scaled) * TORQUE_REQUEST_SCALE;

        //Prevent motor whining while idle
        if (torque_raw < 20) {
            s_state.torque_command = 0;
        }else if (torque_raw > 2300) {
            s_state.torque_command = 2540;
        } else {
            s_state.torque_command = torque_raw;
        }

    }


    // Update LEDs
    if (s_state.fault_bits != 0u) {
        HAL_GPIO_WritePin(s_hw.error_led_port, s_hw.error_led_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(s_hw.error_led_port, s_hw.error_led_pin, GPIO_PIN_RESET);
    }

    if (s_state.heartbeat) {
        HAL_GPIO_WritePin(s_hw.heartbeat_led_port, s_hw.heartbeat_led_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(s_hw.heartbeat_led_port, s_hw.heartbeat_led_pin, GPIO_PIN_RESET);
    }

    if (s_state.brake_gate) {
        HAL_GPIO_WritePin(s_hw.brake_ll_led_port, s_hw.brake_ll_led_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(s_hw.brake_ll_led_port, s_hw.brake_ll_led_pin, GPIO_PIN_RESET);
    }

    if (s_state.bspd_5kw) {
        HAL_GPIO_WritePin(s_hw.motor_5kw_led_port, s_hw.motor_5kw_led_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(s_hw.motor_5kw_led_port, s_hw.motor_5kw_led_pin, GPIO_PIN_RESET);
    }
}

/*
 * Run one slower VCU status iteration.
 *
 * The 10 ms task updates software heartbeat timing and publishes all CAN
 * messages owned by this module.
 */
HAL_StatusTypeDef vcu_step_10ms(void) {
    if (HEARTBEAT_TOGGLE_MS > 0u) {
        s_state.heartbeat_elapsed_ms = (uint16_t)(s_state.heartbeat_elapsed_ms + 10u);
        if (s_state.heartbeat_elapsed_ms >= HEARTBEAT_TOGGLE_MS) {
            s_state.heartbeat = !s_state.heartbeat;
            s_state.heartbeat_elapsed_ms = 0u;
        }
    } else {
        s_state.heartbeat = false;
        s_state.heartbeat_elapsed_ms = 0u;
    }

    vcu_send_can_10ms();
    
    return HAL_OK;
}

/*
 * Copy the current runtime state into generated CAN signal structs and send
 * the corresponding frames.
 *
 * The generated can_api layer owns packing/scaling. This function only selects
 * which internal state values should be exposed on each VCU message.
 */
static void vcu_send_can_10ms(void) {
    vcu_throttle_state.throttle_l_raw = (uint16_t)s_state.throttle_l_raw;
    vcu_throttle_state.throttle_r_raw = (uint16_t)s_state.throttle_r_raw;
    vcu_throttle_state.throttle_l_scaled = (uint8_t)s_state.throttle_l_scaled;
    vcu_throttle_state.throttle_r_scaled = (uint8_t)s_state.throttle_r_scaled;
    vcu_throttle_state.throttle_range_invalid =
        s_state.throttle_range_invalid ? 1u : 0u;
    vcu_throttle_state.throttle_implaus_timer_ms =
        s_state.throttle_implaus_timer_ms;

    vcu_bspd_state.brake_press_sense = s_state.brake_press_sense;
    vcu_bspd_state.brake_press_sense_filtered = s_state.brake_press_sense_ftr;
    vcu_bspd_state.rc_timer_status = s_state.rc_timer_status;
    vcu_bspd_state.brake_gate = s_state.brake_gate ? 1u : 0u;
    vcu_bspd_state.bspd_5kw = s_state.bspd_5kw ? 1u : 0u;

    vcu_status.fault_bits_lo = s_state.fault_bits;
    vcu_status.fault_bits_hi = 0u;
    vcu_status.mode = (uint8_t)s_state.mode;
    vcu_status.heartbeat = s_state.heartbeat ? 1u : 0u;

    /*
    * Sets the torque request in the motor controller command message
    */
    m192_command_message.torque_command = s_state.torque_command;

    // Send Message to CAN
    can_send_vcu_throttle_state();
    can_send_vcu_bspd_state();
    can_send_vcu_status();
    can_send_m192_command_message();
}


/*
 * Validate hardware mappings populated by peripheral init and clear runtime
 * state before the main loop starts.
 */
HAL_StatusTypeDef vcu_init(void) {
    s_state = (vcu_state_s){0};

    // BSPD
    if (s_hw.hadc_brake_press_sense == NULL || s_hw.hadc_brake_press_sense_ftr == NULL
        || s_hw.hadc_rc_timer_status == NULL || s_hw.brake_ll_led_port == NULL
        || s_hw.motor_5kw_led_port == NULL || s_hw.error_led_port == NULL
        || s_hw.heartbeat_led_port == NULL || s_hw.bspd_ll_port == NULL
        || s_hw.brakelight_ll_port == NULL || s_hw.motor_current_sense_port == NULL
        || s_hw.bspd_shutdown_sense_port == NULL || s_hw.brake_ll_led_pin == 0u
        || s_hw.motor_5kw_led_pin == 0u || s_hw.error_led_pin == 0u
        || s_hw.heartbeat_led_pin == 0u || s_hw.bspd_ll_pin == 0u
        || s_hw.brakelight_ll_pin == 0u || s_hw.motor_current_sense_pin == 0u
        || s_hw.bspd_shutdown_sense_pin == 0u) {
        s_state.mode = VCU_MODE_FAULT;
        return HAL_ERROR;
    }

    // Throttle
    if (s_hw.hadc_throttle_l == NULL || s_hw.hadc_throttle_r == NULL
        || s_hw.ss_is_port == NULL || s_hw.error_led_port == NULL
        || s_hw.heartbeat_led_port == NULL || s_hw.ss_is_pin == 0u
        || s_hw.error_led_pin == 0u || s_hw.heartbeat_led_pin == 0u) {
        s_state.mode = VCU_MODE_FAULT;
        return HAL_ERROR;
    }

    s_state.mode = VCU_MODE_NOT_READY;
    s_state.fault_bits = VCU_FAULT_NONE;
    s_state.blocking_fault_bits = VCU_FAULT_NONE;
    s_state.heartbeat = false;
    s_state.heartbeat_elapsed_ms = 0u;

    s_state.torque_command = 0;
    s_state.throttle_implaus_timer_ms = 0u;
    s_state.throttle_implaus_latched = false;
    s_state.brake_throttle_implaus_latched = false;

    s_state.inverter_command_publish_elapsed_ms = 0u;
    
    vcu_apply_outputs();

    can_poll_receive_all(); 
    /*
     * This feature is added so that the inverter cannot be accidentally enabled
     * when first powered up. This feature requires that before sending out an
     * Inverter Enable command, the user must send out an Inverter Disable
     * command. Once the inverter sees a Disable command, the lockout is removed
     * and controller can receive the Inverter Enable command
     */
    can_send_m192_command_message();

    m192_command_message.direction_command = MOTOR_ANTICLOCKWISE;

    return HAL_OK;
}

/*
 * Firmware entry point.
 *
 * Initialization is deliberately ordered as HAL, clocks, GPIO, ADC, CAN, then
 * VCU state validation so vcu_init() can verify every required hardware handle
 * before the scheduler begins running.
 */
int main(void) {
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    if (vcu_gpio_init() != HAL_OK) {
        Error_Handler();
    }
    if (vcu_adc_init() != HAL_OK) {
        Error_Handler();
    }
    if (can_init_vcu() != HAL_OK) {
        Error_Handler();
    }
    if (vcu_init() != HAL_OK) {
        Error_Handler();
    }

    while (true) {
        if (s_tick_1ms) {
            s_tick_1ms = false;
            vcu_step_1ms();
        }

        if (s_tick_10ms) {
            s_tick_10ms = false;
            vcu_step_10ms();
        }
    }
}

/*
 * Return the absolute difference between two signed 16-bit values.
 *
 * Inputs are expected to be scaled pedal values small enough that the
 * subtraction remains inside int16_t range.
 */
static inline int16_t vcu_abs_diff_16(int16_t a, int16_t b) {
    return (a > b) ? (int16_t)(a - b) : (int16_t)(b - a);
}

/* Return the lower of two signed 16-bit values. */
static inline int16_t vcu_min_16(int16_t a, int16_t b) {
    return (a < b) ? a : b;
}
