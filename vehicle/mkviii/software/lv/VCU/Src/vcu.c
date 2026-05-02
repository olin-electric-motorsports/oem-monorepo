#include "vcu.h"
#include "vcu_config.h"
#include "gpio.h"
#include "adc.h"
#include "fdcan.h"
#include "stm32g441xx.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_pwr_ex.h"
#include "stm32g4xx_hal_flash.h"

vcu_state_s s_state = {0};
vcu_hw_s s_hw = {0};

static volatile bool s_tick_1ms = false;
static volatile bool s_tick_10ms = false;

static void vcu_read_inputs_1ms(void);
static void vcu_read_bspd_1ms(void);
static void vcu_read_throttle_1ms(void);
static void vcu_read_shutdown_chain_1ms(void);
static void vcu_update_fault_manager_1ms(void);
static void vcu_check_brake_throttle_implaus_1ms(void);
static void vcu_apply_outputs(void);
static int16_t vcu_abs_diff_16(int16_t a, int16_t b);
static int16_t vcu_min_16(int16_t a, int16_t b);

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
HAL_StatusTypeDef vcu_step_1ms(void) {
    vcu_read_inputs_1ms();
    vcu_check_brake_throttle_implaus_1ms();
    vcu_update_fault_manager_1ms();
    vcu_apply_outputs();
    // update torque command
    return HAL_OK;
}

static void vcu_read_inputs_1ms(void) {
    vcu_read_bspd_1ms();
    vcu_read_throttle_1ms();
    vcu_read_shutdown_chain_1ms();
}

static void vcu_read_bspd_1ms(void) {
    s_state.brake_press_sense = oem_adc_read(s_hw.hadc_brake_press_sense);
    s_state.brake_press_sense_ftr = oem_adc_read(s_hw.hadc_brake_press_sense_ftr);
    s_state.rc_timer_status = oem_adc_read(s_hw.hadc_rc_timer_status);
    
    s_state.brake_gate = !!HAL_GPIO_ReadPin(s_hw.brakelight_ll_port, s_hw.brakelight_ll_pin);
    s_state.bspd_5kw = !!HAL_GPIO_ReadPin(s_hw.motor_current_sense_port, s_hw.motor_current_sense_pin);

    s_state.bspd_latched = !HAL_GPIO_ReadPin(s_hw.bspd_ll_port, s_hw.bspd_ll_pin);
}

static void vcu_read_throttle_1ms(void) {
    // Read raw data from potentiometers
    // Are we still using int16 instead of uint16
    int16_t throttle_l_raw = (int16_t)oem_adc_read(s_hw.hadc_throttle_l);
    s_state.throttle_l_raw = throttle_l_raw;
    int16_t throttle_r_raw = (int16_t)oem_adc_read(s_hw.hadc_throttle_r);
    s_state.throttle_r_raw = throttle_r_raw;

    int16_t raw_l = (int16_t)(throttle_l_raw >> 2);
    int16_t raw_r = (int16_t)(throttle_r_raw >> 2);

    int16_t range_l = THROTTLE_L_MAX_COUNTS - THROTTLE_L_MIN_COUNTS;
    int16_t range_r = THROTTLE_R_MAX_COUNTS - THROTTLE_R_MIN_COUNTS;

    if (range_l <= 0 || range_r <= 0){
        s_state.throttle_range_invalid = true;
    } else {
        s_state.throttle_range_invalid = false;
    }

    int32_t scaled_l = (int32_t)(raw_l - THROTTLE_L_MIN_COUNTS) * MAX_THROTTLE_POS;
    int32_t scaled_r = (int32_t)(raw_r - THROTTLE_R_MIN_COUNTS) * MAX_THROTTLE_POS;

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

static void vcu_read_shutdown_chain_1ms(void) {
    // BSPD_SHUTDOWN_SENSE is inverted before reaching the MCU.
    s_state.ss_bspd_closed =
        HAL_GPIO_ReadPin(s_hw.bspd_shutdown_sense_port, s_hw.bspd_shutdown_sense_pin)
        == GPIO_PIN_RESET;
    s_state.ss_inertia_closed =
        HAL_GPIO_ReadPin(s_hw.ss_is_port, s_hw.ss_is_pin) 
        == GPIO_PIN_RESET;
}

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
    if (s_state.brake_throttle_implaus_latched) {
        fault_bits |= VCU_FAULT_BRAKE_THROTTLE_IMPLAUS;
    }

    s_state.fault_bits = fault_bits;
    s_state.blocking_fault_bits = fault_bits;
}

static void vcu_check_brake_throttle_implaus_1ms(void) {
    int16_t throttle_scaled_min =
        vcu_min_16(s_state.throttle_l_scaled, s_state.throttle_r_scaled);

    if (s_state.brake_throttle_implaus_latched) {
        if (throttle_scaled_min <= APPS_BRAKE_IMPLAUSIBILITY_THRESHOLD_LOW) {
            s_state.brake_throttle_implaus_latched = false;
        }
    } else if (s_state.brake_gate &&
               throttle_scaled_min >= APPS_BRAKE_IMPLAUSIBILITY_THRESHOLD) {
        s_state.brake_throttle_implaus_latched = true;
    }
}

static void vcu_apply_outputs(void){
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
    
    return HAL_OK;
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
    return HAL_OK;
}

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
    if (vcu_init() != HAL_OK) {
        Error_Handler();
    }
    // FDCAN1_Init(); // Temporarily disabled

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

static inline int16_t vcu_abs_diff_16(int16_t a, int16_t b) {
    return (a > b) ? (int16_t)(a - b) : (int16_t)(b - a);
}

static inline int16_t vcu_min_16(int16_t a, int16_t b) {
    return (a < b) ? a : b;
}
