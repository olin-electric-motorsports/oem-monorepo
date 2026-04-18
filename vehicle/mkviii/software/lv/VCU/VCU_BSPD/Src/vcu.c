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
static void vcu_update_fault_manager_1ms(void);
static void vcu_apply_outputs(void);
static int16_t vcu_abs_diff_16(int16_t a, int16_t b);

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
    vcu_update_fault_manager_1ms();
    vcu_apply_outputs();
    // update torque command
    return HAL_OK;
}

static void vcu_read_inputs_1ms(void) {
    s_state.brake_press_sense = oem_adc_read(s_hw.hadc_brake_press_sense);
    s_state.brake_press_sense_ftr = oem_adc_read(s_hw.hadc_brake_press_sense_ftr);
    s_state.rc_timer_status = oem_adc_read(s_hw.hadc_rc_timer_status);
    
    s_state.brake_gate = !!HAL_GPIO_ReadPin(s_hw.brakelight_ll_port, s_hw.brakelight_ll_pin);
    s_state.bspd_5kw = !!HAL_GPIO_ReadPin(s_hw.motor_current_sense_port, s_hw.motor_current_sense_pin);

    s_state.ss_bspd = !HAL_GPIO_ReadPin(s_hw.bspd_shutdown_sense_port, s_hw.bspd_shutdown_sense_pin);
    s_state.bspd_latched = !HAL_GPIO_ReadPin(s_hw.bspd_ll_port, s_hw.bspd_ll_pin);
}

static void vcu_update_fault_manager_1ms() {
    uint32_t fault_bits = VCU_FAULT_NONE;
    
    if (s_state.bspd_latched) {
        fault_bits |= VCU_FAULT_BSPD_POWER_LATCHED;
    }

    s_state.fault_bits = fault_bits;
    // s_state.blocking_fault_bits = fault_bits & VCU_BLOCKING_FAULT_MASK;
}

static void vcu_apply_outputs(void){
    if (s_state.fault_bits != 0u) {
        HAL_GPIO_WritePin(s_hw.error_led_port, s_hw.error_led_pin, GPIO_PIN_SET);
        // Error_Handler();
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

    if (s_hw.hadc_brake_press_sense == NULL || s_hw.hadc_brake_press_sense_ftr == NULL
        || s_hw.hadc_rc_timer_status == NULL || s_hw.brake_ll_led_port == NULL
        || s_hw.moter_5kw_led_port == NULL || s_hw.error_led_port == NULL
        || s_hw.heartbeat_led_port == NULL || s_hw.bspd_ll_port == NULL
        || s_hw.brakelight_ll_port == NULL || s_hw.motor_current_sense_port == NULL
        || s_hw.bspd_shutdown_sense_port == NULL || s_hw.brake_ll_led_pin == 0u
        || s_hw.moter_5kw_led_pin == 0u || s_hw.error_led_pin == 0u
        || s_hw.heartbeat_led_pin == 0u || s_hw.bspd_ll_pin == 0u
        || s_hw.brakelight_ll_pin == 0u || s_hw.motor_current_sense_pin == 0u
        || s_hw.bspd_shutdown_sense_pin == 0u) {
        s_state.mode = VCU_MODE_FAULT;
        return HAL_ERROR;
    }

    s_state.mode = VCU_MODE_NOT_READY;
    s_state.fault_bits = VCU_FAULT_NONE;
    s_state.blocking_fault_bits = VCU_FAULT_NONE;
    s_state.heartbeat = false;
    s_state.heartbeat_elapsed_ms = 0u;
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
