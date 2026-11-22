/*
 * BMS firmware for the STM32G441KBT6 on the BMS Core board.
 */
#include "bms.h"
#include "bms_config.h"
#include "can_api.h"
#include "tasks/tasks.h"
#include "utils/cell_balancing.h"
#include "utils/fault.h"
#include "utils/i2c_helpers.h"
#include <string.h>
#include <stdbool.h>

#define LOOP_PERIOD_MS (10)

// How often (in 10ms loops) the slower jobs run
#define HEARTBEAT_LOOPS     (50)  // toggle every 500ms
#define SLOW_CAN_LOOPS      (50)  // bms_metrics and bms_debug at 2Hz
#define COOLING_CAN_LOOPS   (25)  // bms_cooling at 4Hz
#define BALANCING_LOOPS     (100) // rewrite the discharge switches every 1s
#define OPEN_WIRE_LOOPS     (500) // check for open wires every 5s

/*
 * Fan speed from the hottest thermistor reading. Readings are thermistor
 * voltages, so lower = hotter: off at or below 35degC, full speed at or
 * above 45degC, linear in between.
 */
static void cooling_fan_control(uint16_t max_temp) {
    uint32_t percent;

    if (max_temp <= SOFT_OVERTEMPERATURE_THRESHOLD) {
        percent = 100;
    } else if (max_temp >= SOFT_OVERTEMPERATURE_THRESHOLD_LOW) {
        percent = 0;
    } else {
        uint32_t range = SOFT_OVERTEMPERATURE_THRESHOLD_LOW - SOFT_OVERTEMPERATURE_THRESHOLD;
        percent = (uint32_t)(SOFT_OVERTEMPERATURE_THRESHOLD_LOW - max_temp) * 100 / range;
    }

    bms_cooling.duty_cycle = percent;

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period * percent / 100);
}

/*
 * Update the PEC error count for one task and set/clear BMS_FAULT_PEC.
 * Counts reset after a task reads back with no PEC errors.
 */
static void handle_pec_errors(uint16_t pec_errors, volatile uint8_t* error_count) {
    if (pec_errors != 0) {
        uint16_t count = *error_count + pec_errors;
        *error_count = (count > UINT8_MAX) ? UINT8_MAX : count;
    } else {
        *error_count = 0;
    }

    if (bms_metrics.voltage_pec_error_count >= MAX_PEC_ERROR_COUNT
        || bms_metrics.temperature_pec_error_count >= MAX_PEC_ERROR_COUNT) {
        set_fault(BMS_FAULT_PEC);
    } else {
        clear_fault(BMS_FAULT_PEC);
    }
}

static void monitor_cells(bool run_open_wire) {
    // read all temperatures. ot/ut/min/max only change once per sweep of
    // every mux channel, so they're kept between calls
    static uint32_t ot = 0;
    static uint32_t ut = 0;
    static uint16_t min_temp = 0;
    static uint16_t max_temp = UINT16_MAX;

    uint16_t pec_errors = 0;
    temperature_task(&ot, &ut, &min_temp, &max_temp, &pec_errors);
    handle_pec_errors(pec_errors, &bms_metrics.temperature_pec_error_count);

    // Check for undertemparature and overtemperature faults
    if (ut > MAX_EXTRANEOUS_TEMPERATURES) {
        set_fault(BMS_FAULT_UNDERTEMPERATURE);
    } else {
        clear_fault(BMS_FAULT_UNDERTEMPERATURE);
    }

    if (ot > MAX_EXTRANEOUS_TEMPERATURES) {
        set_fault(BMS_FAULT_OVERTEMPERATURE);
    } else {
        clear_fault(BMS_FAULT_OVERTEMPERATURE);
    }

    // Fan Control based on latest max_temp
    cooling_fan_control(max_temp);

    // read all voltages
    uint32_t ov = 0;
    uint32_t uv = 0;
    uint16_t pack_voltage = 0;
    pec_errors = 0;
    voltage_task(&pack_voltage, &ov, &uv, &pec_errors);
    handle_pec_errors(pec_errors, &bms_metrics.voltage_pec_error_count);
    bms_core.pack_voltage = pack_voltage;

    // Check for undervoltage and overvoltage faults
    if (ov > 0) {
        set_fault(BMS_FAULT_OVERVOLTAGE);
    } else {
        clear_fault(BMS_FAULT_OVERVOLTAGE);
    }

    // The bridged unused channels always read 0V
    if (uv > NUM_UNUSED_VOLTAGE_CHANNELS * NUM_ICS) {
        set_fault(BMS_FAULT_UNDERVOLTAGE);
    } else {
        clear_fault(BMS_FAULT_UNDERVOLTAGE);
    }

    // read current
    int16_t current = 0;
    current_task(&current);
    bms_core.pack_current = current;

    // Check for overcurrent fault
    if (current > CURRENT_THRESH) {
        set_fault(BMS_FAULT_OVERCURRENT);
        bms_core.overcurrent_detect = true;
    } else {
        clear_fault(BMS_FAULT_OVERCURRENT);
        bms_core.overcurrent_detect = false;
    }

    if (run_open_wire) {
        uint32_t ow = 0;
        openwire_task(&ow);
        if (ow > 0) {
            set_fault(BMS_FAULT_OPEN_WIRE);
        } else {
            clear_fault(BMS_FAULT_OPEN_WIRE);
        }
    }
}

int main(void) {
    // 1. Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // 2. Configure the system clock
    SystemClockConfig();

    // 3. Initialize GPIO (relay starts open)
    GpioInit();
    // Fan PWM
    TimerInit();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // 4. Initialize SPI 04/18/26
    oem_spi_init(&bms_spi);

    // Initializing ADCs
    oem_adc_init(&current_sense_vout);

    // If CAN fails to start, keep running so the relay logic still works and
    // show it on the error LED
    bool can_ok = (can_init_bms() == 0);

    // Initial SPI Wakeup
    // This sends a dummy byte to wake up the ADBMS1818s
    wakeup_sleep(NUM_ICS);
    mux_init(NUM_ICS);
    cell_balancing_init();

    // Run the first open wire check before the relay can close
    bool run_open_wire = true;

    // TODO: set from the charger once charging is implemented. Cell balancing
    // only runs while this is true.
    bool charging = false;

    uint16_t loop_counter = 0;
    uint32_t last_loop = HAL_GetTick();

    // Infinite Loop
    while (1) {
        if ((HAL_GetTick() - last_loop) < LOOP_PERIOD_MS) {
            continue;
        }
        last_loop = HAL_GetTick();

        bms_core.bspd_current_sense
            = HAL_GPIO_ReadPin(BSPD_CURRENT_THRESH_PORT, BSPD_CURRENT_THRESH_PIN);

        // Run Safety Tasks (From tasks)
        monitor_cells(run_open_wire);
        run_open_wire = false;

        // Relay Logic (Safety Shutdown)
        // If any fault is set, open the relay (PA8)
        if (!check_fault_state()) {
            HAL_GPIO_WritePin(BMS_RELAY_DRIVE_PORT, BMS_RELAY_DRIVE_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, can_ok ? GPIO_PIN_RESET : GPIO_PIN_SET);
            bms_core.relay_closed = true;
        } else {
            HAL_GPIO_WritePin(BMS_RELAY_DRIVE_PORT, BMS_RELAY_DRIVE_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_SET);
            bms_core.relay_closed = false;
        }

        // Green LED on while every CSC is answering with good data
        bool comms_ok = !get_fault(BMS_FAULT_CSC_MIA) && !get_fault(BMS_FAULT_PEC);
        HAL_GPIO_WritePin(CSC_COMMS_PORT, CSC_COMMS_PIN, comms_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);

        can_wait_for_tx_slot();
        can_send_bms_core();
        can_wait_for_tx_slot();
        can_send_bms_sense();

        loop_counter++;

        if (loop_counter % COOLING_CAN_LOOPS == 0) {
            can_wait_for_tx_slot();
            can_send_bms_cooling();
            can_wait_for_tx_slot();
            can_send_bms_mux();
        }

        if (loop_counter % SLOW_CAN_LOOPS == 0) {
            can_wait_for_tx_slot();
            can_send_bms_metrics();
            can_wait_for_tx_slot();
            can_send_bms_debug();
        }

        // Heartbeat logic (Toggle every 500ms = 50 loops)
        if (loop_counter % HEARTBEAT_LOOPS == 0) {
            HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
        }

        if (loop_counter % BALANCING_LOOPS == 0) {
            update_cell_balancing(charging && !check_fault_state());
        }

        if (loop_counter % OPEN_WIRE_LOOPS == 0) {
            run_open_wire = true;
            loop_counter = 0;
        }
    }

    return 0;
}
