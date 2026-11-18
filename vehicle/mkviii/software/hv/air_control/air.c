/*
 * AIR control firmware for the Mark VIII, ported from the Mark VII board.
 *
 * WHAT THE BOARD DOES
 *
 * Two contactors, AIR+ and AIR-, connect the battery pack to the rest of the
 * car. Closing both at once onto the inverter's empty capacitors (the "DC
 * link") would draw a huge current, so the DC link is first charged slowly
 * through a resistor. That is "precharge". This board runs that sequence:
 *
 *   1. The driver closes the shutdown circuit (the last switch is the TSMS).
 *      The shutdown circuit powers the AIR- coil directly, so AIR- closes on
 *      its own. This board has no say in that.
 *   2. The board turns the precharge relay on and waits 5 s.
 *   3. If the DC link has reached 95% of the pack voltage, the board closes
 *      AIR+ and turns precharge off. The car can now drive.
 *   4. When the shutdown circuit opens, both AIRs lose power. The board
 *      turns its AIR+ driver off, checks that both AIRs really opened (one
 *      that didn't is welded shut), and watches the DC link drain.
 *
 * The board can only ever WITHHOLD power: precharge and AIR+ both need the
 * shutdown circuit closed as well as this board's say-so. If the shutdown
 * circuit opens, the contactors open even if this firmware has stopped.
 *
 * STATES (air_state in the CAN message)
 *
 *   INIT --startup checks pass--> IDLE
 *   IDLE --TSMS closes--> SHUTDOWN_CIRCUIT_CLOSED
 *   SHUTDOWN_CIRCUIT_CLOSED --AIR- reads closed--> PRECHARGE
 *   PRECHARGE --5 s passed and DC link above 95%--> TS_ACTIVE
 *   SHUTDOWN_CIRCUIT_CLOSED, PRECHARGE, TS_ACTIVE --TSMS opens--> DISCHARGE
 *   DISCHARGE --both AIRs open and DC link below 5 V--> IDLE
 *   any state --any fault--> FAULT, which lasts until power is cycled
 *
 * Outputs follow the state: precharge is on only in PRECHARGE, AIR+ only in
 * TS_ACTIVE (see write_outputs()).
 *
 * FAULTS (air_fault in the CAN message). Only the first one is kept.
 *
 *   AIR_N_WELD     AIR- reads closed when it must be open: at power-up, or
 *                  100 ms after the shutdown circuit opened
 *   AIR_P_WELD     AIR+ reads closed while this board isn't driving it: at
 *                  power-up, in IDLE, SHUTDOWN_CIRCUIT_CLOSED and PRECHARGE,
 *                  or 100 ms into DISCHARGE
 *   BOTH_AIRS_WELD both read closed 100 ms after the shutdown circuit opened
 *   PRECHARGE_FAIL DC link not above 95% of the pack after 5 s of precharge
 *   DISCHARGE_FAIL DC link still at 60 V or more 5 s after the shutdown
 *                  circuit opened, or not below 5 V after 10 s
 *   PRECHARGE_FAIL_RELAY_WELDED
 *                  never set; kept so the numbers match Mark VI
 *   CAN_ERROR      CAN failed to start, or reading a received message failed
 *   CAN_BMS_TIMEOUT
 *                  no pack voltage within 1 s (at power-up and in PRECHARGE)
 *   CAN_MC_TIMEOUT no DC link voltage from the inverter: within 1 s at
 *                  power-up, or in the last 500 ms at the end of PRECHARGE
 *                  and in DISCHARGE
 *   SHUTDOWN_IMPLAUSIBILITY
 *                  the shutdown circuit and the contactors disagree: TSMS
 *                  already closed at power-up; AIR- not closed within 200 ms
 *                  of TSMS closing; AIR- or AIR+ open for 200 ms when it
 *                  should be closed. Also used when the state itself is
 *                  corrupt, or the watchdog fails to start.
 *   TRACTIVE_VOLTAGE
 *                  DC link above 5 V at power-up
 *   BMS_VOLTAGE    pack below 200 V at power-up
 *   IMD_STATUS     the IMD reports an insulation fault (checked from 4 s
 *                  after power-up)
 *
 * CAN
 *
 *   Sends     air_control_critical (ID 0xD): state, fault, and what every
 *             input reads. Defined in air.yml.
 *   Receives  M167_Voltage_Info from the inverter, for the DC link voltage.
 *   Missing   bms_core, for the pack voltage. See `pack` below.
 *
 * All CAN traffic goes through the generated API (can_api.h, built from
 * air.yml by //common/can_api), used the way common/can_api/README.md
 * describes. Each call's return value is checked (0 = success):
 *
 *   can_init_air_control()           once, in main()
 *   can_poll_receive_all()           every pass of the main loop, in
 *                                    receive_can(); it fills the global
 *                                    struct m167_voltage_info
 *   can_send_air_control_critical()  every 62 ms, from the main loop; it
 *                                    sends the global struct
 *                                    air_control_critical
 *
 * This file never touches the CAN peripheral directly, apart from
 * HAL_FDCAN_MspInit() below, which the HAL requires.
 *
 * Pins and limits are in air.h.
 */
#include <stdbool.h>
#include <stdint.h>

#include "air.h"
#include "can_api.h"

// Sets the system clock to 64 MHz from the board's 16 MHz crystal.
void SystemClockConfig(void); // common/startup.c

/*
 * A voltage from CAN, in 0.1 V, and when it arrived
 *
 * The arrival time matters as much as the value: a voltage that stopped
 * updating looks perfectly healthy unless its age is checked. fresh() does
 * that check.
 */
struct reading {
    bool seen;          // false until the first message arrives
    uint32_t time_ms;   // HAL_GetTick() when the latest message arrived
    int32_t voltage_dV; // the latest value, in 0.1 V
};

// DC link voltage, from M167_Voltage_Info
static struct reading tractive;

/*
 * Pack voltage, from bms_core. bms_core isn't in the mkviii DBC yet (there's
 * no Mark VIII BMS YAML), so nothing fills this in and INIT faults with
 * CAN_BMS_TIMEOUT. Once it's there: subscribe to bms_core in air.yml, fill
 * this in from it in receive_can(), and fault with BMS_VOLTAGE when its fault
 * code isn't 0, as Mark VII did.
 *
 * Until then the board cannot leave INIT, so it can never close a contactor.
 */
static struct reading pack;

/*
 * The CAN API doesn't say which messages arrived, so before each poll the DC
 * link voltage is set to a value the inverter never sends (-3276.8 V). If it
 * changed, a new M167_Voltage_Info came in.
 *
 * (can_poll_receive_all() just overwrites m167_voltage_info with whatever
 * arrived. Without this marker, a silent inverter and one that keeps sending
 * the same voltage would look identical.)
 */
#define NO_NEW_MESSAGE INT16_MIN

/*
 * Timestamps, all from HAL_GetTick() (milliseconds since power-up).
 *
 * Elapsed time is always computed as `now - then` on uint32_t. Unsigned
 * subtraction gives the right answer even when the counter wraps around to 0
 * after 49.7 days, which comparing two timestamps directly would not.
 */
static uint32_t state_start_ms;  // when the current state was entered
static uint32_t loop_closed_ms;  // last time TSMS read closed
static uint32_t air_p_closed_ms; // last time AIR+ read closed in TS_ACTIVE
static uint32_t air_n_closed_ms; // last time AIR- read closed in PRECHARGE or TS_ACTIVE
static bool startup_checks_passed; // INIT checks 1-4 passed

/*
 * The watchdog (settings in air.h). This is the HAL's handle for it.
 *
 * Why the board has one: if this firmware ever stops running (stuck in a
 * loop, crashed), the outputs stay frozen as they were, which could be
 * "precharge relay on", and the board goes silent on CAN. The shutdown
 * circuit would still open the contactors, but nothing would turn precharge
 * off or report the problem.
 *
 * The watchdog turns a stopped firmware into a reset. A reset turns the AIR+
 * driver off, and the firmware starts again from INIT with both outputs low.
 * If the shutdown circuit is still closed at that point, the startup check
 * "TSMS starts open" faults, so the board never re-arms by itself after a
 * reset.
 *
 * A crash also ends here: the default fault handler in common/startup.c is an
 * endless loop, which stops the main loop, which starves the watchdog.
 *
 * Once started, the watchdog cannot be stopped; only a reset stops it.
 */
static IWDG_HandleTypeDef watchdog;

/*
 * The 1 ms system tick. The HAL sets the SysTick timer to interrupt every
 * millisecond; this handler counts them, and HAL_GetTick() returns the
 * count. It is the only interrupt this firmware uses. The CAN examples in
 * this repo define theirs the same way.
 */
void SysTick_Handler(void) {
    HAL_IncTick();
}

/*
 * HAL_FDCAN_Init() (called by can_init_air_control()) calls this to set up
 * FDCAN1's pins and clock; nothing else in the repo does. The bit timing in
 * the generated can_api.c is 32 time quanta per bit with no prescaler, so the
 * 16 MHz crystal (HSE) gives 500 kbit/s.
 *
 * The other clock choices (the 64 MHz system clock) would give 2 Mbit/s with
 * that same timing, and the board would not be understood on the bus.
 *
 * The name and signature are fixed by the HAL: it has a "weak" empty version
 * of this function, and defining one here replaces it.
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan) {
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    // Feed the CAN peripheral from the crystal, then switch its clock on
    RCC_PeriphCLKInitTypeDef clock = {0};
    clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
    (void)HAL_RCCEx_PeriphCLKConfig(&clock); // can only fail for the RTC clock
    __HAL_RCC_FDCAN_CLK_ENABLE();

    // Hand PA11 and PA12 to the CAN peripheral ("alternate function" 9)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = CAN_RX_Pin | CAN_TX_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(CAN_GPIO_Port, &gpio);
}

/*
 * Outputs start low (off). Every input has its own pull resistor on the board.
 *
 * For an output, the level is written BEFORE the pin is switched to output
 * mode. Doing it the other way round could drive the pin high for an instant
 * and pulse a contactor driver.
 */
static void init_pin(GPIO_TypeDef* port, uint16_t pin, uint32_t mode) {
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pin;
    gpio.Mode = mode;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    if (mode == GPIO_MODE_OUTPUT_PP) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }
    HAL_GPIO_Init(port, &gpio);
}

/*
 * Sets up every pin except the two CAN pins (HAL_FDCAN_MspInit() does those).
 * A GPIO port does nothing until its clock is switched on, hence the first
 * two lines.
 */
static void gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Outputs: the two that matter (precharge, AIR+), then the LEDs
    init_pin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_MODE_OUTPUT_PP);
    init_pin(AIR_P_LSD_GPIO_Port, AIR_P_LSD_Pin, GPIO_MODE_OUTPUT_PP);
    init_pin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_MODE_OUTPUT_PP);
    init_pin(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, GPIO_MODE_OUTPUT_PP);
    init_pin(INIT_LED_GPIO_Port, INIT_LED_Pin, GPIO_MODE_OUTPUT_PP);

    // Inputs: shutdown circuit senses, AIR auxiliary contacts, IMD
    init_pin(SS_TSMS_GPIO_Port, SS_TSMS_Pin, GPIO_MODE_INPUT);
    init_pin(SS_IMD_GPIO_Port, SS_IMD_Pin, GPIO_MODE_INPUT);
    init_pin(SS_MPC_GPIO_Port, SS_MPC_Pin, GPIO_MODE_INPUT);
    init_pin(SS_TSMP_GPIO_Port, SS_TSMP_Pin, GPIO_MODE_INPUT);
    init_pin(SS_BMS_GPIO_Port, SS_BMS_Pin, GPIO_MODE_INPUT);
    init_pin(SS_HVD_GPIO_Port, SS_HVD_Pin, GPIO_MODE_INPUT);
    init_pin(AIR_P_WELD_DETECT_GPIO_Port, AIR_P_WELD_DETECT_Pin, GPIO_MODE_INPUT);
    init_pin(AIR_N_WELD_DETECT_GPIO_Port, AIR_N_WELD_DETECT_Pin, GPIO_MODE_INPUT);
    init_pin(IMD_SENSE_GPIO_Port, IMD_SENSE_Pin, GPIO_MODE_INPUT);
}

/*
 * Starts the watchdog. Returns false if the HAL reports that it didn't take
 * its settings (its slow oscillator isn't running, for example).
 */
static bool watchdog_start(void) {
    // Pause the watchdog whenever a debugger has halted the chip. Without
    // this, sitting at a breakpoint for 250 ms would reset the board.
    __HAL_DBGMCU_FREEZE_IWDG();

    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = WATCHDOG_PRESCALER;
    watchdog.Init.Reload = WATCHDOG_RELOAD;
    // The "window" option (refreshing too EARLY also resets) is not used
    watchdog.Init.Window = IWDG_WINDOW_DISABLE;
    return HAL_IWDG_Init(&watchdog) == HAL_OK;
}

// True if the pin reads high (3.3 V)
static bool pin_high(GPIO_TypeDef* port, uint16_t pin) {
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
}

// Drives an output high (on) or low (off)
static void write_pin(GPIO_TypeDef* port, uint16_t pin, bool on) {
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * Reads every input into air_control_critical, the struct that is sent on
 * CAN. The state machine reads its inputs back out of the same struct, so
 * what the board acts on and what it reports can never differ.
 *
 * Called once a millisecond. A contactor takes far longer than that to move,
 * so polling is fast enough and no pin interrupts are needed.
 */
static void read_inputs(void) {
    // Shutdown senses are low when closed (see air.h), so they are flipped
    // here: true = closed
    air_control_critical.ss_tsms = !pin_high(SS_TSMS_GPIO_Port, SS_TSMS_Pin);
    air_control_critical.ss_imd = !pin_high(SS_IMD_GPIO_Port, SS_IMD_Pin);
    air_control_critical.ss_mpc = !pin_high(SS_MPC_GPIO_Port, SS_MPC_Pin);
    // ss_hvd_conn is air.yml's name for the TSMP node
    air_control_critical.ss_hvd_conn = !pin_high(SS_TSMP_GPIO_Port, SS_TSMP_Pin);
    air_control_critical.ss_hvd = !pin_high(SS_HVD_GPIO_Port, SS_HVD_Pin);
    air_control_critical.ss_bms = !pin_high(SS_BMS_GPIO_Port, SS_BMS_Pin);

    // Auxiliary contacts are high when the contactor is closed: true = closed
    air_control_critical.air_p_status
        = pin_high(AIR_P_WELD_DETECT_GPIO_Port, AIR_P_WELD_DETECT_Pin);
    air_control_critical.air_n_status
        = pin_high(AIR_N_WELD_DETECT_GPIO_Port, AIR_N_WELD_DETECT_Pin);
    // IMD is high when the insulation is OK: true = OK
    air_control_critical.imd_status = pin_high(IMD_SENSE_GPIO_Port, IMD_SENSE_Pin);
}

/*
 * Only precharge in PRECHARGE and only AIR+ in TS_ACTIVE; off everywhere else
 *
 * This is the only place the outputs are driven after startup, and it works
 * from the state alone. So the state machine never has to remember to turn
 * something off: leaving a state is enough. In particular, both are off in
 * FAULT, and the two are never on together.
 */
static void write_outputs(void) {
    uint8_t state = air_control_critical.air_state;

    write_pin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, state == AIR_STATE_PRECHARGE);
    write_pin(AIR_P_LSD_GPIO_Port, AIR_P_LSD_Pin, state == AIR_STATE_TS_ACTIVE);
    write_pin(INIT_LED_GPIO_Port, INIT_LED_Pin, state == AIR_STATE_INIT);
    write_pin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, state == AIR_STATE_FAULT);
}

/*
 * FAULT is latched until power is cycled. Only the first fault is reported.
 *
 * The first fault is the useful one: once something has gone wrong, later
 * faults are usually just consequences of it. Nothing in this file clears a
 * fault or leaves the FAULT state.
 */
static void set_fault(enum air_fault_e the_fault) {
    if (air_control_critical.air_fault == AIR_FAULT_NONE) {
        air_control_critical.air_fault = the_fault;
    }
    air_control_critical.air_state = AIR_STATE_FAULT;
}

// Moves to a new state and restarts the "time in this state" timer
static void enter_state(enum air_state_e state, uint32_t now) {
    air_control_critical.air_state = state;
    state_start_ms = now;
}

// True if a reading has arrived and is no older than timeout_ms
static bool fresh(const struct reading* r, uint32_t now, uint32_t timeout_ms) {
    return r->seen && (now - r->time_ms) <= timeout_ms;
}

/*
 * For an AIR that should be closed: true once it has read open for 200 ms
 * straight. Anything shorter is the contact still closing or bouncing.
 *
 * last_closed_ms is the caller's timestamp for that AIR. It is brought up to
 * date on every call where the AIR reads closed, so "open for 200 ms" is just
 * "200 ms since that timestamp".
 */
static bool air_opened(bool closed, uint32_t* last_closed_ms, uint32_t now) {
    if (closed) {
        *last_closed_ms = now;
        return false;
    }
    return now - *last_closed_ms >= AIR_CLOSE_TIMEOUT_MS;
}

/*
 * Takes in whatever CAN messages have arrived and notes the DC link voltage
 * if a new one came. Called on every pass of the main loop, so the CAN
 * peripheral's small receive buffer (3 messages) never fills up.
 */
static void receive_can(uint32_t now) {
    // Plant the marker; see NO_NEW_MESSAGE above
    m167_voltage_info.dc_bus_voltage = NO_NEW_MESSAGE;

    // Generated API: unpacks every waiting message into its global struct.
    // Nonzero means the peripheral reported an error reading one.
    if (can_poll_receive_all() != 0) {
        set_fault(AIR_FAULT_CAN_ERROR);
        return;
    }

    // Marker gone: an M167_Voltage_Info arrived during this poll
    if (m167_voltage_info.dc_bus_voltage != NO_NEW_MESSAGE) {
        tractive.seen = true;
        tractive.time_ms = now;
        tractive.voltage_dV = m167_voltage_info.dc_bus_voltage;
    }
}

/*
 * Mark VII's initial checks, in its order. As on Mark VII, checks 1-4 pass
 * once at power-up; closing TSMS after that, while the IMD settles, is fine.
 *
 * Runs once a millisecond while the state is INIT. `elapsed` is the time
 * since power-up. Each check either faults, or returns to wait and try again
 * next millisecond, or falls through to the next check. The board leaves
 * INIT for IDLE only when all five have passed.
 */
static void initial_checks(uint32_t now, uint32_t elapsed) {
    if (!startup_checks_passed) {
        // 1) BMS is talking and the pack is above 200 V
        //    It has 1 s from power-up to be heard; until then, keep waiting.
        if (!fresh(&pack, now, BMS_TIMEOUT_MS)) {
            if (elapsed >= BMS_TIMEOUT_MS) {
                set_fault(AIR_FAULT_CAN_BMS_TIMEOUT);
            }
            return;
        }
        if (pack.voltage_dV < BMS_VOLTAGE_THRESHOLD_LOW_dV) {
            set_fault(AIR_FAULT_BMS_VOLTAGE);
            return;
        }

        // 2) TSMS starts open
        //    If the shutdown circuit is already closed when the board powers
        //    up (or resets), AIR- is already closed and the board has not
        //    seen the car in a safe state. It refuses to continue.
        if (air_control_critical.ss_tsms) {
            set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
            return;
        }

        // 3) Inverter is talking and the DC link is below 5 V
        //    A charged DC link with both AIRs supposedly open means
        //    something is connected that shouldn't be.
        if (!fresh(&tractive, now, MC_INIT_TIMEOUT_MS)) {
            if (elapsed >= MC_INIT_TIMEOUT_MS) {
                set_fault(AIR_FAULT_CAN_MC_TIMEOUT);
            }
            return;
        }
        if (tractive.voltage_dV > TRACTIVE_THRESHOLD_LOW_dV) {
            set_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
            return;
        }

        // 4) Both AIRs are open
        //    Nothing is powering them yet, so one that reads closed is welded.
        if (air_control_critical.air_p_status) {
            set_fault(AIR_FAULT_AIR_P_WELD);
            return;
        }
        if (air_control_critical.air_n_status) {
            set_fault(AIR_FAULT_AIR_N_WELD);
            return;
        }

        // Checks 1-4 are not repeated: from here the driver may close TSMS
        startup_checks_passed = true;
    }

    // 5) IMD is OK once it has settled
    //    Its output isn't meaningful for the first 4 s, so wait that long
    //    before reading it.
    if (elapsed < IMD_STABILITY_CHECK_DELAY_MS) {
        return;
    }
    if (!air_control_critical.imd_status) {
        set_fault(AIR_FAULT_IMD_STATUS);
        return;
    }

    enter_state(AIR_STATE_IDLE, now);
}

/*
 * One step of the state machine. Runs once a millisecond, after
 * read_inputs() and before write_outputs().
 *
 * It only decides the state (and records faults). It never touches a pin:
 * write_outputs() turns the state into pin levels straight afterwards.
 *
 * Within a state, the checks are an if / else-if chain, so their order is
 * their priority: the first one that applies wins and the rest are skipped
 * for this millisecond.
 */
static void state_machine_run(uint32_t now) {
    // How long the board has been in the current state
    uint32_t elapsed = now - state_start_ms;

    // A recorded fault always means the FAULT state. This also catches a
    // fault set outside this function (receive_can(), main()).
    if (air_control_critical.air_fault != AIR_FAULT_NONE) {
        air_control_critical.air_state = AIR_STATE_FAULT;
    }

    // After INIT, a bad IMD faults from any state
    // (INIT does its own IMD check, after the 4 s settling time.)
    if (air_control_critical.air_state != AIR_STATE_INIT
        && !air_control_critical.imd_status) {
        set_fault(AIR_FAULT_IMD_STATUS);
    }

    // Kept up to date in every state, so that DISCHARGE can tell how long
    // the shutdown circuit has been open
    if (air_control_critical.ss_tsms) {
        loop_closed_ms = now;
    }

    switch (air_control_critical.air_state) {
        case AIR_STATE_INIT: {
            // Power-up. Both outputs off. Waits for the startup checks.
            initial_checks(now, elapsed);
        } break;
        case AIR_STATE_IDLE: {
            // Shutdown circuit open, both AIRs open, both outputs off.
            if (air_control_critical.air_p_status) {
                // AIR+ isn't driven, so reading closed means it's welded
                set_fault(AIR_FAULT_AIR_P_WELD);
            } else if (air_control_critical.ss_tsms) {
                // Idle until shutdown circuit is closed
                enter_state(AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED, now);
            }
        } break;
        case AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED: {
            // The shutdown circuit closes AIR- by itself. It has 200 ms to
            // read closed, then precharge starts. AIR+ must stay open.
            if (!air_control_critical.ss_tsms) {
                // The driver changed their mind; make sure everything opens
                enter_state(AIR_STATE_DISCHARGE, now);
            } else if (air_control_critical.air_p_status) {
                // Still not driven. With AIR- closing too, a welded AIR+
                // connects the pack with no precharge.
                set_fault(AIR_FAULT_AIR_P_WELD);
            } else if (air_control_critical.air_n_status) {
                enter_state(AIR_STATE_PRECHARGE, now);
                // Start AIR-'s "must stay closed" timer from now
                air_n_closed_ms = now;
            } else if (elapsed >= AIR_CLOSE_TIMEOUT_MS) {
                // The circuit says closed but AIR- never moved
                set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
            }
        } break;
        case AIR_STATE_PRECHARGE: {
            // After 5 s of precharge, close AIR+ if the DC link is above 95%
            // of the pack voltage; otherwise fault. Until then AIR+ must
            // stay open and AIR- closed.
            //
            // The precharge output is on for the whole of this state. The
            // current path is pack -> precharge resistor -> DC link -> AIR-.
            if (!air_control_critical.ss_tsms) {
                enter_state(AIR_STATE_DISCHARGE, now);
            } else if (air_control_critical.air_p_status) {
                // AIR+ closed by itself: precharge is being bypassed
                set_fault(AIR_FAULT_AIR_P_WELD);
            } else if (air_opened(air_control_critical.air_n_status,
                                  &air_n_closed_ms, now)) {
                // AIR- dropped out, so nothing is being charged
                set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
            } else if (!fresh(&pack, now, BMS_TIMEOUT_MS)) {
                // No target to compare the DC link against
                set_fault(AIR_FAULT_CAN_BMS_TIMEOUT);
            } else if (elapsed < PRECHARGE_DELAY_MS) {
                // Still precharging
            } else if (!fresh(&tractive, now, MC_TIMEOUT_MS)) {
                // The 5 s are up but the DC link reading is too old to trust
                set_fault(AIR_FAULT_CAN_MC_TIMEOUT);
            } else if (tractive.voltage_dV * 100
                       > pack.voltage_dV * PRECHARGE_THRESHOLD_PERCENT) {
                // Same as "DC link > 95% of pack", in whole numbers.
                // Charged: write_outputs() now turns AIR+ on and precharge
                // off. AIR+ gets 200 ms from now to read closed.
                enter_state(AIR_STATE_TS_ACTIVE, now);
                air_p_closed_ms = now;
            } else {
                // Not charged after 5 s: a short on the DC link, a broken
                // precharge circuit, or a wrong reading. Do not close AIR+.
                set_fault(AIR_FAULT_PRECHARGE_FAIL);
            }
        } break;
        case AIR_STATE_TS_ACTIVE: {
            // If any of the shutdown nodes open, SS_TSMS opens as well (it is
            // the last node in the shutdown circuit). AIR+ has 200 ms to read
            // closed, and then both AIRs must stay closed.
            //
            // The AIR+ output is on for the whole of this state: the car is
            // live and can drive. The || below stops at the first true, which
            // is fine because either one is the same fault.
            if (!air_control_critical.ss_tsms) {
                enter_state(AIR_STATE_DISCHARGE, now);
            } else if (air_opened(air_control_critical.air_n_status,
                                  &air_n_closed_ms, now)
                       || air_opened(air_control_critical.air_p_status,
                                     &air_p_closed_ms, now)) {
                set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
            }
        } break;
        case AIR_STATE_DISCHARGE: {
            // AIR+ is off. After 100 ms both AIRs must read open, and the DC
            // link must fall below 60 V within 5 s and below 5 V within 10 s.
            //
            // If the shutdown circuit closes again (TSMS back on), AIR-
            // closes with it, which isn't a weld, and the discharge circuit
            // is off. The AIR- and DC link checks wait, and their timers
            // start over when the circuit opens again.
            //
            // Two clocks are used here:
            //   elapsed  time since this state began, i.e. since this board
            //            stopped driving AIR+
            //   open_ms  time since TSMS last read closed, i.e. how long the
            //            shutdown circuit has been open without a break. It
            //            is 0 while the circuit reads closed again.
            // AIR+ is judged on the first, AIR- and the DC link on the
            // second. To drive again after closing the circuit early, the
            // driver must open it until this state finishes.
            uint32_t open_ms = now - loop_closed_ms;
            bool loop_open = open_ms > WELD_CHECK_DELAY_MS;
            bool air_p_closed = air_control_critical.air_p_status;
            bool air_n_closed = air_control_critical.air_n_status;

            if (elapsed <= WELD_CHECK_DELAY_MS) {
                // AIRs still opening
            } else if (air_p_closed && air_n_closed && loop_open) {
                set_fault(AIR_FAULT_BOTH_AIRS_WELD);
            } else if (air_p_closed) {
                // Not driven for over 100 ms and still closed
                set_fault(AIR_FAULT_AIR_P_WELD);
            } else if (!loop_open) {
                // Shutdown circuit closed again, or AIR- still opening
            } else if (air_n_closed) {
                // No power for over 100 ms and still closed
                set_fault(AIR_FAULT_AIR_N_WELD);
            } else if (open_ms >= DISCHARGE_TIMEOUT_MS) {
                // 10 s and still not below 5 V
                set_fault(AIR_FAULT_DISCHARGE_FAIL);
            } else if (!fresh(&tractive, now, MC_TIMEOUT_MS)) {
                // Can't tell whether the DC link is draining
                set_fault(AIR_FAULT_CAN_MC_TIMEOUT);
            } else if (tractive.voltage_dV < TRACTIVE_THRESHOLD_LOW_dV) {
                // Both AIRs open and the DC link is empty: done
                enter_state(AIR_STATE_IDLE, now);
            } else if (open_ms >= DISCHARGE_LOW_VOLTAGE_MS
                       && tractive.voltage_dV >= DISCHARGE_LOW_VOLTAGE_dV) {
                // 5 s and still at 60 V or more: the discharge circuit is
                // too slow for the rules
                set_fault(AIR_FAULT_DISCHARGE_FAIL);
            }
        } break;
        case AIR_STATE_FAULT: {
            // Latched; write_outputs() keeps precharge and AIR+ off. A FAULT
            // with no fault code (corrupted memory) gets one here.
            // (set_fault() leaves an existing code alone.)
            set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        } break;
        default: {
            // Shouldn't happen, but just in case
            // (air_state holds a number that isn't a state.)
            set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        } break;
    }
}

/*
 * Setup, then the main loop. The loop never waits for anything, so one pass
 * takes a tiny fraction of a millisecond, and each pass does these jobs:
 *
 *   every pass      refresh the watchdog, take in CAN messages
 *   once per ms     read inputs -> state machine -> write outputs
 *   every 62 ms     send air_control_critical
 *   every 500 ms    toggle the heartbeat LED
 */
int main(void) {
    // HAL setup; also starts the 1 ms tick
    HAL_Init();
    // Outputs go low first, so precharge and AIR+ are off even if the clock
    // setup below stops in ErrorHandler()
    // (which is what happens if the 16 MHz crystal doesn't start).
    gpio_init();
    SystemClockConfig();

    uint32_t now = HAL_GetTick();
    enter_state(AIR_STATE_INIT, now);

    // Generated API: sets up and starts CAN. Nonzero means it failed.
    bool can_ok = (can_init_air_control() == 0);
    if (!can_ok) {
        // Without CAN there's no DC link voltage, so the AIRs must stay open
        // The loop below still runs, to hold the outputs off and show the
        // fault on the ERROR LED, but it leaves CAN alone.
        set_fault(AIR_FAULT_CAN_ERROR);
    }

    // The watchdog is started last, so the slow parts of setup above (the
    // crystal starting, CAN starting) can't trip it.
    if (!watchdog_start()) {
        // A board whose watchdog won't start has a clock or chip problem;
        // don't let it close a contactor
        set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
    }

    // When each timed job last ran
    uint32_t last_step_ms = now;
    uint32_t last_send_ms = now;
    uint32_t last_heartbeat_ms = now;

    while (1) {
        // Tell the watchdog the loop is alive. This is the only place it is
        // refreshed, so anything that stops the loop for 250 ms resets the
        // board. (The call cannot fail; it always returns HAL_OK.)
        (void)HAL_IWDG_Refresh(&watchdog);

        now = HAL_GetTick();

        if (can_ok) {
            receive_can(now);
        }

        // The state machine runs once per 1 ms tick
        // (`now` only changes once a millisecond, so this is true on one
        // pass in many.)
        if (now != last_step_ms) {
            last_step_ms = now;
            read_inputs();
            state_machine_run(now);
            write_outputs();
        }

        // Sent from here, never from an interrupt, as the CAN API requires.
        // read_inputs() and the state machine have already filled in the
        // fields of air_control_critical; this packs them and queues the
        // message.
        if (can_ok && now - last_send_ms >= CAN_SEND_PERIOD_MS) {
            last_send_ms = now;
            if (can_send_air_control_critical() != 0) {
                // Sending failed: the message couldn't be queued, which
                // means nothing is getting out on the bus.
                // A failed send isn't a fault, as on Mark VI and VII
                // The board carries on and tries again in 62 ms.
            }
        }

        // A blinking LED shows this loop is still running
        if (now - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS) {
            last_heartbeat_ms = now;
            HAL_GPIO_TogglePin(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin);
        }
    }

    return 0;
}
