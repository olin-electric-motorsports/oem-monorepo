#ifndef AIR_H
#define AIR_H

#include "stm32g4xx_hal.h"

/*
 * AIR control board, Mark VIII: pin assignments and limits for air.c.
 *
 * An AIR (accumulator isolation relay) is one of the two high-voltage
 * contactors, AIR+ and AIR-, that connect the battery pack to the rest of the
 * car. The top of air.c explains what the board does with them.
 *
 * ============================== PINS ==============================
 *
 * Every pin below was traced on the PCB, from the pad of the microcontroller
 * (U1, an STM32G441KBT6) to the part it drives or reads. The PCB is
 * vehicle/mkviii/hardware/hv/air_control on branch sarahs/air_control. The
 * board plugs into the HV motherboard (branch avereyk/hv_motherboard) through
 * its card-edge connector J2; where a signal leaves the board, its J2 pin is
 * given.
 *
 * Each signal has a <NAME>_GPIO_Port and a <NAME>_Pin, the pair that the
 * HAL's GPIO functions take.
 */

/*
 * Outputs. Driving the pin high turns the thing on. They are all low (off)
 * from reset until air.c decides otherwise.
 */

// Precharge relay command. PA3 (net PRECHARGE_IN) goes through two transistor
// stages, Q1 then Q2, that shift it up to 5 V, and leaves the board on J2 A8
// for the precharge/discharge board. Each stage inverts the signal, so it
// comes out the right way round: high = precharge relay on.
// The relay's coil is powered by the shutdown circuit, so the relay can only
// close while the shutdown circuit is closed, whatever this pin says.
// Unlike the AIR+ driver below, Q1's gate has no pull-down resistor, so this
// output is undefined for the instant the microcontroller spends in reset,
// until gpio_init() drives the pin low.
#define PRECHARGE_GPIO_Port         GPIOA // PRECHARGE_IN -> J2 A8 -> precharge relay
#define PRECHARGE_Pin               GPIO_PIN_3

// AIR+ low-side driver ("LSD"). PA10 drives the gate of transistor Q3, which
// connects the low side of the AIR+ coil to ground (J2 A10 and A11). The high
// side of that coil is fed by the shutdown circuit. So AIR+ closes only when
// this pin is high AND the shutdown circuit is closed. A 1k resistor (R15)
// holds the gate low while the microcontroller is in reset.
// The PCB labels this net AIR-_LSD, but the transistor's output net is
// AIR+_LSD_OUT and the motherboard wires it to the AIR+ contactor.
#define AIR_P_LSD_GPIO_Port         GPIOA // PCB net AIR-_LSD, but Q3 switches AIR+ (J2 A10/A11)
#define AIR_P_LSD_Pin               GPIO_PIN_10

// Status LEDs on the board. Each LED sits between its pin and ground through
// a 1k resistor, so high = lit.
//   ERROR     (D1, red)   lit while the board is in FAULT
//   HEARTBEAT (D2, amber) blinks while the main loop is running
//   INIT      (D3, amber) lit during the startup checks
#define ERROR_LED_GPIO_Port         GPIOA
#define ERROR_LED_Pin               GPIO_PIN_0
#define HEARTBEAT_LED_GPIO_Port     GPIOA
#define HEARTBEAT_LED_Pin           GPIO_PIN_1
#define INIT_LED_GPIO_Port          GPIOA
#define INIT_LED_Pin                GPIO_PIN_2

/*
 * Shutdown circuit senses. Inputs, LOW = that point of the circuit is live.
 *
 * The shutdown circuit is a chain of switches in series that carries the
 * power for the AIR coils: open any switch and both AIRs lose power. The
 * motherboard sends several points along that chain to this board (J2 B12 to
 * B18) so the firmware can see how far the chain is closed.
 *
 * Each point drives the gate of a transistor (Q4 to Q10) that pulls the
 * microcontroller pin LOW when the point is live; otherwise a 10k resistor
 * pulls the pin up to 3.3 V. air.c flips the reading, so in the code and on
 * CAN, ss_xxx = true means "closed".
 *
 * SS_TSMS is the important one. It is the END of the chain: on the
 * motherboard it is the net FINAL_SHDN, the same net that feeds both AIR
 * coils. If it reads closed, the coils have power. It is the only sense the
 * state machine acts on; the others are only reported on CAN, to show which
 * part of the chain is open. (TSMS = tractive system master switch, IMD =
 * insulation monitoring device, TSMP = tractive system measuring points,
 * HVD = high-voltage disconnect.)
 *
 * The PCB has a seventh sense, SHDN_SENSE_EMETER, on PB0. air.yml has no
 * signal for it, so the firmware doesn't read it.
 */
#define SS_TSMS_GPIO_Port           GPIOA // J2 B12, via Q8
#define SS_TSMS_Pin                 GPIO_PIN_4
#define SS_IMD_GPIO_Port            GPIOA // J2 B13, via Q7
#define SS_IMD_Pin                  GPIO_PIN_5
#define SS_MPC_GPIO_Port            GPIOA // J2 B15, via Q5
#define SS_MPC_Pin                  GPIO_PIN_6
#define SS_TSMP_GPIO_Port           GPIOA // J2 B16, via Q4
#define SS_TSMP_Pin                 GPIO_PIN_7
#define SS_BMS_GPIO_Port            GPIOA // J2 B14, via Q6
#define SS_BMS_Pin                  GPIO_PIN_15
#define SS_HVD_GPIO_Port            GPIOB // J2 B18, via Q10
#define SS_HVD_Pin                  GPIO_PIN_3

/*
 * AIR auxiliary contacts ("weld detect"). Inputs, HIGH = contactor closed.
 *
 * Each AIR has a small extra contact that moves with its main contacts. The
 * motherboard puts 5 V on one side of it and sends the other side to this
 * board (J2 B6 for AIR+, J2 B7 for AIR-), where a 10k resistor pulls the
 * line to ground. So the pin is high while the contactor is closed and low
 * while it is open. (This assumes the extra contact closes when the main
 * contacts close, as on Mark VI and VII.)
 *
 * This is how the firmware knows what a contactor is really doing. An AIR
 * that reads closed when it should be open is welded shut, or its sense wire
 * is stuck high; either way it is a fault.
 */
#define AIR_P_WELD_DETECT_GPIO_Port GPIOA // J2 B6
#define AIR_P_WELD_DETECT_Pin       GPIO_PIN_9
#define AIR_N_WELD_DETECT_GPIO_Port GPIOA // J2 B7
#define AIR_N_WELD_DETECT_Pin       GPIO_PIN_8

/*
 * IMD status. Input, HIGH = insulation OK.
 *
 * The IMD (insulation monitoring device) watches for a leak between the
 * high-voltage system and the chassis. Its status comes from the motherboard
 * (net IMD_STATUS, J2 A9) through a resistor divider (R1 and R2) that also
 * pulls the pin down, so an unplugged wire reads "not OK".
 *
 * High = OK is carried over from Mark VI and VII. The IMD latch card that
 * drives this signal has not been traced, so confirm it on the bench: with
 * the wrong polarity a healthy car faults with IMD_STATUS 4 s after power-up.
 */
#define IMD_SENSE_GPIO_Port         GPIOB
#define IMD_SENSE_Pin               GPIO_PIN_4

/*
 * CAN. FDCAN1 on PA11 (receive) and PA12 (transmit), to the MCP2561
 * transceiver (U2). HAL_FDCAN_MspInit() in air.c sets these two pins up.
 */
#define CAN_GPIO_Port               GPIOA
#define CAN_RX_Pin                  GPIO_PIN_11
#define CAN_TX_Pin                  GPIO_PIN_12

/*
 * ============================== LIMITS ==============================
 *
 * Voltages are in decivolts (dV, 0.1 V), so 2000 means 200 V. That is the
 * unit the inverter reports its DC bus voltage in (DC_Bus_Voltage in
 * M167_Voltage_Info), which saves converting. Times are in milliseconds.
 * Unless a comment says otherwise, the values are the Mark VII AIR board's.
 *
 * "DC link" below means the high-voltage bus on the car side of the AIRs
 * (the inverter's input capacitors). "Pack" means the battery side.
 */

// The board refuses to start if the pack is below this (BMS_VOLTAGE fault)
#define BMS_VOLTAGE_THRESHOLD_LOW_dV (2000) // 200 V

// The DC link counts as discharged below this. Required at power-up
// (otherwise TRACTIVE_VOLTAGE) and to finish DISCHARGE.
#define TRACTIVE_THRESHOLD_LOW_dV    (50)   // 5 V

// Precharge has worked if the DC link is above this share of the pack
// voltage; only then may AIR+ close. (The FSAE rules ask for at least 90%.)
#define PRECHARGE_THRESHOLD_PERCENT  (95)   // of pack voltage

// Time for an AIR to read closed: AIR- after TSMS closes, AIR+ in TS_ACTIVE.
// Also how long an AIR that should be closed may read open before it faults.
#define AIR_CLOSE_TIMEOUT_MS (200)

// Time delay during discharge before checking state of AIRs. It gives the
// contactors time to open after their coils lose power.
#define WELD_CHECK_DELAY_MS (100)

// Milliseconds to wait while the IMD output stabilizes before reading it.
// The IMD needs a few seconds after power-up before its output means anything.
#define IMD_STABILITY_CHECK_DELAY_MS (4000)

// Precharge runs for this long, then the DC link voltage is checked once
#define PRECHARGE_DELAY_MS   (5000)  // TS voltage reaches 95% in 5 s
// After the shutdown circuit opens, the DC link must be discharged by this
// time (otherwise DISCHARGE_FAIL)
#define DISCHARGE_TIMEOUT_MS (10000) // TS voltage falls below 5 V in 10 s

// FSAE EV.8.2.2.c: TS voltage under 60 V within 5 s of the shutdown circuit
// opening. 60 V is where the rules draw the line between low and high
// voltage. (Not from Mark VII, which only had the 10 s check above.)
#define DISCHARGE_LOW_VOLTAGE_dV (600)
#define DISCHARGE_LOW_VOLTAGE_MS (5000)

// Oldest CAN data accepted. A voltage older than this is treated as missing,
// because acting on a stale reading is worse than faulting.
#define BMS_TIMEOUT_MS     (1000) // pack voltage, in INIT and PRECHARGE
#define MC_INIT_TIMEOUT_MS (1000) // in INIT
#define MC_TIMEOUT_MS      (500)  // in PRECHARGE and DISCHARGE

// How often air_control_critical is sent.
// About 16 Hz, the rate the Mark VI and VII boards actually sent at
// Keep this in step with freq_hz in air.yml (16), which is what the car's
// DBC tells everyone else to expect.
#define CAN_SEND_PERIOD_MS (62)

// The heartbeat LED changes state this often, so it blinks once a second
#define HEARTBEAT_PERIOD_MS (500)

/*
 * Watchdog: a hardware timer inside the microcontroller that resets the chip
 * unless the main loop keeps restarting ("refreshing") it. See `watchdog` in
 * air.c for why the board has one.
 *
 * It runs from its own slow oscillator (about 32 kHz), so it keeps working
 * even if the main clock stops. Dividing that by 32 makes it count about
 * once a millisecond, and it resets the chip after counting WATCHDOG_RELOAD
 * + 1 = 250 times with no refresh: about 250 ms. That oscillator is only
 * accurate to several percent, so the real figure is roughly 235 to 270 ms.
 *
 * The main loop refreshes it many times every millisecond, so 250 ms with no
 * refresh can only mean the firmware has stopped. (Not from Mark VII, which
 * had no watchdog.)
 */
#define WATCHDOG_PRESCALER (IWDG_PRESCALER_32)
#define WATCHDOG_RELOAD    (249)

#endif // AIR_H
