/* the pin table: which port and pin each signal uses. The firmware still
 * refuses to start until AIR_BOARD_CONFIGURED is set (see air_board_init) */
#include "air_board.h"
#include <stddef.h>

/* Pins for the STM32G441 dev board (vehicle/examples/hardware/stm32_g4_dev_board)
 * wired to the rig Arduino for tests/hardware/air_hw_test.py. EXT_SIG_n are its
 * test points; IMD latch, MPC, TSMP and e-meter sit on unconnected pins held closed
 * (low) by the pull-down. Replace them with the AIR PCB's pins once its
 * schematic is final. active_level means: output asserted / shutdown CLOSED /
 * contactor CLOSED / IMD healthy. LEDs may be {NULL, 0}; every other row is
 * required. tests/board_test.c checks whatever table is here */
const air_pin_t air_pins[AIR_PIN_COUNT] = {
    /* Signal                       port  pin           active          pull           AF */
    [AIR_PIN_PRECHARGE_CTL]   = {GPIOA, GPIO_PIN_9,  GPIO_PIN_SET,   GPIO_NOPULL,   0}, /* EXT_SIG_1 */
    [AIR_PIN_MAIN_CTL]        = {GPIOA, GPIO_PIN_10, GPIO_PIN_SET,   GPIO_NOPULL,   0}, /* EXT_SIG_2 */
    [AIR_PIN_SS_TSMS]         = {GPIOA, GPIO_PIN_15, GPIO_PIN_RESET, GPIO_NOPULL,   0}, /* EXT_SIG_3 */
    [AIR_PIN_SS_IMD_LATCH]    = {GPIOA, GPIO_PIN_3,  GPIO_PIN_RESET, GPIO_PULLDOWN, 0},
    [AIR_PIN_SS_MPC]          = {GPIOA, GPIO_PIN_4,  GPIO_PIN_RESET, GPIO_PULLDOWN, 0},
    [AIR_PIN_SS_TSMP]         = {GPIOA, GPIO_PIN_5,  GPIO_PIN_RESET, GPIO_PULLDOWN, 0},
    [AIR_PIN_SS_HVD]          = {GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET, GPIO_NOPULL,   0}, /* EXT_SIG_4 */
    [AIR_PIN_SS_BMS]          = {GPIOB, GPIO_PIN_4,  GPIO_PIN_RESET, GPIO_NOPULL,   0}, /* EXT_SIG_5 */
    [AIR_PIN_SS_EMETER]       = {GPIOA, GPIO_PIN_6,  GPIO_PIN_RESET, GPIO_PULLDOWN, 0},
    [AIR_PIN_IMD_SENSE]       = {GPIOB, GPIO_PIN_5,  GPIO_PIN_SET,   GPIO_NOPULL,   0}, /* EXT_SIG_6 */
    [AIR_PIN_AIR_P_FEEDBACK]  = {GPIOB, GPIO_PIN_6,  GPIO_PIN_SET,   GPIO_NOPULL,   0}, /* EXT_SIG_7 */
    [AIR_PIN_AIR_N_FEEDBACK]  = {GPIOB, GPIO_PIN_7,  GPIO_PIN_SET,   GPIO_NOPULL,   0}, /* EXT_SIG_8 */
    [AIR_PIN_ERROR_LED]       = {GPIOA, GPIO_PIN_2,  GPIO_PIN_SET,   GPIO_NOPULL,   0}, /* PWM_TEST LED */
    [AIR_PIN_HEARTBEAT_LED]   = {GPIOA, GPIO_PIN_1,  GPIO_PIN_SET,   GPIO_NOPULL,   0}, /* HEARTBEAT LED */
    [AIR_PIN_INIT_LED]        = {GPIOA, GPIO_PIN_8,  GPIO_PIN_SET,   GPIO_NOPULL,   0}, /* INTERRUPT_TEST_SW LED */
    [AIR_PIN_CAN_RX]          = {GPIOA, GPIO_PIN_11, GPIO_PIN_SET,   GPIO_NOPULL,   GPIO_AF9_FDCAN1},
    [AIR_PIN_CAN_TX]          = {GPIOA, GPIO_PIN_12, GPIO_PIN_SET,   GPIO_NOPULL,   GPIO_AF9_FDCAN1},
};
