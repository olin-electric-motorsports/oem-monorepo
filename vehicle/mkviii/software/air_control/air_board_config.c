#include "air_board.h"
#include <stddef.h>

/* Replace NULL and 0 with GPIOx and GPIO_PIN_n from YOUR schematic.
 * active_level means: output asserted / shutdown CLOSED / contactor CLOSED /
 * IMD healthy. Legacy polarities below are provisional, not board assignments.
 * LEDs may remain unassigned; all other rows are required. */
const air_pin_t air_pins[AIR_PIN_COUNT] = {
    /* Signal                       port  pin active           pull         AF */
    [AIR_PIN_PRECHARGE_CTL]   = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, 0},
    [AIR_PIN_MAIN_CTL]        = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, 0},
    [AIR_PIN_SS_TSMS]         = {NULL, 0, GPIO_PIN_RESET, GPIO_NOPULL, 0},
    [AIR_PIN_SS_IMD_LATCH]    = {NULL, 0, GPIO_PIN_RESET, GPIO_NOPULL, 0},
    [AIR_PIN_SS_MPC]          = {NULL, 0, GPIO_PIN_RESET, GPIO_NOPULL, 0},
    [AIR_PIN_SS_TSMP]         = {NULL, 0, GPIO_PIN_RESET, GPIO_NOPULL, 0},
    [AIR_PIN_SS_HVD]          = {NULL, 0, GPIO_PIN_RESET, GPIO_NOPULL, 0},
    [AIR_PIN_SS_BMS]          = {NULL, 0, GPIO_PIN_RESET, GPIO_NOPULL, 0},
    [AIR_PIN_SS_EMETER]       = {NULL, 0, GPIO_PIN_RESET, GPIO_NOPULL, 0},
    [AIR_PIN_IMD_SENSE]       = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, 0},
    [AIR_PIN_AIR_P_FEEDBACK]  = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, 0},
    [AIR_PIN_AIR_N_FEEDBACK]  = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, 0},
    [AIR_PIN_ERROR_LED]       = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, 0},
    [AIR_PIN_HEARTBEAT_LED]   = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, 0},
    [AIR_PIN_INIT_LED]        = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, 0},
    [AIR_PIN_CAN_RX]          = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, GPIO_AF9_FDCAN1},
    [AIR_PIN_CAN_TX]          = {NULL, 0, GPIO_PIN_SET,   GPIO_NOPULL, GPIO_AF9_FDCAN1},
};
