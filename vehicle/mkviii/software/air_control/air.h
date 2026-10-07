#ifndef AIR_H
#define AIR_H

#include <stdbool.h>
#include <stdint.h>

/* Preserve the legacy CAN enum numbers; append new faults only. */
typedef enum {
    AIR_FAULT_NONE, AIR_FAULT_AIR_N_WELD, AIR_FAULT_AIR_P_WELD,
    AIR_FAULT_BOTH_AIRS_WELD, AIR_FAULT_PRECHARGE_FAIL,
    AIR_FAULT_DISCHARGE_FAIL, AIR_FAULT_PRECHARGE_FAIL_RELAY_WELDED,
    AIR_FAULT_CAN_ERROR, AIR_FAULT_CAN_BMS_TIMEOUT,
    AIR_FAULT_CAN_GMETER_TIMEOUT, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY,
    AIR_FAULT_TRACTIVE_VOLTAGE, AIR_FAULT_BMS_VOLTAGE,
    AIR_FAULT_IMD_STATUS, AIR_FAULT_BOARD_CONFIG,
    AIR_FAULT_CONTACTOR_FEEDBACK, AIR_FAULT_IVT_STATUS
} air_fault_t;

typedef enum {
    AIR_STATE_INIT, AIR_STATE_IDLE, AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED,
    AIR_STATE_PRECHARGE, AIR_STATE_TS_ACTIVE, AIR_STATE_DISCHARGE,
    AIR_STATE_FAULT
} air_state_t;

enum {
    AIR_SS_TSMS = 1u << 0, AIR_SS_IMD = 1u << 1,
    AIR_SS_MPC = 1u << 2, AIR_SS_TSMP = 1u << 3,
    AIR_SS_HVD = 1u << 4, AIR_SS_BMS = 1u << 5,
    AIR_SS_EMETER = 1u << 6, AIR_SS_ALL = 0x7fu
};

/* Values are logical states AFTER conversion from board-specific polarity. */
typedef struct {
    uint8_t shutdown_closed;
    bool imd_ok, air_p_closed, air_n_closed;
} air_inputs_t;

typedef struct {
    bool bms_seen, ivt_seen;
    uint32_t bms_ms, ivt_ms, pack_mv;
    int32_t tractive_mv;
    uint16_t bms_fault;
    uint8_t bms_state;
    bool ivt_error;
} air_measurements_t;

typedef struct {
    air_state_t state;
    air_fault_t fault;
    uint32_t entered_ms;
    bool contactor_confirmed;
    bool main_command, precharge_command;
} air_t;

void air_init(air_t *air, uint32_t now);
void air_trip(air_t *air, air_fault_t fault);
void air_step(air_t *air, const air_inputs_t *in,
              const air_measurements_t *data, uint32_t now);

#endif
