#include "air.h"
#include "air_config.h"

static void enter(air_t *a, air_state_t state, uint32_t now) {
    a->state = state;
    a->entered_ms = now;
    a->contactor_confirmed = false;
}

void air_init(air_t *a, uint32_t now) {
    *a = (air_t){.state = AIR_STATE_INIT, .entered_ms = now};
}

void air_trip(air_t *a, air_fault_t fault) {
    if (a->fault == AIR_FAULT_NONE) a->fault = fault;
    a->state = AIR_STATE_FAULT;
    a->main_command = false;
    a->precharge_command = false;
}

static bool check_open(air_t *a, const air_inputs_t *in) {
    if (in->air_p_closed && in->air_n_closed)
        air_trip(a, AIR_FAULT_BOTH_AIRS_WELD);
    else if (in->air_p_closed) air_trip(a, AIR_FAULT_AIR_P_WELD);
    else if (in->air_n_closed) air_trip(a, AIR_FAULT_AIR_N_WELD);
    return a->fault == AIR_FAULT_NONE;
}

static bool check_data(air_t *a, const air_measurements_t *d, uint32_t now) {
    if (!d->bms_seen || (uint32_t)(now - d->bms_ms) >= AIR_BMS_TIMEOUT_MS)
        air_trip(a, AIR_FAULT_CAN_BMS_TIMEOUT);
    else if (!d->ivt_seen || (uint32_t)(now - d->ivt_ms) >= AIR_IVT_TIMEOUT_MS)
        air_trip(a, AIR_FAULT_CAN_GMETER_TIMEOUT);
    else if (d->bms_fault || d->bms_state >= 2 || d->pack_mv < AIR_PACK_MIN_MV)
        air_trip(a, AIR_FAULT_BMS_VOLTAGE);
    else if (d->ivt_error) air_trip(a, AIR_FAULT_IVT_STATUS);
    else if (d->tractive_mv < 0) air_trip(a, AIR_FAULT_TRACTIVE_VOLTAGE);
    return a->fault == AIR_FAULT_NONE;
}

void air_step(air_t *a, const air_inputs_t *in,
              const air_measurements_t *d, uint32_t now) {
    const uint32_t elapsed = now - a->entered_ms;
    const bool requested = (in->shutdown_closed & AIR_SS_TSMS) != 0;
    const bool shutdown_ok = in->shutdown_closed == AIR_SS_ALL;
    const bool controlled_closed = AIR_CONTROLLED_NEGATIVE ? in->air_n_closed : in->air_p_closed;
    const bool other_closed = AIR_CONTROLLED_NEGATIVE ? in->air_p_closed : in->air_n_closed;
    a->main_command = false;
    a->precharge_command = false;

    if (a->fault != AIR_FAULT_NONE || a->state == AIR_STATE_FAULT) {
        if (a->fault == AIR_FAULT_NONE) air_trip(a, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        a->state = AIR_STATE_FAULT;
        return;
    }

    if (a->state == AIR_STATE_INIT) {
        /* Never re-arm after a reset with the TSMS already closed. */
        if (requested) { air_trip(a, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY); return; }
        if (elapsed < AIR_IMD_SETTLE_MS) return;
        if (!in->imd_ok) { air_trip(a, AIR_FAULT_IMD_STATUS); return; }
        if ((!d->bms_seen || !d->ivt_seen) &&
            elapsed < AIR_IMD_SETTLE_MS + AIR_STARTUP_CAN_WAIT_MS) return;
        if (!check_data(a, d, now) || !check_open(a, in)) return;
        if ((uint32_t)d->tractive_mv >= AIR_TRACTIVE_SAFE_MV) {
            air_trip(a, AIR_FAULT_TRACTIVE_VOLTAGE); return;
        }
        enter(a, AIR_STATE_IDLE, now);
        return;
    }

    if (!in->imd_ok) { air_trip(a, AIR_FAULT_IMD_STATUS); return; }
    if (!check_data(a, d, now)) return;

    /* Aborting at ANY stage of contactor closure removes both commands now. */
    if ((a->state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED ||
         a->state == AIR_STATE_PRECHARGE || a->state == AIR_STATE_TS_ACTIVE) && !shutdown_ok) {
        enter(a, AIR_STATE_DISCHARGE, now);
        return;
    }

    switch (a->state) {
    case AIR_STATE_IDLE:
        if (controlled_closed) {
            air_trip(a, AIR_CONTROLLED_NEGATIVE ? AIR_FAULT_AIR_N_WELD : AIR_FAULT_AIR_P_WELD);
        } else if (!requested) {
            (void)check_open(a, in);
            if (a->fault == AIR_FAULT_NONE && (uint32_t)d->tractive_mv >= AIR_TRACTIVE_SAFE_MV)
                air_trip(a, AIR_FAULT_TRACTIVE_VOLTAGE);
        } else if (!shutdown_ok) {
            air_trip(a, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        } else if ((uint32_t)d->tractive_mv >= AIR_TRACTIVE_SAFE_MV) {
            air_trip(a, AIR_FAULT_TRACTIVE_VOLTAGE);
        } else {
            enter(a, AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED, now);
        }
        break;
    case AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED:
        if (controlled_closed) air_trip(a, AIR_FAULT_CONTACTOR_FEEDBACK);
        else if (other_closed) {
            enter(a, AIR_STATE_PRECHARGE, now);
            a->precharge_command = true;
        } else if (elapsed >= AIR_CONTACTOR_CLOSE_MS) air_trip(a, AIR_FAULT_CONTACTOR_FEEDBACK);
        break;
    case AIR_STATE_PRECHARGE:
        if (!other_closed || controlled_closed) {
            air_trip(a, AIR_FAULT_CONTACTOR_FEEDBACK);
        } else if (elapsed >= AIR_PRECHARGE_TIMEOUT_MS) {
            air_trip(a, AIR_FAULT_PRECHARGE_FAIL);
        } else if ((uint64_t)d->tractive_mv * 100u >= (uint64_t)d->pack_mv * AIR_PRECHARGE_PERCENT) {
            enter(a, AIR_STATE_TS_ACTIVE, now);
            a->main_command = true;
            a->precharge_command = true;
        } else a->precharge_command = true;
        break;
    case AIR_STATE_TS_ACTIVE:
        if (!other_closed || (a->contactor_confirmed && !controlled_closed) ||
            (!controlled_closed && elapsed >= AIR_CONTACTOR_CLOSE_MS)) {
            air_trip(a, AIR_FAULT_CONTACTOR_FEEDBACK);
        } else {
            if (controlled_closed) a->contactor_confirmed = true;
            a->main_command = true;
            /* Bridge the mechanical closing interval before removing precharge. */
            a->precharge_command = !a->contactor_confirmed;
        }
        break;
    case AIR_STATE_DISCHARGE:
        if (elapsed < AIR_CONTACTOR_OPEN_MS) break;
        if (!check_open(a, in)) break;
        if ((uint32_t)d->tractive_mv < AIR_TRACTIVE_SAFE_MV) {
            /* Require TSMS release before allowing a new cycle. */
            if (!requested) enter(a, AIR_STATE_IDLE, now);
        } else if (elapsed >= AIR_DISCHARGE_TIMEOUT_MS) air_trip(a, AIR_FAULT_DISCHARGE_FAIL);
        break;
    default:
        air_trip(a, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        break;
    }
}
