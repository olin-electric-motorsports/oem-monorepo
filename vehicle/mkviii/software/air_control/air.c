/* the AIR state machine: decides when to precharge, when to close our AIR and
 * when to fault. Plain logic with no hardware, so the tests run it on a PC */
#include "air.h"
#include "air_config.h"

/* Switch state and restart that state's timer */
static void enter(air_t *a, air_state_t state, uint32_t now) {
    a->state = state;
    a->entered_ms = now;
    a->contactor_confirmed = false;
}

/* Start fresh in INIT with both outputs off */
void air_init(air_t *a, uint32_t now) {
    *a = (air_t){.state = AIR_STATE_INIT, .entered_ms = now,
                 .loop_closed_ms = now, .plausible_ms = now};
}

/* Fault: go to FAULT with both outputs off. The first fault code sticks */
void air_trip(air_t *a, air_fault_t fault) {
    if (a->fault == AIR_FAULT_NONE) a->fault = fault;
    a->state = AIR_STATE_FAULT;
    a->main_command = false;
    a->precharge_command = false;
}

/* Both AIRs should read open; if not, fault naming the welded one(s) */
static bool check_open(air_t *a, const air_inputs_t *in) {
    if (in->air_p_closed && in->air_n_closed)
        air_trip(a, AIR_FAULT_BOTH_AIRS_WELD);
    else if (in->air_p_closed) air_trip(a, AIR_FAULT_AIR_P_WELD);
    else if (in->air_n_closed) air_trip(a, AIR_FAULT_AIR_N_WELD);
    return a->fault == AIR_FAULT_NONE;
}

/* BMS and IVT data must be recent and healthy, or fault */
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

/* Runs every 1 ms: looks at the inputs and CAN data, moves between states,
 * and sets the two outputs (our AIR and precharge) */
void air_step(air_t *a, const air_inputs_t *in,
              const air_measurements_t *d, uint32_t now) {
    const uint32_t elapsed = now - a->entered_ms; /* time in this state */
    const bool requested = (in->shutdown_closed & AIR_SS_TSMS) != 0; /* TSMS on */
    const bool shutdown_ok = in->shutdown_closed == AIR_SS_ALL; /* all 7 nodes closed */
    /* "controlled" is our AIR, "other" is the loop's AIR */
    const bool controlled_closed = AIR_CONTROLLED_NEGATIVE ? in->air_n_closed : in->air_p_closed;
    const bool other_closed = AIR_CONTROLLED_NEGATIVE ? in->air_p_closed : in->air_n_closed;
    const air_fault_t controlled_weld = AIR_CONTROLLED_NEGATIVE ? AIR_FAULT_AIR_N_WELD : AIR_FAULT_AIR_P_WELD;
    /* Last step's outputs, kept through a glitch */
    const bool main_was = a->main_command, precharge_was = a->precharge_command;
    if (shutdown_ok) a->loop_closed_ms = now;
    if (shutdown_ok || !requested) a->plausible_ms = now;
    /* How long at least one node has read open; 0 while all seven read closed. */
    const uint32_t open_ms = now - a->loop_closed_ms;
    /* How long TSMS has read closed while an upstream node reads open; else 0. */
    const uint32_t implausible_ms = now - a->plausible_ms;
    /* Outputs are off unless a state below turns them on */
    a->main_command = false;
    a->precharge_command = false;

    /* Once faulted, stay faulted with everything off until power is cycled */
    if (a->fault != AIR_FAULT_NONE || a->state == AIR_STATE_FAULT) {
        if (a->fault == AIR_FAULT_NONE) air_trip(a, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        a->state = AIR_STATE_FAULT;
        return;
    }

    /* INIT: wait for the IMD and CAN, then make sure everything is safe */
    if (a->state == AIR_STATE_INIT) {
        /* Never re-arm after a reset with the TSMS already closed. */
        if (requested) { air_trip(a, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY); return; }
        if (elapsed < AIR_IMD_SETTLE_MS) return; /* give the IMD 4 s */
        if (!in->imd_ok) { air_trip(a, AIR_FAULT_IMD_STATUS); return; }
        if ((!d->bms_seen || !d->ivt_seen) &&
            elapsed < AIR_IMD_SETTLE_MS + AIR_STARTUP_CAN_WAIT_MS) return; /* up to 1 s more for CAN */
        if (!check_data(a, d, now) || !check_open(a, in)) return;
        if ((uint32_t)d->tractive_mv >= AIR_TRACTIVE_SAFE_MV) { /* DC link must be empty */
            air_trip(a, AIR_FAULT_TRACTIVE_VOLTAGE); return;
        }
        enter(a, AIR_STATE_IDLE, now);
        return;
    }

    /* After startup the IMD and the CAN data are checked every step */
    if (!in->imd_ok) { air_trip(a, AIR_FAULT_IMD_STATUS); return; }
    if (!check_data(a, d, now)) return;

    /* Any node open for 3 ms drops the TS to DISCHARGE, whatever stage of
     * closing it's at. Shorter than that may be a glitch: keep the same state
     * and outputs and skip the AIR checks below (the IMD and CAN checks above
     * still run) */
    if ((a->state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED ||
         a->state == AIR_STATE_PRECHARGE || a->state == AIR_STATE_TS_ACTIVE) && !shutdown_ok) {
        if (open_ms >= AIR_SHUTDOWN_DEBOUNCE_MS) {
            enter(a, AIR_STATE_DISCHARGE, now);
        } else {
            a->main_command = main_was;
            a->precharge_command = precharge_was;
        }
        return;
    }

    switch (a->state) {
    /* IDLE: everything off, waiting for the driver to close the loop */
    case AIR_STATE_IDLE:
        if (controlled_closed) { /* our AIR closed by itself: welded */
            air_trip(a, controlled_weld);
        } else if (!requested) { /* loop open: both AIRs open, DC link under 6 V */
            (void)check_open(a, in);
            if (a->fault == AIR_FAULT_NONE && (uint32_t)d->tractive_mv >= AIR_TRACTIVE_CHARGED_MV)
                air_trip(a, AIR_FAULT_TRACTIVE_VOLTAGE);
        } else if (!shutdown_ok) {
            /* TSMS is the last node, so it can't really read closed while an
             * earlier node is open. Fault if that lasts 3 ms; shorter may be
             * noise, so keep waiting */
            if (implausible_ms >= AIR_SHUTDOWN_DEBOUNCE_MS)
                air_trip(a, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        } else if ((uint32_t)d->tractive_mv >= AIR_TRACTIVE_CHARGED_MV) {
            air_trip(a, AIR_FAULT_TRACTIVE_VOLTAGE); /* loop closed on a charged DC link */
        } else {
            enter(a, AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED, now); /* loop closed: start */
        }
        break;
    /* Loop closed: its AIR should close within 200 ms, then precharge starts.
     * Our AIR must stay open */
    case AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED:
        if (controlled_closed) air_trip(a, AIR_FAULT_CONTACTOR_FEEDBACK);
        else if (other_closed) {
            enter(a, AIR_STATE_PRECHARGE, now);
            a->precharge_command = true;
        } else if (elapsed >= AIR_CONTACTOR_CLOSE_MS) air_trip(a, AIR_FAULT_CONTACTOR_FEEDBACK);
        break;
    /* Precharge until the DC link reaches 95% of the pack (within 5 s), then
     * close our AIR. Either AIR changing by itself is a fault */
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
    /* Driving: our AIR on. Fault if the loop's AIR opens, or ours doesn't
     * close within 200 ms or drops out later */
    case AIR_STATE_TS_ACTIVE:
        if (!other_closed || (a->contactor_confirmed && !controlled_closed) ||
            (!controlled_closed && elapsed >= AIR_CONTACTOR_CLOSE_MS)) {
            air_trip(a, AIR_FAULT_CONTACTOR_FEEDBACK);
        } else {
            if (controlled_closed) a->contactor_confirmed = true;
            a->main_command = true;
            /* Keep precharge on until our AIR is confirmed closed */
            a->precharge_command = !a->contactor_confirmed;
        }
        break;
    /* Loop opened: both outputs off while the AIRs open and the DC link drains.
     * Our AIR must open within 100 ms. While the loop reads closed again (a
     * glitch ended, or an e-stop was pulled back out) its AIR may close and the
     * discharge circuit is off, so that AIR and the timers wait until the loop
     * opens again. Then: under 60 V by 5 s, under 5 V by 10 s, and TSMS off
     * before going back to IDLE */
    case AIR_STATE_DISCHARGE:
        if (elapsed < AIR_CONTACTOR_OPEN_MS) break; /* AIRs still opening */
        if (open_ms < AIR_CONTACTOR_OPEN_MS) { /* loop closed again, or only just reopened */
            if (controlled_closed) air_trip(a, controlled_weld);
            break;
        }
        if (!check_open(a, in)) break;
        if ((uint32_t)d->tractive_mv < AIR_TRACTIVE_SAFE_MV) {
            /* Only go back to IDLE once the TSMS is off */
            if (!requested) enter(a, AIR_STATE_IDLE, now);
        } else if (open_ms >= AIR_DISCHARGE_TIMEOUT_MS ||
                   (open_ms >= AIR_DISCHARGE_LIMIT_MS &&
                    (uint32_t)d->tractive_mv >= AIR_DISCHARGE_LIMIT_MV)) {
            air_trip(a, AIR_FAULT_DISCHARGE_FAIL);
        }
        break;
    default: /* a corrupted state value */
        air_trip(a, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        break;
    }
}
