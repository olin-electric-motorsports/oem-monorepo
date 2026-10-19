/* Known issues. Each check describes what the firmware SHOULD do.
 *
 * While an issue is still present this test reports it and passes, so CI
 * stays green. Once a fix makes a check pass, this test fails on purpose:
 * move that check into air_paths_test.c as a normal test and delete it here. */
#include "vehicle/mkviii/software/air_control/tests/air_harness.h"

/* FSAE 2026 EV.7.2.2.c: within 5 s of the shutdown circuit opening, the
 * tractive system must be at 60 V or less. A DC link still above 60 V at 5 s
 * means the discharge circuit failed, so the firmware should fault by then.
 * Today it only checks for 5 V, after 10 s. */
static bool discharge_is_checked_at_60v_within_5s(void) {
    to_active();
    in.shutdown_closed = 0;
    step(1);
    CHECK(a.state == AIR_STATE_DISCHARGE);
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = 100000; /* stuck at 100 V */
    run_ms(5000 + 100);     /* 100 ms margin for one late measurement */
    return a.state == AIR_STATE_FAULT && a.fault == AIR_FAULT_DISCHARGE_FAIL;
}

/* One noisy 1 ms sample on a shutdown input drops both commands, which is
 * the safe reaction. But the loop never opened, so it still holds its AIR
 * closed, and that should not be reported as a weld. */
static bool input_glitch_is_not_reported_as_weld(void) {
    to_active();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_MPC; /* one bad sample */
    step(1);
    CHECK(!a.main_command && !a.precharge_command);
    in.shutdown_closed = AIR_SS_ALL;
    set_driven(false); /* the driven AIR opens as commanded */
    run_ms(200);
    return a.fault != WELD_LOOP && a.fault != AIR_FAULT_BOTH_AIRS_WELD;
}

/* Closing the loop again while the DC link is still discharging (for example
 * pulling an e-stop back out) makes the loop close its AIR again. That should
 * not be reported as a weld. */
static bool reclose_during_discharge_is_not_reported_as_weld(void) {
    to_active();
    in.shutdown_closed = 0;
    step(1);
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = 100000; /* still draining */
    run_ms(500);
    CHECK(a.state == AIR_STATE_DISCHARGE);
    in.shutdown_closed = AIR_SS_ALL;
    set_loop_air(true);
    run_ms(200);
    return a.fault != WELD_LOOP && a.fault != AIR_FAULT_BOTH_AIRS_WELD;
}

/* Leaving DISCHARGE needs a reading under 5 V, and IDLE faults on a reading
 * of 5 V or more, with no margin between them. One reading that wobbles back
 * across 5 V just after a normal discharge latches TRACTIVE_VOLTAGE.
 * air_invariant_test hits this regularly with 0.3 V of measurement noise. */
static bool voltage_noise_at_5v_does_not_fault(void) {
    to_active();
    in.shutdown_closed = 0;
    step(1);
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = 5100;
    run_ms(AIR_CONTACTOR_OPEN_MS);
    d.tractive_mv = 4990; /* discharge finishing */
    step(1);
    CHECK(a.state == AIR_STATE_IDLE);
    d.tractive_mv = 5010; /* one reading 20 mV back over the line */
    step(1);
    d.tractive_mv = 4900;
    run_ms(100);
    return a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE;
}

int main(void) {
    static const struct {
        const char *summary;
        bool (*fixed)(void);
    } issues[] = {
        {"discharge is not checked against 60 V within 5 s (EV.7.2.2.c)",
         discharge_is_checked_at_60v_within_5s},
        {"a 1 ms glitch on one shutdown input is reported as a welded AIR",
         input_glitch_is_not_reported_as_weld},
        {"closing the loop again during discharge is reported as a welded AIR",
         reclose_during_discharge_is_not_reported_as_weld},
        {"a 20 mV wobble across 5 V after discharge latches TRACTIVE_VOLTAGE",
         voltage_noise_at_5v_does_not_fault},
    };
    int fixed = 0;
    for (size_t i = 0; i < sizeof issues / sizeof issues[0]; ++i) {
        if (issues[i].fixed()) {
            printf("FIXED: %s. Move this check into air_paths_test.c.\n", issues[i].summary);
            ++fixed;
        } else {
            printf("KNOWN ISSUE: %s\n", issues[i].summary);
        }
    }
    return fixed ? 1 : 0;
}
