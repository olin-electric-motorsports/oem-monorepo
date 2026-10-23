/* tests for what is not in air_test.c Together, they cover all bases */
#include "air_harness.h"

/* A bad IMD is ignored for the first 4 s while it settles, then it faults */
static void test_init_ignores_imd_until_settled(void) {
    power_up();
    in.imd_ok = false;
    step(AIR_IMD_SETTLE_MS - 1);
    CHECK(a.state == AIR_STATE_INIT && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(AIR_FAULT_IMD_STATUS);
}

/* Startup waits up to 1 s more for the first BMS and IVT messages */
static void test_init_waits_for_first_can_frames(void) {
    power_up();
    d.bms_seen = d.ivt_seen = false; /* nothing heard yet */
    step(AIR_IMD_SETTLE_MS);
    CHECK(a.state == AIR_STATE_INIT && a.fault == AIR_FAULT_NONE);
    step(AIR_STARTUP_CAN_WAIT_MS - 1);
    CHECK(a.state == AIR_STATE_INIT && a.fault == AIR_FAULT_NONE);
    d.bms_seen = d.ivt_seen = true; /* they show up just in time */
    step(1);
    CHECK(a.state == AIR_STATE_IDLE);
}

/* No IVT message by the end of that wait is a fault */
static void test_init_faults_when_ivt_never_heard(void) {
    power_up();
    d.ivt_seen = false;
    step(AIR_IMD_SETTLE_MS);
    CHECK(a.state == AIR_STATE_INIT && a.fault == AIR_FAULT_NONE);
    step(AIR_STARTUP_CAN_WAIT_MS);
    expect_fault(AIR_FAULT_CAN_GMETER_TIMEOUT);
}

/* An AIR that reads closed at startup is reported as welded: ours, the loop's, or both */
static void test_init_faults_on_welded_air(void) {
    const struct { bool driven, loop_air; air_fault_t fault; } cases[] = {
        {true, false, WELD_DRIVEN},
        {false, true, WELD_LOOP},
        {true, true, AIR_FAULT_BOTH_AIRS_WELD},
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
        power_up();
        set_driven(cases[i].driven);
        set_loop_air(cases[i].loop_air);
        step(AIR_IMD_SETTLE_MS);
        expect_fault(cases[i].fault);
    }
}

/* Startup faults if the DC link still has 5 V or more on it */
static void test_init_faults_when_dc_link_not_empty(void) {
    power_up();
    d.tractive_mv = AIR_TRACTIVE_SAFE_MV;
    step(AIR_IMD_SETTLE_MS);
    expect_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
}

/* FAULT with no fault code saved (corrupted memory) still gets one */
static void test_fault_state_without_a_code_gets_one(void) {
    to_idle();
    a.state = AIR_STATE_FAULT; /* corrupted: FAULT with no cause recorded */
    step(1);
    expect_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
}

/* In IDLE our AIR should be open; reading closed means it's welded */
static void test_idle_faults_when_driven_air_closed(void) {
    to_idle();
    set_driven(true);
    step(1);
    expect_fault(WELD_DRIVEN);
}

/* Same for the loop's AIR while the loop is open */
static void test_idle_faults_when_loop_air_closed_with_loop_open(void) {
    to_idle();
    set_loop_air(true);
    step(1);
    expect_fault(WELD_LOOP);
}

/* TSMS is the last node, so it can't really read closed while an earlier
 * node is open. If that lasts 3 ms, fault */
static void test_idle_faults_when_final_node_closes_before_upstream(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_HVD;
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
}

/* Shorter than 3 ms is treated as noise on the TSMS input, and the car can
 * still start normally afterwards */
static void test_idle_ignores_short_implausible_reading(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL & ~(AIR_SS_HVD | AIR_SS_TSMS); /* HVD out */
    run_ms(1000);
    in.shutdown_closed |= AIR_SS_TSMS; /* TSMS reads closed, HVD still open */
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
    CHECK(!a.main_command && !a.precharge_command);
    in.shutdown_closed &= ~AIR_SS_TSMS;
    run_ms(1000);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
    in.shutdown_closed = AIR_SS_ALL; /* HVD back in, TSMS on */
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED && a.fault == AIR_FAULT_NONE);
}

/* Nodes that close together can read closed a sample apart, so the cycle
 * starts once all 7 read closed */
static void test_idle_starts_cycle_after_short_implausible_reading(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_BMS; /* the BMS node reads closed late */
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED && a.fault == AIR_FAULT_NONE);
}

/* Closing the loop with 6 V or more on the DC link is a fault */
static void test_idle_faults_when_loop_closes_on_charged_dc_link(void) {
    to_idle();
    d.tractive_mv = AIR_TRACTIVE_CHARGED_MV;
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    expect_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
}

/* Just under 6 V is fine */
static void test_idle_starts_cycle_just_below_charged_line(void) {
    to_idle();
    d.tractive_mv = AIR_TRACTIVE_CHARGED_MV - 1;
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED && a.fault == AIR_FAULT_NONE);
}

/* Leaving DISCHARGE needs under 5 V but IDLE only faults at 6 V, so noise
 * right after a normal discharge doesn't fault */
static void test_noise_after_discharge_does_not_fault(void) {
    to_active();
    open_loop();
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = AIR_TRACTIVE_SAFE_MV + 100;
    run_ms(AIR_CONTACTOR_OPEN_MS);
    CHECK(a.state == AIR_STATE_DISCHARGE);
    d.tractive_mv = AIR_TRACTIVE_SAFE_MV - 10; /* discharge finishing */
    step(1);
    CHECK(a.state == AIR_STATE_IDLE);
    d.tractive_mv = AIR_TRACTIVE_CHARGED_MV - 1; /* the highest reading IDLE accepts */
    step(1);
    d.tractive_mv = AIR_TRACTIVE_SAFE_MV - 100;
    run_ms(100);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
    d.tractive_mv = AIR_TRACTIVE_CHARGED_MV; /* a real 6 V still faults */
    step(1);
    expect_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
}

/* Sitting in IDLE for 10 s does nothing */
static void test_idle_waits_quietly(void) {
    to_idle();
    run_ms(10000);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
    CHECK(!a.main_command && !a.precharge_command);
}

/* Opening the loop before the loop's AIR closes goes straight to DISCHARGE */
static void test_loop_opening_before_air_closes_aborts(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED);
    open_loop();
}

/* A node open for under 3 ms is just a glitch: TS_ACTIVE keeps going with
 * both outputs as they were */
static void test_short_glitch_keeps_ts_active(void) {
    to_active();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_MPC;
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command && !a.precharge_command);
    in.shutdown_closed = AIR_SS_ALL;
    run_ms(1000);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.fault == AIR_FAULT_NONE);
    CHECK(a.main_command && !a.precharge_command);
}

/* In a real opening the AIRs lose power at once and can drop out inside
 * those 3 ms. That's the loop opening, not a fault: it ends in DISCHARGE */
static void test_air_dropout_during_debounce_is_not_a_fault(void) {
    to_active();
    in.shutdown_closed = 0;
    in.air_p_closed = in.air_n_closed = false; /* worst case: both read open at once */
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.fault == AIR_FAULT_NONE && a.main_command);
    step(1);
    CHECK(a.state == AIR_STATE_DISCHARGE && !a.main_command && !a.precharge_command);
}

/* During a glitch, outputs already on stay on, but nothing new turns on */
static void test_glitch_delays_the_next_command(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    set_loop_air(true);
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_IMD; /* glitch: precharge waits */
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED && !a.precharge_command);
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_PRECHARGE && a.precharge_command);
    d.tractive_mv = PRECHARGED_MV;
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_BMS; /* glitch at 95%: our AIR waits */
    step(1);
    CHECK(a.state == AIR_STATE_PRECHARGE && a.precharge_command && !a.main_command);
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command && a.precharge_command);
}

/* A glitch over 3 ms still drops the TS. The loop never really opened, so
 * its AIR stays closed (not a weld) and the discharge circuit stays off (no
 * timers run). The car waits, with no drive, until the TSMS is cycled */
static void test_long_glitch_waits_for_tsms_cycle(void) {
    to_active();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_MPC;
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS);
    CHECK(a.state == AIR_STATE_DISCHARGE && !a.main_command && !a.precharge_command);
    in.shutdown_closed = AIR_SS_ALL;
    set_driven(false); /* our AIR opens as told; the DC link stays charged */
    run_ms(AIR_DISCHARGE_TIMEOUT_MS + 1000);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    CHECK(!a.main_command && !a.precharge_command);
    in.shutdown_closed = 0; /* TSMS off: the loop opens, then so does its AIR */
    set_loop_air(false);
    run_ms(AIR_CONTACTOR_OPEN_MS);
    d.tractive_mv = AIR_TRACTIVE_SAFE_MV - 1;
    step(1);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
}

/* Even with the loop closed again, our AIR must open within 100 ms */
static void test_driven_air_must_open_while_loop_reads_closed(void) {
    to_active();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_HVD;
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS);
    CHECK(a.state == AIR_STATE_DISCHARGE);
    in.shutdown_closed = AIR_SS_ALL; /* our AIR stays closed: welded */
    run_ms(AIR_CONTACTOR_OPEN_MS - 1);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(WELD_DRIVEN);
}

/* Closing the loop again mid-discharge (an e-stop pulled back out) closes its
 * AIR and turns the discharge circuit off. Not a fault: the 5 s and 10 s
 * timers start over when the loop opens again */
static void test_reclose_during_discharge_restarts_deadlines(void) {
    to_active();
    open_loop();
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = 100000; /* still draining */
    run_ms(500);
    CHECK(a.state == AIR_STATE_DISCHARGE);
    in.shutdown_closed = AIR_SS_ALL; /* e-stop pulled back out */
    set_loop_air(true);
    run_ms(AIR_DISCHARGE_TIMEOUT_MS); /* held at 100 V with the discharge circuit off */
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    in.shutdown_closed = 0; /* open again: the 5 s starts now */
    set_loop_air(false);
    run_ms(AIR_DISCHARGE_LIMIT_MS - 1);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(AIR_FAULT_DISCHARGE_FAIL);
}

/* FSAE EV.7.2.2.c: under 60 V within 5 s of the loop opening, or the
 * discharge circuit failed */
static void test_discharge_must_reach_60v_within_5s(void) {
    to_active();
    open_loop(); /* the loop opened 3 ms ago */
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = AIR_DISCHARGE_LIMIT_MV; /* stuck at exactly 60 V */
    run_ms(AIR_DISCHARGE_LIMIT_MS - AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    step(1); /* 5 s */
    expect_fault(AIR_FAULT_DISCHARGE_FAIL);
}

/* Under 60 V at 5 s is fine, but it still has to get under 5 V by 10 s */
static void test_discharge_under_60v_has_until_10s_for_5v(void) {
    to_active();
    open_loop();
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = AIR_DISCHARGE_LIMIT_MV - 1;
    run_ms(AIR_DISCHARGE_TIMEOUT_MS - AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    step(1); /* 10 s */
    expect_fault(AIR_FAULT_DISCHARGE_FAIL);
}

/* DISCHARGE gives the AIRs 100 ms to open, then goes to IDLE once under 5 V */
static void test_discharge_waits_for_contacts_then_voltage(void) {
    to_active();
    open_loop();
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = 100000;
    run_ms(AIR_CONTACTOR_OPEN_MS - 1); /* contacts still moving: no checks yet */
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    run_ms(1000); /* still draining, well inside the timeout */
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    CHECK(!a.main_command && !a.precharge_command);
    d.tractive_mv = AIR_TRACTIVE_SAFE_MV - 1;
    step(1);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
}

/* Our AIR closing before we tell it to is a fault */
static void test_driven_air_closing_uncommanded_faults(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    set_driven(true);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

/* The loop's AIR opening during precharge is a fault */
static void test_precharge_faults_when_loop_air_drops(void) {
    to_precharge();
    set_loop_air(false);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

/* Our AIR closing during precharge is a fault */
static void test_precharge_faults_when_driven_air_closes_early(void) {
    to_precharge();
    set_driven(true);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

/* Precharge stays on until our AIR is confirmed closed */
static void test_precharge_stays_on_until_driven_air_confirms(void) {
    to_precharge();
    d.tractive_mv = PRECHARGED_MV;
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command && a.precharge_command);
    step(AIR_CONTACTOR_CLOSE_MS - 1);
    CHECK(a.fault == AIR_FAULT_NONE && a.main_command && a.precharge_command);
    set_driven(true);
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command && !a.precharge_command);
}

/* The loop's AIR opening while driving is a fault */
static void test_active_faults_when_loop_air_drops(void) {
    to_active();
    set_loop_air(false);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

/* Same for our AIR (air_test.c only checks this in one wiring) */
static void test_active_faults_when_driven_air_drops(void) {
    to_active();
    set_driven(false);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

/* BMS "charging" counts as healthy, so the TS can turn on while charging.
 * That's what the code does today, not a confirmed requirement: change this
 * test if the team decides otherwise */
static void test_bms_charging_counts_as_healthy(void) {
    to_precharge();
    d.bms_state = 1;
    d.tractive_mv = PRECHARGED_MV;
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command);
}

/* The CAN decoder ignores wrong IDs, wrong lengths and other IVT results */
static void test_decoder_ignores_unknown_and_malformed_frames(void) {
    air_measurements_t m = {0};
    const uint8_t other_result[6] = {2, 0, 0, 5, 0xcc, 0x60}; /* IVT frame, not U1 */
    const uint8_t too_long[7] = {1, 0, 0, 5, 0xcc, 0x60, 0};
    CHECK(!air_decode(&m, AIR_CAN_IVT_ID, other_result, sizeof other_result, 1));
    CHECK(!air_decode(&m, AIR_CAN_IVT_ID, too_long, sizeof too_long, 2));
    CHECK(!air_decode(&m, 0x123u, too_long, AIR_CAN_IVT_BYTES, 3));
    CHECK(!m.ivt_seen && !m.bms_seen && m.ivt_ms == 0 && m.tractive_mv == 0);
}

/* Runs every test; the first failed CHECK prints its line and stops */
int main(void) {
    test_init_ignores_imd_until_settled();
    test_init_waits_for_first_can_frames();
    test_init_faults_when_ivt_never_heard();
    test_init_faults_on_welded_air();
    test_init_faults_when_dc_link_not_empty();
    test_fault_state_without_a_code_gets_one();
    test_idle_faults_when_driven_air_closed();
    test_idle_faults_when_loop_air_closed_with_loop_open();
    test_idle_faults_when_final_node_closes_before_upstream();
    test_idle_ignores_short_implausible_reading();
    test_idle_starts_cycle_after_short_implausible_reading();
    test_idle_faults_when_loop_closes_on_charged_dc_link();
    test_idle_starts_cycle_just_below_charged_line();
    test_noise_after_discharge_does_not_fault();
    test_idle_waits_quietly();
    test_loop_opening_before_air_closes_aborts();
    test_short_glitch_keeps_ts_active();
    test_air_dropout_during_debounce_is_not_a_fault();
    test_glitch_delays_the_next_command();
    test_long_glitch_waits_for_tsms_cycle();
    test_driven_air_must_open_while_loop_reads_closed();
    test_reclose_during_discharge_restarts_deadlines();
    test_discharge_must_reach_60v_within_5s();
    test_discharge_under_60v_has_until_10s_for_5v();
    test_discharge_waits_for_contacts_then_voltage();
    test_driven_air_closing_uncommanded_faults();
    test_precharge_faults_when_loop_air_drops();
    test_precharge_faults_when_driven_air_closes_early();
    test_precharge_stays_on_until_driven_air_confirms();
    test_active_faults_when_loop_air_drops();
    test_active_faults_when_driven_air_drops();
    test_bms_charging_counts_as_healthy();
    test_decoder_ignores_unknown_and_malformed_frames();
    puts("AIR: all 33 path tests passed");
    return 0;
}
