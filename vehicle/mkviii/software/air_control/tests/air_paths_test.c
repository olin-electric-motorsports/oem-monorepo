/* Directed tests for every decision in air.c and air_protocol.c that
 * air_test.c does not reach. Together the two cover every branch;
 * tests/coverage.sh checks that claim. */
#include "vehicle/mkviii/software/air_control/tests/air_harness.h"

static void test_init_ignores_imd_until_settled(void) {
    power_up();
    in.imd_ok = false;
    step(AIR_IMD_SETTLE_MS - 1);
    CHECK(a.state == AIR_STATE_INIT && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(AIR_FAULT_IMD_STATUS);
}

static void test_init_waits_for_first_can_frames(void) {
    power_up();
    d.bms_seen = d.ivt_seen = false;
    step(AIR_IMD_SETTLE_MS);
    CHECK(a.state == AIR_STATE_INIT && a.fault == AIR_FAULT_NONE);
    step(AIR_STARTUP_CAN_WAIT_MS - 1);
    CHECK(a.state == AIR_STATE_INIT && a.fault == AIR_FAULT_NONE);
    d.bms_seen = d.ivt_seen = true;
    step(1);
    CHECK(a.state == AIR_STATE_IDLE);
}

static void test_init_faults_when_ivt_never_heard(void) {
    power_up();
    d.ivt_seen = false;
    step(AIR_IMD_SETTLE_MS);
    CHECK(a.state == AIR_STATE_INIT && a.fault == AIR_FAULT_NONE);
    step(AIR_STARTUP_CAN_WAIT_MS);
    expect_fault(AIR_FAULT_CAN_GMETER_TIMEOUT);
}

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

static void test_init_faults_when_dc_link_not_empty(void) {
    power_up();
    d.tractive_mv = AIR_TRACTIVE_SAFE_MV;
    step(AIR_IMD_SETTLE_MS);
    expect_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
}

static void test_fault_state_without_a_code_gets_one(void) {
    to_idle();
    a.state = AIR_STATE_FAULT; /* corrupted: FAULT with no cause recorded */
    step(1);
    expect_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
}

static void test_idle_faults_when_driven_air_closed(void) {
    to_idle();
    set_driven(true);
    step(1);
    expect_fault(WELD_DRIVEN);
}

static void test_idle_faults_when_loop_air_closed_with_loop_open(void) {
    to_idle();
    set_loop_air(true);
    step(1);
    expect_fault(WELD_LOOP);
}

static void test_idle_faults_when_final_node_closes_before_upstream(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_HVD;
    step(1);
    expect_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
}

static void test_idle_faults_when_loop_closes_on_charged_dc_link(void) {
    to_idle();
    d.tractive_mv = AIR_TRACTIVE_CHARGED_MV;
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    expect_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
}

/* IDLE only faults at AIR_TRACTIVE_CHARGED_MV, so a cycle may start just below it. */
static void test_idle_starts_cycle_just_below_charged_line(void) {
    to_idle();
    d.tractive_mv = AIR_TRACTIVE_CHARGED_MV - 1;
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED && a.fault == AIR_FAULT_NONE);
}

/* Leaving DISCHARGE needs a reading under 5 V, but IDLE faults only at 6 V,
 * so a reading that wobbles back over 5 V after a normal discharge (as with
 * the invariant model's 0.3 V of noise) does not latch TRACTIVE_VOLTAGE. */
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
    d.tractive_mv = AIR_TRACTIVE_CHARGED_MV;
    step(1);
    expect_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
}

static void test_idle_waits_quietly(void) {
    to_idle();
    run_ms(10000);
    CHECK(a.state == AIR_STATE_IDLE && a.fault == AIR_FAULT_NONE);
    CHECK(!a.main_command && !a.precharge_command);
}

static void test_loop_opening_before_air_closes_aborts(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED);
    open_loop();
}

/* A node that reads open for less than AIR_SHUTDOWN_DEBOUNCE_MS is treated as
 * a glitch: TS_ACTIVE keeps its state and both commands throughout. */
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

/* In a real opening the AIRs lose coil power at once and may drop out before
 * the debounce ends. That is the loop opening, not a feedback fault. */
static void test_air_dropout_during_debounce_is_not_a_fault(void) {
    to_active();
    in.shutdown_closed = 0;
    in.air_p_closed = in.air_n_closed = false; /* worst case: both read open at once */
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.fault == AIR_FAULT_NONE && a.main_command);
    step(1);
    CHECK(a.state == AIR_STATE_DISCHARGE && !a.main_command && !a.precharge_command);
}

/* Commands already on are held through a glitch, but nothing new turns on. */
static void test_glitch_delays_the_next_command(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    set_loop_air(true);
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_IMD;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED && !a.precharge_command);
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_PRECHARGE && a.precharge_command);
    d.tractive_mv = PRECHARGED_MV;
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_BMS;
    step(1);
    CHECK(a.state == AIR_STATE_PRECHARGE && a.precharge_command && !a.main_command);
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command && a.precharge_command);
}

/* A glitch that outlasts the debounce still aborts. The loop never really
 * opened, so it keeps its own AIR closed and the discharge circuit off: not a
 * weld, and no discharge deadline runs. The car waits, without drive, until
 * the TSMS is cycled. */
static void test_long_glitch_waits_for_tsms_cycle(void) {
    to_active();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_MPC;
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS);
    CHECK(a.state == AIR_STATE_DISCHARGE && !a.main_command && !a.precharge_command);
    in.shutdown_closed = AIR_SS_ALL;
    set_driven(false); /* the driven AIR opens as commanded; DC link stays charged */
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

/* While the loop reads closed its own AIR may close again, but this board's
 * AIR must still open within AIR_CONTACTOR_OPEN_MS of its command dropping. */
static void test_driven_air_must_open_while_loop_reads_closed(void) {
    to_active();
    in.shutdown_closed = AIR_SS_ALL & ~AIR_SS_HVD;
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS);
    CHECK(a.state == AIR_STATE_DISCHARGE);
    in.shutdown_closed = AIR_SS_ALL; /* the driven AIR stays closed: welded */
    run_ms(AIR_CONTACTOR_OPEN_MS - 1);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(WELD_DRIVEN);
}

/* Closing the loop again during DISCHARGE (an e-stop pulled back out) closes
 * the loop's AIR again and turns the discharge circuit off. Neither is a
 * fault; the discharge deadlines restart when the loop opens again. */
static void test_reclose_during_discharge_restarts_deadlines(void) {
    to_active();
    open_loop();
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = 100000; /* still draining */
    run_ms(500);
    CHECK(a.state == AIR_STATE_DISCHARGE);
    in.shutdown_closed = AIR_SS_ALL;
    set_loop_air(true);
    run_ms(AIR_DISCHARGE_TIMEOUT_MS); /* held at 100 V with the discharge circuit off */
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    in.shutdown_closed = 0;
    set_loop_air(false);
    run_ms(AIR_DISCHARGE_LIMIT_MS - 1);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(AIR_FAULT_DISCHARGE_FAIL);
}

/* FSAE 2026 EV.7.2.2.c: 60 V or less within 5 s of the shutdown circuit
 * opening. Still at 60 V or more by then means the discharge circuit failed. */
static void test_discharge_must_reach_60v_within_5s(void) {
    to_active();
    open_loop(); /* the loop last read closed AIR_SHUTDOWN_DEBOUNCE_MS ago */
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = AIR_DISCHARGE_LIMIT_MV;
    run_ms(AIR_DISCHARGE_LIMIT_MS - AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(AIR_FAULT_DISCHARGE_FAIL);
}

/* Under 60 V at 5 s meets the rule; the 5 V deadline at 10 s still applies. */
static void test_discharge_under_60v_has_until_10s_for_5v(void) {
    to_active();
    open_loop();
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = AIR_DISCHARGE_LIMIT_MV - 1;
    run_ms(AIR_DISCHARGE_TIMEOUT_MS - AIR_SHUTDOWN_DEBOUNCE_MS - 1);
    CHECK(a.state == AIR_STATE_DISCHARGE && a.fault == AIR_FAULT_NONE);
    step(1);
    expect_fault(AIR_FAULT_DISCHARGE_FAIL);
}

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

static void test_driven_air_closing_uncommanded_faults(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    set_driven(true);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

static void test_precharge_faults_when_loop_air_drops(void) {
    to_precharge();
    set_loop_air(false);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

static void test_precharge_faults_when_driven_air_closes_early(void) {
    to_precharge();
    set_driven(true);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

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

static void test_active_faults_when_loop_air_drops(void) {
    to_active();
    set_loop_air(false);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

/* air_test.c drops AIR- by name, which is the loop's AIR in the legacy
 * topology; this covers the driven AIR dropping in both topologies. */
static void test_active_faults_when_driven_air_drops(void) {
    to_active();
    set_driven(false);
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

/* Documents current behavior, not a confirmed requirement: BMS state 1
 * (CHARGING) counts as healthy, so the tractive system can be energized while
 * the BMS reports charging. That may be intended, since charging needs the
 * AIRs closed. Change this test if the team decides otherwise. */
static void test_bms_charging_counts_as_healthy(void) {
    to_precharge();
    d.bms_state = 1;
    d.tractive_mv = PRECHARGED_MV;
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command);
}

static void test_decoder_ignores_unknown_and_malformed_frames(void) {
    air_measurements_t m = {0};
    const uint8_t other_result[6] = {2, 0, 0, 5, 0xcc, 0x60}; /* IVT frame, not U1 */
    const uint8_t too_long[7] = {1, 0, 0, 5, 0xcc, 0x60, 0};
    CHECK(!air_decode(&m, AIR_CAN_IVT_ID, other_result, sizeof other_result, 1));
    CHECK(!air_decode(&m, AIR_CAN_IVT_ID, too_long, sizeof too_long, 2));
    CHECK(!air_decode(&m, 0x123u, too_long, AIR_CAN_IVT_BYTES, 3));
    CHECK(!m.ivt_seen && !m.bms_seen && m.ivt_ms == 0 && m.tractive_mv == 0);
}

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
    puts("AIR: all 31 path tests passed");
    return 0;
}
