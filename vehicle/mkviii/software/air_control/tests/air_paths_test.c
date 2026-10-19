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
    d.tractive_mv = AIR_TRACTIVE_SAFE_MV;
    in.shutdown_closed = AIR_SS_ALL;
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
    in.shutdown_closed = 0;
    step(1);
    CHECK(a.state == AIR_STATE_DISCHARGE && !a.main_command && !a.precharge_command);
}

static void test_discharge_waits_for_contacts_then_voltage(void) {
    to_active();
    in.shutdown_closed = 0;
    step(1);
    CHECK(a.state == AIR_STATE_DISCHARGE);
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
    test_idle_waits_quietly();
    test_loop_opening_before_air_closes_aborts();
    test_discharge_waits_for_contacts_then_voltage();
    test_driven_air_closing_uncommanded_faults();
    test_precharge_faults_when_loop_air_drops();
    test_precharge_faults_when_driven_air_closes_early();
    test_precharge_stays_on_until_driven_air_confirms();
    test_active_faults_when_loop_air_drops();
    test_active_faults_when_driven_air_drops();
    test_bms_charging_counts_as_healthy();
    test_decoder_ignores_unknown_and_malformed_frames();
    puts("AIR: all 21 path tests passed");
    return 0;
}
