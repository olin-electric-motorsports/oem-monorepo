/* the original tests from the STM32 port: normal drive cycles, every shutdown
 * input, the IMD, voltages, CAN timeouts, startup, and the CAN byte layout */
#include "air.h"
#include "air_config.h"
#include "air_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Like assert(), but still works when Bazel builds with -DNDEBUG */
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

/* The state machine, its inputs, its CAN data and the clock */
static air_t a;
static air_inputs_t in;
static air_measurements_t d;
static uint32_t now;

/* Move the clock forward in one jump, mark CAN data fresh, run one step */
static void step(uint32_t ms) {
    now += ms;
    d.bms_ms = now;
    d.ivt_ms = now;
    air_step(&a, &in, &d, now);
}

/* Power up at time `start` with healthy inputs and wait out startup */
static void reset_at(uint32_t start) {
    now = start;
    air_init(&a, now);
    in = (air_inputs_t){.imd_ok = true};
    d = (air_measurements_t){.bms_seen = true, .ivt_seen = true, .pack_mv = 400000};
    step(AIR_IMD_SETTLE_MS);
    CHECK(a.state == AIR_STATE_IDLE);
}

/* Get to PRECHARGE: close the loop, then the loop's AIR closes */
static void precharge(void) {
    reset_at(0);
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED);
    CHECK(!a.main_command && !a.precharge_command);
    if (AIR_CONTROLLED_NEGATIVE) in.air_p_closed = true;
    else in.air_n_closed = true;
    step(1);
    CHECK(a.state == AIR_STATE_PRECHARGE && a.precharge_command);
}

/* Get to TS_ACTIVE: precharge hits 95%, both AIRs closed, precharge off */
static void active(void) {
    precharge();
    d.tractive_mv = 380000;
    step(1);
    CHECK(a.main_command && a.precharge_command);
    in.air_p_closed = in.air_n_closed = true;
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command && !a.precharge_command);
}

/* Check we're in FAULT with this code and both outputs off */
static void expect_fault(air_fault_t f) {
    CHECK(a.state == AIR_STATE_FAULT && a.fault == f);
    CHECK(!a.main_command && !a.precharge_command);
}

/* A normal drive cycle (on, drive, off, discharge, back to IDLE), then a
 * second start where the loop's AIR never closes */
static void test_normal_and_repeated_cycles(void) {
    active();
    step(2000); /* Must not falsely time out after feedback is confirmed. */
    CHECK(a.state == AIR_STATE_TS_ACTIVE);
    in.shutdown_closed = 0;
    step(AIR_SHUTDOWN_DEBOUNCE_MS);
    CHECK(a.state == AIR_STATE_DISCHARGE && !a.main_command && !a.precharge_command);
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = 4999;
    step(AIR_CONTACTOR_OPEN_MS);
    CHECK(a.state == AIR_STATE_IDLE);
    in.shutdown_closed = AIR_SS_ALL; /* second start */
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED);
    step(AIR_CONTACTOR_CLOSE_MS - 1);
    CHECK(a.fault == AIR_FAULT_NONE);
    step(1); /* 200 ms and the loop's AIR still isn't closed */
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
}

/* Each of the 7 shutdown inputs drops the TS once it's been open 3 ms, in
 * precharge and while driving. Then a bad IMD faults and the first fault sticks */
static void test_shutdown_and_imd(void) {
    for (unsigned bit = 0; bit < 7; ++bit) {
        precharge();
        in.shutdown_closed &= ~(1u << bit);
        step(AIR_SHUTDOWN_DEBOUNCE_MS - 1); /* Shorter open readings are glitches. */
        CHECK(a.state == AIR_STATE_PRECHARGE && a.precharge_command);
        step(1);
        CHECK(a.state == AIR_STATE_DISCHARGE && !a.precharge_command);
        active();
        in.shutdown_closed &= ~(1u << bit);
        step(AIR_SHUTDOWN_DEBOUNCE_MS - 1);
        CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command);
        step(1);
        CHECK(a.state == AIR_STATE_DISCHARGE && !a.main_command);
    }
    precharge();
    in.imd_ok = false;
    step(1);
    expect_fault(AIR_FAULT_IMD_STATUS);
    in.imd_ok = true;
    step(1);
    expect_fault(AIR_FAULT_IMD_STATUS); /* First fault latches. */
    air_trip(&a, AIR_FAULT_CAN_ERROR);
    expect_fault(AIR_FAULT_IMD_STATUS);
}

/* Precharge ends at exactly 95%; AIR feedback timeouts; welds and a slow
 * discharge after the loop opens */
static void test_voltage_and_feedback(void) {
    precharge();
    d.tractive_mv = 379999; /* just under 95% */
    step(1);
    CHECK(!a.main_command);
    d.tractive_mv = 380000; /* exactly 95% */
    step(1);
    CHECK(a.main_command && a.precharge_command);
    step(AIR_CONTACTOR_CLOSE_MS); /* our AIR never confirms closed */
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
    precharge();
    step(AIR_PRECHARGE_TIMEOUT_MS); /* precharge never gets to 95% */
    expect_fault(AIR_FAULT_PRECHARGE_FAIL);
    active();
    in.air_n_closed = false; /* AIR- drops out while driving */
    step(1);
    expect_fault(AIR_FAULT_CONTACTOR_FEEDBACK);
    active();
    in.shutdown_closed = 0;
    step(AIR_SHUTDOWN_DEBOUNCE_MS);
    step(AIR_CONTACTOR_OPEN_MS); /* both AIRs still read closed */
    expect_fault(AIR_FAULT_BOTH_AIRS_WELD);
    active();
    in.shutdown_closed = 0;
    step(AIR_SHUTDOWN_DEBOUNCE_MS);
    in.air_p_closed = in.air_n_closed = false;
    step(AIR_DISCHARGE_TIMEOUT_MS); /* DC link still at 380 V after 10 s */
    expect_fault(AIR_FAULT_DISCHARGE_FAIL);
}

/* Old or bad BMS and IVT data faults */
static void test_can_faults(void) {
    active();
    now += AIR_IVT_TIMEOUT_MS; /* IVT goes quiet */
    d.bms_ms = now;
    air_step(&a, &in, &d, now);
    expect_fault(AIR_FAULT_CAN_GMETER_TIMEOUT);
    precharge();
    now += AIR_BMS_TIMEOUT_MS; /* BMS goes quiet */
    d.ivt_ms = now;
    air_step(&a, &in, &d, now);
    expect_fault(AIR_FAULT_CAN_BMS_TIMEOUT);
    active();
    d.bms_fault = 0x8000;
    step(1);
    expect_fault(AIR_FAULT_BMS_VOLTAGE);
    active();
    d.ivt_error = true;
    step(1);
    expect_fault(AIR_FAULT_IVT_STATUS);
    precharge();
    d.tractive_mv = -1; /* a negative voltage reading */
    step(1);
    expect_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
}

/* TSMS closed at power-up faults, no CAN at startup faults, and the timers
 * still work when the ms counter wraps around */
static void test_startup_and_rollover(void) {
    air_init(&a, 0);
    in = (air_inputs_t){.shutdown_closed = AIR_SS_TSMS};
    d = (air_measurements_t){0};
    air_step(&a, &in, &d, 0);
    expect_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
    air_init(&a, 0);
    in = (air_inputs_t){.imd_ok = true};
    air_step(&a, &in, &d, AIR_IMD_SETTLE_MS + AIR_STARTUP_CAN_WAIT_MS);
    expect_fault(AIR_FAULT_CAN_BMS_TIMEOUT);
    reset_at(UINT32_MAX - 2000u); /* startup timing crosses uint32 wrap */
    CHECK(a.fault == AIR_FAULT_NONE);
    precharge();
    now = UINT32_MAX - 10u;
    a.entered_ms = now; /* precharge starts 10 ms before the wrap */
    step(20);
    CHECK(a.state == AIR_STATE_PRECHARGE);
    step(AIR_PRECHARGE_TIMEOUT_MS - 20);
    expect_fault(AIR_FAULT_PRECHARGE_FAIL);
}

/* Each weld after the loop opens, bad BMS values, a charged DC link in IDLE,
 * holding TSMS during discharge, and a corrupted state value */
static void test_additional_interlocks(void) {
    for (unsigned feedback = 1; feedback < 4; ++feedback) {
        active();
        in.shutdown_closed = 0;
        step(AIR_SHUTDOWN_DEBOUNCE_MS);
        in.air_p_closed = (feedback & 1u) != 0;
        in.air_n_closed = (feedback & 2u) != 0;
        step(AIR_CONTACTOR_OPEN_MS);
        expect_fault(feedback == 3 ? AIR_FAULT_BOTH_AIRS_WELD :
                     feedback == 1 ? AIR_FAULT_AIR_P_WELD : AIR_FAULT_AIR_N_WELD);
    }
    reset_at(0);
    d.pack_mv = AIR_PACK_MIN_MV - 1; /* pack voltage too low */
    step(1);
    expect_fault(AIR_FAULT_BMS_VOLTAGE);
    reset_at(0);
    d.bms_state = 2; /* BMS in its fault state */
    step(1);
    expect_fault(AIR_FAULT_BMS_VOLTAGE);
    reset_at(0);
    d.tractive_mv = AIR_TRACTIVE_CHARGED_MV;
    step(1);
    expect_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
    active();
    in.shutdown_closed &= ~AIR_SS_BMS;
    step(AIR_SHUTDOWN_DEBOUNCE_MS);
    in.air_p_closed = in.air_n_closed = false;
    d.tractive_mv = 0;
    step(AIR_CONTACTOR_OPEN_MS);
    CHECK(a.state == AIR_STATE_DISCHARGE); /* Held TSMS does not re-arm. */
    in.shutdown_closed = 0;
    step(1);
    CHECK(a.state == AIR_STATE_IDLE);
    precharge();
    a.state = (air_state_t)255;
    step(1);
    expect_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
}

/* CAN messages decode and encode byte for byte like the old firmware */
static void test_wire_protocol(void) {
    /* Independent legacy wire vectors: 15625 * .0256 = 400 V. */
    const uint8_t bms[7] = {0, 0, 0x24, 0xf4, 0, 0, 0};
    const uint8_t ivt[6] = {1, 0, 0, 5, 0xcc, 0x60}; /* 380000 mV */
    air_measurements_t data = {0};
    CHECK(air_decode(&data, AIR_CAN_BMS_ID, bms, sizeof bms, 123));
    CHECK(data.pack_mv == 400000 && data.bms_fault == 0 && data.bms_ms == 123);
    CHECK(air_decode(&data, AIR_CAN_IVT_ID, ivt, sizeof ivt, 124));
    CHECK(data.tractive_mv == 380000 && data.ivt_ms == 124);
    CHECK(!air_decode(&data, AIR_CAN_BMS_ID, bms, 6, 999)); /* wrong length: ignored */
    CHECK(data.bms_ms == 123);
    uint8_t negative[6] = {1, 0x80, 0xff, 0xff, 0xff, 0xff}; /* -1 mV with an error flag */
    CHECK(air_decode(&data, AIR_CAN_IVT_ID, negative, 6, 125));
    CHECK(data.tractive_mv == -1 && data.ivt_error);
    uint8_t fault[7] = {0xfe, 0xff, 3, 0, 0, 0, 0}; /* every BMS fault bit set */
    CHECK(air_decode(&data, AIR_CAN_BMS_ID, fault, 7, 126));
    CHECK(data.bms_fault == 65535 && data.bms_state == 2);
    a = (air_t){.state = AIR_STATE_TS_ACTIVE, .fault = AIR_FAULT_NONE};
    in = (air_inputs_t){.shutdown_closed = AIR_SS_ALL, .imd_ok = true,
                        .air_n_closed = true, .air_p_closed = true};
    uint8_t encoded[4];
    air_encode_status(encoded, &a, &in);
    const uint8_t expected[4] = {0, 4, 255, 3};
    CHECK(memcmp(encoded, expected, 4) == 0);
}

/* Runs every group; the first failed CHECK prints its line and stops */
int main(void) {
    test_normal_and_repeated_cycles();
    test_shutdown_and_imd();
    test_voltage_and_feedback();
    test_can_faults();
    test_startup_and_rollover();
    test_additional_interlocks();
    test_wire_protocol();
    puts("AIR: all seven test groups passed");
    return 0;
}
