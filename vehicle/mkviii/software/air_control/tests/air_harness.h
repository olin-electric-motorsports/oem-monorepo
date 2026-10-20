/* helpers the tests share (never built into the firmware). Each "to_" helper
 * starts again from power-up, so one test can't affect the next */
#ifndef AIR_HARNESS_H
#define AIR_HARNESS_H

#include "vehicle/mkviii/software/air_control/air.h"
#include "vehicle/mkviii/software/air_control/air_config.h"
#include "vehicle/mkviii/software/air_control/air_protocol.h"
#include <stdio.h>
#include <stdlib.h>

/* Like assert(), but still works when Bazel builds with -DNDEBUG: prints the
 * failing line and stops the test */
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

#define PACK_MV 400000u      /* a 400 V pack */
#define PRECHARGED_MV 380000 /* exactly 95% of PACK_MV, where precharge ends */

/* Weld fault codes for our AIR and the loop's AIR, so the same tests work
 * whichever AIR this board drives */
#define WELD_DRIVEN (AIR_CONTROLLED_NEGATIVE ? AIR_FAULT_AIR_N_WELD : AIR_FAULT_AIR_P_WELD)
#define WELD_LOOP (AIR_CONTROLLED_NEGATIVE ? AIR_FAULT_AIR_P_WELD : AIR_FAULT_AIR_N_WELD)

/* The state machine, its inputs, its CAN data and the clock */
static air_t a;
static air_inputs_t in;
static air_measurements_t d;
static uint32_t now;

/* Our AIR: the one this board switches on (AIR- by default) */
static inline void set_driven(bool closed) {
    if (AIR_CONTROLLED_NEGATIVE) in.air_n_closed = closed;
    else in.air_p_closed = closed;
}

/* The loop's AIR: the one the shutdown loop powers directly (AIR+ by default) */
static inline void set_loop_air(bool closed) {
    if (AIR_CONTROLLED_NEGATIVE) in.air_p_closed = closed;
    else in.air_n_closed = closed;
}

/* Move the clock forward, mark the CAN data as just arrived, run one step */
static inline void step(uint32_t ms) {
    now += ms;
    d.bms_ms = now;
    d.ivt_ms = now;
    air_step(&a, &in, &d, now);
}

/* Same, but 1 ms at a time like the real main loop, not one big jump */
static inline void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) step(1);
}

/* Fresh start with healthy inputs and data; nothing has run yet */
static inline void power_up(void) {
    now = 0;
    air_init(&a, now);
    in = (air_inputs_t){.imd_ok = true};
    d = (air_measurements_t){.bms_seen = true, .ivt_seen = true, .pack_mv = PACK_MV};
}

/* Power up and wait out startup */
static inline void to_idle(void) {
    power_up();
    step(AIR_IMD_SETTLE_MS);
    CHECK(a.state == AIR_STATE_IDLE);
}

/* Close the loop, then the loop's AIR closes and precharge starts */
static inline void to_precharge(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED);
    set_loop_air(true);
    step(1);
    CHECK(a.state == AIR_STATE_PRECHARGE && a.precharge_command);
}

/* Precharge reaches 95%, our AIR closes and precharge turns off */
static inline void to_active(void) {
    to_precharge();
    d.tractive_mv = PRECHARGED_MV;
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command);
    set_driven(true);
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command && !a.precharge_command);
}

/* Open every node, 1 ms at a time, for 3 ms so the TS drops to DISCHARGE */
static inline void open_loop(void) {
    in.shutdown_closed = 0;
    run_ms(AIR_SHUTDOWN_DEBOUNCE_MS);
    CHECK(a.state == AIR_STATE_DISCHARGE && !a.main_command && !a.precharge_command);
}

/* Check we're in FAULT with this code and both outputs off */
static inline void expect_fault(air_fault_t f) {
    CHECK(a.state == AIR_STATE_FAULT && a.fault == f);
    CHECK(!a.main_command && !a.precharge_command);
}

#endif
