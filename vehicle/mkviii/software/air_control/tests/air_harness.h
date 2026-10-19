/* Shared helpers for the host logic tests; never part of air.elf.
 * Each scenario helper starts from power-up, so tests stay independent. */
#ifndef AIR_HARNESS_H
#define AIR_HARNESS_H

#include "vehicle/mkviii/software/air_control/air.h"
#include "vehicle/mkviii/software/air_control/air_config.h"
#include "vehicle/mkviii/software/air_control/air_protocol.h"
#include <stdio.h>
#include <stdlib.h>

/* Unlike assert(), CHECK still runs when Bazel uses -DNDEBUG. */
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

#define PACK_MV 400000u
#define PRECHARGED_MV 380000 /* exactly 95% of PACK_MV */

/* Weld faults for the AIR this board drives and the AIR the loop closes. */
#define WELD_DRIVEN (AIR_CONTROLLED_NEGATIVE ? AIR_FAULT_AIR_N_WELD : AIR_FAULT_AIR_P_WELD)
#define WELD_LOOP (AIR_CONTROLLED_NEGATIVE ? AIR_FAULT_AIR_P_WELD : AIR_FAULT_AIR_N_WELD)

static air_t a;
static air_inputs_t in;
static air_measurements_t d;
static uint32_t now;

/* Feedback of the AIR this board drives (AIR- by default). */
static inline void set_driven(bool closed) {
    if (AIR_CONTROLLED_NEGATIVE) in.air_n_closed = closed;
    else in.air_p_closed = closed;
}

/* Feedback of the AIR the shutdown loop closes directly (AIR+ by default). */
static inline void set_loop_air(bool closed) {
    if (AIR_CONTROLLED_NEGATIVE) in.air_p_closed = closed;
    else in.air_n_closed = closed;
}

/* Advance time with fresh CAN data, then run one step. */
static inline void step(uint32_t ms) {
    now += ms;
    d.bms_ms = now;
    d.ivt_ms = now;
    air_step(&a, &in, &d, now);
}

/* Run every millisecond in between, not one big jump. */
static inline void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) step(1);
}

/* Power up with healthy inputs and data. No step has run yet. */
static inline void power_up(void) {
    now = 0;
    air_init(&a, now);
    in = (air_inputs_t){.imd_ok = true};
    d = (air_measurements_t){.bms_seen = true, .ivt_seen = true, .pack_mv = PACK_MV};
}

static inline void to_idle(void) {
    power_up();
    step(AIR_IMD_SETTLE_MS);
    CHECK(a.state == AIR_STATE_IDLE);
}

static inline void to_precharge(void) {
    to_idle();
    in.shutdown_closed = AIR_SS_ALL;
    step(1);
    CHECK(a.state == AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED);
    set_loop_air(true);
    step(1);
    CHECK(a.state == AIR_STATE_PRECHARGE && a.precharge_command);
}

static inline void to_active(void) {
    to_precharge();
    d.tractive_mv = PRECHARGED_MV;
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command);
    set_driven(true);
    step(1);
    CHECK(a.state == AIR_STATE_TS_ACTIVE && a.main_command && !a.precharge_command);
}

static inline void expect_fault(air_fault_t f) {
    CHECK(a.state == AIR_STATE_FAULT && a.fault == f);
    CHECK(!a.main_command && !a.precharge_command);
}

#endif
