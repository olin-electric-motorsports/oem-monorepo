/* Randomized safety test for air_step().
 *
 * A small model of the car drives the state machine the way the firmware's
 * main loop does: sample the inputs, decode CAN frames, step. The model has
 * the shutdown loop, both AIRs, the precharge relay, the DC link, and BMS and
 * IVT frames. Each power-up, a simulated driver opens and closes the loop,
 * one scenario from scenario_t may inject a fault, and timing jumps, lost
 * frames and rare input glitches happen throughout. After every step the
 * rules in check_rules() must hold.
 *
 * Usage: air_invariant_test [seed] [steps]
 */
#include "vehicle/mkviii/software/air_control/air.h"
#include "vehicle/mkviii/software/air_control/air_config.h"
#include "vehicle/mkviii/software/air_control/air_protocol.h"
#include <stdio.h>
#include <stdlib.h>

#define DEFAULT_SEED 0x41495231ull /* "AIR1" */
#define DEFAULT_STEPS 10000000ul   /* about 3 hours of simulated time */

/* Random numbers: xorshift64*, the same sequence on every platform. */
static uint64_t rng;

static uint32_t rnd(void) {
    rng ^= rng >> 12;
    rng ^= rng << 25;
    rng ^= rng >> 27;
    return (uint32_t)((rng * 2685821657736338717ull) >> 32);
}

static uint32_t between(uint32_t lo, uint32_t hi) { return lo + rnd() % (hi - lo + 1u); }
static bool one_in(uint32_t n) { return rnd() % n == 0u; }

/* Uniform noise in [-amplitude, +amplitude]. */
static double noise(double amplitude) {
    return amplitude * ((double)between(0, 2000) - 1000.0) / 1000.0;
}

/* True once time t has reached the deadline, across 32-bit wraparound. */
static bool reached(uint32_t t, uint32_t deadline) {
    return (uint32_t)(t - deadline) < 0x80000000u;
}

/* A contactor whose contacts follow its coil after a delay. */
typedef struct {
    bool coil, contact;
    bool welded, stuck_open, reads_open, reads_closed;
    uint32_t coil_ms, close_ms, open_ms;
} contactor_t;

static contactor_t new_contactor(uint32_t close_lo, uint32_t close_hi) {
    contactor_t c = {0};
    c.close_ms = between(close_lo, close_hi);
    c.open_ms = between(2, 30);
    return c;
}

static void contactor_update(contactor_t *c, bool coil, uint32_t t) {
    if (coil != c->coil) {
        c->coil = coil;
        c->coil_ms = t;
    }
    bool target = c->coil && !c->stuck_open;
    if (c->welded && c->contact) target = true;
    const uint32_t delay = target ? c->close_ms : c->open_ms;
    if (target != c->contact && (uint32_t)(t - c->coil_ms) >= delay) c->contact = target;
}

static bool reads_closed(const contactor_t *c) {
    if (c->reads_closed) return true;
    if (c->reads_open) return false;
    return c->contact;
}

/* At most one scenario per power-up. */
typedef enum {
    S_NONE,
    S_WELD_LOOP_AIR,          /* the AIR the loop closes welds shut */
    S_WELD_DRIVEN_AIR,        /* the AIR this board drives welds shut */
    S_LOOP_AIR_STUCK_OPEN,
    S_DRIVEN_AIR_STUCK_OPEN,
    S_SLOW_CONTACTOR,         /* closes after AIR_CONTACTOR_CLOSE_MS */
    S_DRIVEN_FEEDBACK_OPEN,   /* broken feedback wire */
    S_FEEDBACK_CLOSED_AT_BOOT,
    S_NO_PRECHARGE,           /* open precharge resistor */
    S_SLOW_PRECHARGE,
    S_SLOW_DISCHARGE,
    S_CHARGED_AT_BOOT,        /* DC link charged, discharge circuit dead */
    S_TSMS_CLOSED_AT_BOOT,
    S_QUICK_RECLOSE,          /* driver closes the loop again while discharging */
    S_NODE_SENSE_FAULT,       /* one node reads wrong for a few ms */
    S_NOISY_INPUTS,           /* frequent 1 ms glitches */
    S_IMD_FAULT,
    S_BMS_SILENT,
    S_IVT_SILENT,
    S_BMS_FAULT_BIT,
    S_BMS_FAULT_STATE,
    S_BMS_CHARGING,
    S_PACK_LOW,
    S_IVT_ERROR,
    S_IVT_NEGATIVE,
    S_WATCHDOG_RESETS,
    SCENARIO_COUNT
} scenario_t;

typedef struct {
    scenario_t scenario;
    uint32_t end_ms;            /* power-up ends at the first IDLE after this */
    uint32_t event_ms;          /* when the scenario's fault starts */
    bool event_done;
    bool loop;                  /* the real shutdown loop is closed */
    uint32_t toggle_ms;         /* when the driver next opens or closes it */
    uint8_t open_nodes;         /* nodes that still read closed while it is open */
    bool sensing_wrong;
    uint8_t wrong_node;
    uint32_t sensing_wrong_until;
    bool imd_ok;
    uint32_t imd_bad_until;
    bool bms_quiet, ivt_quiet;
    uint32_t bms_quiet_until, ivt_quiet_until;
    uint16_t bms_fault_bits;
    bool bms_fault_state, ivt_error, ivt_negative;
    contactor_t loop_air, driven_air, precharge;
    double pack_v, link_v, tau_precharge_ms, tau_discharge_ms;
    uint32_t bms_next, ivt_next, reset_ms;
    uint8_t ivt_counter;
    bool faulted;
    uint32_t restart_ms;        /* the car is power-cycled after a fault */
} plant_t;

static plant_t p;
static air_t a;
static air_inputs_t in;
static air_measurements_t data;
static uint32_t now, boot_ms;
static uint64_t seed;
static unsigned long step_no, power_ups, active_entries, misreported_welds, threshold_trips;
static unsigned long state_entries[AIR_STATE_FAULT + 1];
static unsigned long faults[AIR_FAULT_IVT_STATUS + 1];

static const char *const fault_names[] = {
    "NONE", "AIR_N_WELD", "AIR_P_WELD", "BOTH_AIRS_WELD", "PRECHARGE_FAIL",
    "DISCHARGE_FAIL", "PRECHARGE_FAIL_RELAY_WELDED", "CAN_ERROR", "CAN_BMS_TIMEOUT",
    "CAN_GMETER_TIMEOUT", "SHUTDOWN_IMPLAUSIBILITY", "TRACTIVE_VOLTAGE", "BMS_VOLTAGE",
    "IMD_STATUS", "BOARD_CONFIG", "CONTACTOR_FEEDBACK", "IVT_STATUS",
};
_Static_assert(sizeof fault_names / sizeof fault_names[0] == AIR_FAULT_IVT_STATUS + 1,
               "one name per fault code");

static const char *fault_name(air_fault_t f) {
    return (unsigned)f <= AIR_FAULT_IVT_STATUS ? fault_names[f] : "invalid";
}

static void fail(const char *rule) {
    fprintf(stderr, "RULE BROKEN: %s\n", rule);
    fprintf(stderr, "  seed %llu, step %lu, time %lu ms, scenario %d\n",
            (unsigned long long)seed, step_no, (unsigned long)now, (int)p.scenario);
    fprintf(stderr, "  state %d, fault %s, AIR- command %d, precharge command %d\n",
            (int)a.state, fault_name(a.fault), a.main_command, a.precharge_command);
    fprintf(stderr, "  nodes 0x%02x, IMD OK %d, AIR+ closed %d, AIR- closed %d\n",
            (unsigned)in.shutdown_closed, in.imd_ok, in.air_p_closed, in.air_n_closed);
    fprintf(stderr, "  pack %lu mV, DC link %ld mV, BMS age %lu ms, IVT age %lu ms\n",
            (unsigned long)data.pack_mv, (long)data.tractive_mv,
            (unsigned long)(now - data.bms_ms), (unsigned long)(now - data.ivt_ms));
    fprintf(stderr, "  rerun: air_invariant_test %llu %lu\n", (unsigned long long)seed, step_no + 1);
    exit(1);
}

/* Power-up or reset: the controller starts over and RAM is cleared. */
static void boot_controller(void) {
    air_init(&a, now);
    data = (air_measurements_t){0};
    boot_ms = now;
}

static void start_power_up(void) {
    ++power_ups;
    p = (plant_t){0};
    p.scenario = between(0, 99) < 30 ? S_NONE : (scenario_t)between(1, SCENARIO_COUNT - 1);
    p.end_ms = now + between(8000, 30000);
    p.event_ms = now + between(0, 20000);
    p.toggle_ms = now + between(AIR_IMD_SETTLE_MS + 300, AIR_IMD_SETTLE_MS + 3000);
    p.imd_ok = true;
    p.pack_v = (double)between(280, 440);
    p.tau_precharge_ms = (double)between(100, 1200);
    p.tau_discharge_ms = (double)between(200, 1000);
    p.loop_air = new_contactor(5, 60);
    p.driven_air = new_contactor(5, 60);
    p.precharge = new_contactor(2, 15);
    p.bms_next = now + between(1, 10);
    p.ivt_next = now + between(1, 10);

    switch (p.scenario) {
    case S_LOOP_AIR_STUCK_OPEN: p.loop_air.stuck_open = true; break;
    case S_DRIVEN_AIR_STUCK_OPEN: p.driven_air.stuck_open = true; break;
    case S_SLOW_CONTACTOR:
        (one_in(2) ? &p.loop_air : &p.driven_air)->close_ms =
            between(AIR_CONTACTOR_CLOSE_MS + 10, 400);
        break;
    case S_DRIVEN_FEEDBACK_OPEN: p.driven_air.reads_open = true; break;
    case S_FEEDBACK_CLOSED_AT_BOOT: {
        const uint32_t which = between(0, 2); /* loop AIR, driven AIR or both */
        p.loop_air.reads_closed = which != 1;
        p.driven_air.reads_closed = which != 0;
        break;
    }
    case S_SLOW_PRECHARGE: p.tau_precharge_ms = (double)between(2000, 4000); break;
    case S_SLOW_DISCHARGE: p.tau_discharge_ms = (double)between(3000, 8000); break;
    case S_CHARGED_AT_BOOT:
        p.link_v = (double)between(20, 400);
        p.tau_discharge_ms = 600000.0;
        break;
    case S_TSMS_CLOSED_AT_BOOT:
        p.loop = true;
        p.toggle_ms = now + between(500, 6000);
        break;
    case S_PACK_LOW: p.pack_v = (double)between(120, 190); break;
    case S_WATCHDOG_RESETS: p.reset_ms = p.event_ms; break;
    default: break;
    }
    boot_controller();
}

static void scenario_event(void) {
    if (p.event_done || !reached(now, p.event_ms)) return;
    p.event_done = true;
    switch (p.scenario) {
    case S_WELD_LOOP_AIR: p.loop_air.welded = true; break;
    case S_WELD_DRIVEN_AIR: p.driven_air.welded = true; break;
    case S_NODE_SENSE_FAULT:
        p.sensing_wrong = true;
        p.wrong_node = (uint8_t)between(1, 6);
        p.sensing_wrong_until = now + between(1, 50);
        break;
    case S_IMD_FAULT:
        p.imd_ok = false;
        p.imd_bad_until = now + between(1, 300);
        break;
    case S_BMS_SILENT:
        p.bms_quiet = true;
        p.bms_quiet_until = now + between(AIR_BMS_TIMEOUT_MS + 100, 3000);
        break;
    case S_IVT_SILENT:
        p.ivt_quiet = true;
        p.ivt_quiet_until = now + between(AIR_IVT_TIMEOUT_MS + 100, 2000);
        break;
    case S_BMS_FAULT_BIT: p.bms_fault_bits = (uint16_t)(1u << between(0, 15)); break;
    case S_BMS_FAULT_STATE: p.bms_fault_state = true; break;
    case S_IVT_ERROR: p.ivt_error = true; break;
    case S_IVT_NEGATIVE: p.ivt_negative = true; break;
    default: break;
    }
}

/* The driver closes the loop after power-up, then opens and closes it at random. */
static void driver(void) {
    if (!reached(now, p.toggle_ms)) return;
    p.loop = !p.loop;
    if (p.loop) {
        p.toggle_ms = now + between(300, 9000);
        return;
    }
    /* Nodes before the break still read closed; TSMS, the last node, reads open. */
    p.open_nodes = (uint8_t)(rnd() & (AIR_SS_ALL & ~AIR_SS_TSMS));
    if (p.scenario == S_QUICK_RECLOSE) p.toggle_ms = now + between(120, 400);
    else if (p.scenario == S_SLOW_DISCHARGE) p.toggle_ms = now + between(12000, 20000);
    else p.toggle_ms = now + between(5000, 12000);
}

static void plant_update(uint32_t dt) {
    contactor_update(&p.loop_air, p.loop, now);
    /* This board's driver and the precharge relay coil both get loop power. */
    contactor_update(&p.driven_air, p.loop && a.main_command, now);
    contactor_update(&p.precharge, p.loop && a.precharge_command, now);

    const bool through_resistor = p.precharge.contact && p.scenario != S_NO_PRECHARGE;
    double target = 0.0, tau_ms = 60000.0; /* an isolated DC link leaks slowly */
    if (p.loop_air.contact && (p.driven_air.contact || through_resistor)) {
        target = p.pack_v;
        tau_ms = p.driven_air.contact ? 2.0 : p.tau_precharge_ms;
    } else if (!p.loop) {
        tau_ms = p.tau_discharge_ms; /* the discharge circuit is on while the loop is open */
    }
    double k = (double)dt / tau_ms;
    if (k > 1.0) k = 1.0;
    p.link_v += (target - p.link_v) * k;
}

static void sense(void) {
    uint8_t nodes = p.loop ? (uint8_t)AIR_SS_ALL : p.open_nodes;
    if (p.sensing_wrong) {
        if (reached(now, p.sensing_wrong_until)) p.sensing_wrong = false;
        else if (p.loop) nodes = (uint8_t)(nodes & ~(1u << p.wrong_node));
        else nodes = (uint8_t)(nodes | AIR_SS_TSMS);
    }
    if (one_in(p.scenario == S_NOISY_INPUTS ? 300u : 50000u))
        nodes = (uint8_t)(nodes ^ (1u << between(0, 6)));
    if (!p.imd_ok && reached(now, p.imd_bad_until)) p.imd_ok = true;
    const bool loop_fb = reads_closed(&p.loop_air), driven_fb = reads_closed(&p.driven_air);
    in.shutdown_closed = nodes;
    in.imd_ok = p.imd_ok;
    in.air_p_closed = AIR_CONTROLLED_NEGATIVE ? loop_fb : driven_fb;
    in.air_n_closed = AIR_CONTROLLED_NEGATIVE ? driven_fb : loop_fb;
}

static bool frame_lost(bool *quiet, uint32_t until) {
    if (*quiet && reached(now, until)) *quiet = false;
    return *quiet || one_in(200);
}

static void send_bms(void) {
    if (!reached(now, p.bms_next)) return;
    p.bms_next = now + 10u; /* 100 Hz */
    if (frame_lost(&p.bms_quiet, p.bms_quiet_until)) return;
    const uint64_t state = p.bms_fault_state ? 2u : (p.scenario == S_BMS_CHARGING ? 1u : 0u);
    const uint64_t raw = (uint64_t)((p.pack_v + noise(0.5)) / 0.0256 + 0.5);
    const uint64_t bits = state | ((uint64_t)p.bms_fault_bits << 2) | (raw << 18);
    uint8_t frame[AIR_CAN_BMS_BYTES];
    for (unsigned i = 0; i < AIR_CAN_BMS_BYTES; ++i) frame[i] = (uint8_t)(bits >> (8u * i));
    if (!air_decode(&data, AIR_CAN_BMS_ID, frame, sizeof frame, now)) fail("BMS frames decode");
}

static void send_ivt(void) {
    if (!reached(now, p.ivt_next)) return;
    p.ivt_next = now + 10u;
    if (frame_lost(&p.ivt_quiet, p.ivt_quiet_until)) return;
    /* Clamped at 0 V. A real IVT may read slightly negative at rest, and air.c
     * faults on any negative reading; S_IVT_NEGATIVE covers that path. */
    int32_t mv = (int32_t)((p.link_v + noise(0.3)) * 1000.0);
    if (mv < 0) mv = 0;
    if (p.ivt_negative) {
        mv = -(int32_t)between(1, 2000);
        p.ivt_negative = false;
    }
    const uint32_t raw = (uint32_t)mv;
    const uint8_t flags = (uint8_t)((p.ivt_error ? 0x10u : 0u) | (p.ivt_counter++ & 0x0fu));
    const uint8_t frame[AIR_CAN_IVT_BYTES] = {
        1u, flags, (uint8_t)(raw >> 24), (uint8_t)(raw >> 16), (uint8_t)(raw >> 8), (uint8_t)raw,
    };
    if (!air_decode(&data, AIR_CAN_IVT_ID, frame, sizeof frame, now)) fail("IVT frames decode");
}

static bool data_healthy(void) {
    return data.bms_seen && (uint32_t)(now - data.bms_ms) < AIR_BMS_TIMEOUT_MS &&
           data.ivt_seen && (uint32_t)(now - data.ivt_ms) < AIR_IVT_TIMEOUT_MS &&
           data.bms_fault == 0 && data.bms_state < 2 && data.pack_mv >= AIR_PACK_MIN_MV &&
           !data.ivt_error && data.tractive_mv >= 0;
}

/* The safety rules. They must hold after every step, whatever the inputs. */
static void check_rules(const air_t *before) {
    const bool driven_closed = AIR_CONTROLLED_NEGATIVE ? in.air_n_closed : in.air_p_closed;
    const bool loop_air_closed = AIR_CONTROLLED_NEGATIVE ? in.air_p_closed : in.air_n_closed;
    const bool all_closed = in.shutdown_closed == AIR_SS_ALL;
    const bool ready = all_closed && in.imd_ok && data_healthy() && loop_air_closed;
    const bool commanding = a.main_command || a.precharge_command;

    if ((unsigned)a.state > AIR_STATE_FAULT) fail("the state is always a valid value");
    if (a.main_command && a.state != AIR_STATE_TS_ACTIVE)
        fail("AIR- is only commanded in TS_ACTIVE");
    if (a.main_command && !ready)
        fail("AIR- is only commanded with all 7 nodes closed, the IMD OK, fresh healthy "
             "CAN data and the loop's AIR closed");
    if (a.main_command && !before->main_command &&
        (driven_closed ||
         (uint64_t)data.tractive_mv * 100u < (uint64_t)data.pack_mv * AIR_PRECHARGE_PERCENT))
        fail("AIR- is first commanded only at 95% of pack voltage, while it still reads open");
    if (a.precharge_command &&
        !(ready && (a.state == AIR_STATE_PRECHARGE || a.state == AIR_STATE_TS_ACTIVE)))
        fail("precharge is only commanded while closing, with all 7 nodes closed, the IMD "
             "OK, fresh healthy CAN data and the loop's AIR closed");
    if (!all_closed && commanding) fail("any open node drops both commands in the same step");
    if (a.fault != AIR_FAULT_NONE && (a.state != AIR_STATE_FAULT || commanding))
        fail("a fault means the FAULT state with both commands off");
    if (before->fault != AIR_FAULT_NONE && a.fault != before->fault)
        fail("a latched fault never changes or clears");
    if ((uint32_t)(now - boot_ms) < AIR_IMD_SETTLE_MS && commanding)
        fail("nothing is commanded in the first 4 s after power-up");
    if (before->state == AIR_STATE_INIT && (in.shutdown_closed & AIR_SS_TSMS) &&
        a.fault == AIR_FAULT_NONE)
        fail("TSMS closed during startup always faults");
}

static void record(const air_t *before) {
    if (a.state != before->state) {
        ++state_entries[a.state];
        if (a.state == AIR_STATE_TS_ACTIVE) ++active_entries;
    }
    if (a.fault == AIR_FAULT_NONE || before->fault != AIR_FAULT_NONE) return;
    ++faults[a.fault];
    const bool weld = a.fault == AIR_FAULT_AIR_N_WELD || a.fault == AIR_FAULT_AIR_P_WELD ||
                      a.fault == AIR_FAULT_BOTH_AIRS_WELD;
    const bool stuck = p.loop_air.welded || p.driven_air.welded ||
                       p.loop_air.reads_closed || p.driven_air.reads_closed;
    if (weld && !stuck) ++misreported_welds;
    if (a.fault == AIR_FAULT_TRACTIVE_VOLTAGE && before->state == AIR_STATE_IDLE && p.link_v < 5.5)
        ++threshold_trips;
    p.faulted = true;
    p.restart_ms = now + between(50, 1500);
}

int main(int argc, char **argv) {
    seed = argc > 1 ? strtoull(argv[1], NULL, 0) : DEFAULT_SEED;
    const unsigned long steps = argc > 2 ? strtoul(argv[2], NULL, 0) : DEFAULT_STEPS;
    rng = seed ? seed : 1u;
    now = 0u - between(1, 3600000); /* the millisecond counter wraps in the first hour */
    start_power_up();

    for (step_no = 0; step_no < steps; ++step_no) {
        const uint32_t dt = one_in(50) ? between(2, 40) : 1u; /* a late main loop */
        now += dt;
        driver();
        scenario_event();
        if (p.scenario == S_WATCHDOG_RESETS && !p.faulted && reached(now, p.reset_ms)) {
            boot_controller();
            p.reset_ms = now + between(2000, 10000);
        }
        plant_update(dt);
        sense();
        send_bms();
        send_ivt();
        const air_t before = a;
        air_step(&a, &in, &data, now);
        check_rules(&before);
        record(&before);
        if (p.faulted ? reached(now, p.restart_ms)
                      : (reached(now, p.end_ms) && a.state == AIR_STATE_IDLE))
            start_power_up();
    }

    printf("AIR invariants: %lu steps, seed %llu, %lu power-ups, TS_ACTIVE entered %lu times\n",
           steps, (unsigned long long)seed, power_ups, active_entries);
    printf("faults latched:");
    for (unsigned f = 1; f <= AIR_FAULT_IVT_STATUS; ++f)
        if (faults[f]) printf(" %s %lu", fault_names[f], faults[f]);
    printf("\nknown issues seen: %lu weld faults with no stuck contactor, "
           "%lu TRACTIVE_VOLTAGE faults from noise at 5 V after discharge\n",
           misreported_welds, threshold_trips);

    if (steps < DEFAULT_STEPS) {
        puts("all safety rules held (short run: exploration not checked)");
        return 0;
    }
    /* Make sure the model really reached every path air.c can fault on. */
    static const air_fault_t reachable[] = {
        AIR_FAULT_AIR_N_WELD, AIR_FAULT_AIR_P_WELD, AIR_FAULT_BOTH_AIRS_WELD,
        AIR_FAULT_PRECHARGE_FAIL, AIR_FAULT_DISCHARGE_FAIL, AIR_FAULT_CAN_BMS_TIMEOUT,
        AIR_FAULT_CAN_GMETER_TIMEOUT, AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY,
        AIR_FAULT_TRACTIVE_VOLTAGE, AIR_FAULT_BMS_VOLTAGE, AIR_FAULT_IMD_STATUS,
        AIR_FAULT_CONTACTOR_FEEDBACK, AIR_FAULT_IVT_STATUS,
    };
    bool missed = active_entries < 100;
    for (size_t i = 0; i < sizeof reachable / sizeof reachable[0]; ++i) {
        if (!faults[reachable[i]]) {
            printf("the model never produced %s\n", fault_names[reachable[i]]);
            missed = true;
        }
    }
    for (unsigned s = AIR_STATE_IDLE; s <= AIR_STATE_FAULT; ++s) {
        if (!state_entries[s]) {
            printf("the model never reached state %u\n", s);
            missed = true;
        }
    }
    if (missed) {
        puts("exploration too narrow: adjust the scenarios");
        return 1;
    }
    puts("all safety rules held");
    return 0;
}
