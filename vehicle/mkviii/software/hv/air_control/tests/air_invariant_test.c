/* random test: a simple model of the car drives the firmware for 10 million
 * 1 ms steps (about 3 hours), with faults thrown in, and checks the rules in
 * check_rules() after every step: the safety rules, plus four rules against
 * false alarms. The same seed always gives the same run, and a failure prints
 * the command to rerun it.
 *
 * The model has the shutdown loop, both AIRs, the precharge relay, the DC
 * link, and BMS and IVT CAN messages. Each power-up a pretend driver opens and
 * closes the loop, one scenario from scenario_t may add a fault, and late main
 * loops, lost messages and rare input glitches happen all the time.
 *
 * Usage: air_invariant_test [seed] [steps]
 */
#include "../air.h"
#include "../air_config.h"
#include "../air_protocol.h"
#include <stdio.h>
#include <stdlib.h>

#define DEFAULT_SEED 0x41495231ull /* "AIR1" */
#define DEFAULT_STEPS 10000000ul   /* about 3 hours of simulated time */

/* Random numbers that come out the same on every computer (xorshift64*) */
static uint64_t rng;

static uint32_t rnd(void) {
    rng ^= rng >> 12;
    rng ^= rng << 25;
    rng ^= rng >> 27;
    return (uint32_t)((rng * 2685821657736338717ull) >> 32);
}

/* A random number from lo to hi, and true about 1 time in n */
static uint32_t between(uint32_t lo, uint32_t hi) { return lo + rnd() % (hi - lo + 1u); }
static bool one_in(uint32_t n) { return rnd() % n == 0u; }

/* Random noise between -amplitude and +amplitude */
static double noise(double amplitude) {
    return amplitude * ((double)between(0, 2000) - 1000.0) / 1000.0;
}

/* True once time t has reached the deadline, even when the ms counter wraps */
static bool reached(uint32_t t, uint32_t deadline) {
    return (uint32_t)(t - deadline) < 0x80000000u;
}

/* A contactor whose contacts follow its coil after a delay */
typedef struct {
    bool coil, contact;                                 /* coil powered; contacts closed */
    bool welded, stuck_open, reads_open, reads_closed; /* faults it can have */
    uint32_t coil_ms, close_ms, open_ms;                /* coil change time; close and open delays */
} contactor_t;

/* A contactor that closes in close_lo to close_hi ms and opens in 2 to 30 ms */
static contactor_t new_contactor(uint32_t close_lo, uint32_t close_hi) {
    contactor_t c = {0};
    c.close_ms = between(close_lo, close_hi);
    c.open_ms = between(2, 30);
    return c;
}

/* Move the contacts to match the coil once the delay has passed; a weld
 * holds them shut */
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

/* What the feedback wire says, which a broken wire can get wrong */
static bool reads_closed(const contactor_t *c) {
    if (c->reads_closed) return true;
    if (c->reads_open) return false;
    return c->contact;
}

/* At most one scenario per power-up */
typedef enum {
    S_NONE,                   /* no fault */
    S_WELD_LOOP_AIR,          /* the AIR the loop closes welds shut */
    S_WELD_DRIVEN_AIR,        /* the AIR this board drives welds shut */
    S_LOOP_AIR_STUCK_OPEN,    /* the loop's AIR never closes */
    S_DRIVEN_AIR_STUCK_OPEN,  /* our AIR never closes */
    S_SLOW_CONTACTOR,         /* closes after AIR_CONTACTOR_CLOSE_MS */
    S_DRIVEN_FEEDBACK_OPEN,   /* broken feedback wire */
    S_FEEDBACK_CLOSED_AT_BOOT, /* feedback reads closed from power-up */
    S_NO_PRECHARGE,           /* open precharge resistor */
    S_SLOW_PRECHARGE,         /* precharge time constant of 2 to 4 s */
    S_SLOW_DISCHARGE,         /* discharge time constant of 3 to 8 s */
    S_CHARGED_AT_BOOT,        /* DC link charged, discharge circuit dead */
    S_TSMS_CLOSED_AT_BOOT,    /* the loop is already closed at power-up */
    S_QUICK_RECLOSE,          /* driver closes the loop again while discharging */
    S_NODE_SENSE_FAULT,       /* one node reads wrong for a few ms, every few seconds */
    S_NOISY_INPUTS,           /* frequent 1 ms glitches */
    S_IMD_FAULT,              /* the IMD reports a fault for a moment */
    S_BMS_SILENT,             /* the BMS stops sending for over 1 s */
    S_IVT_SILENT,             /* the IVT stops sending for over 0.5 s */
    S_BMS_FAULT_BIT,          /* the BMS sets one fault bit */
    S_BMS_FAULT_STATE,        /* the BMS goes into its fault state */
    S_BMS_CHARGING,           /* the BMS says it's charging */
    S_PACK_LOW,               /* pack under 200 V */
    S_IVT_ERROR,              /* the IVT sets an error flag */
    S_IVT_NEGATIVE,           /* one negative voltage reading */
    S_WATCHDOG_RESETS,        /* the controller keeps resetting */
    SCENARIO_COUNT
} scenario_t;

/* Everything about the simulated car for one power-up */
typedef struct {
    scenario_t scenario;        /* which fault this power-up has */
    uint32_t end_ms;            /* power-up ends at the first IDLE after this */
    uint32_t event_ms;          /* when the scenario's fault starts */
    bool event_done;
    bool loop;                  /* the real shutdown loop is closed */
    uint32_t toggle_ms;         /* when the driver next opens or closes it */
    uint8_t open_nodes;         /* nodes that still read closed while it is open */
    bool sensing_wrong;         /* a node is reading wrong: which one, until when */
    uint8_t wrong_node;
    uint32_t sensing_wrong_until;
    bool imd_ok;                /* IMD health, and when a bad spell ends */
    uint32_t imd_bad_until;
    bool bms_quiet, ivt_quiet;  /* BMS or IVT not sending, and until when */
    uint32_t bms_quiet_until, ivt_quiet_until;
    uint16_t bms_fault_bits;    /* what the BMS and IVT report */
    bool bms_fault_state, ivt_error, ivt_negative;
    contactor_t loop_air, driven_air, precharge;
    double pack_v, link_v, tau_precharge_ms, tau_discharge_ms; /* volts; time constants */
    uint32_t bms_next, ivt_next, reset_ms; /* next BMS and IVT messages; next reset */
    uint8_t ivt_counter;        /* the IVT's message counter */
    bool faulted;               /* the firmware latched a fault */
    uint32_t restart_ms;        /* the car is power-cycled after a fault */
} plant_t;

/* The simulated car, the firmware's state, inputs and CAN data, and the clock */
static plant_t p;
static air_t a;
static air_inputs_t in;
static air_measurements_t data;
static uint32_t now, boot_ms;   /* boot_ms: when the controller last started */
static uint32_t loop_closed_ms; /* last step on which all 7 nodes read closed */
static uint32_t plausible_ms;   /* last step on which TSMS read open or all 7 closed */
static uint64_t seed;
/* Counters for the summary at the end */
static unsigned long step_no, power_ups, active_entries, held_steps;
static unsigned long idle_glitch_steps, idle_implausible_faults;
static unsigned long state_entries[AIR_STATE_FAULT + 1];
static unsigned long faults[AIR_FAULT_IVT_STATUS + 1];

/* Fault names for printing; the assert makes sure there's one per fault code */
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

/* A rule broke: print which one, what was going on, and the command to rerun
 * this exact case, then stop */
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

/* Power-up or reset: the controller starts over and RAM is cleared */
static void boot_controller(void) {
    air_init(&a, now);
    data = (air_measurements_t){0};
    boot_ms = now;
}

/* A new power-up: pick a scenario (none 30% of the time) and random values
 * for the car */
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

    /* Scenarios that change the car from the start */
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

/* Scenarios that start partway through: start the fault once its time comes */
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
        p.event_done = false; /* an intermittent input: it happens again */
        p.event_ms = now + between(1000, 6000);
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

/* The driver closes the loop after power-up, then opens and closes it at random */
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

/* Move the contactors and the DC link voltage forward by dt ms */
static void plant_update(uint32_t dt) {
    contactor_update(&p.loop_air, p.loop, now);
    /* This board's driver and the precharge relay coil both get loop power. */
    contactor_update(&p.driven_air, p.loop && a.main_command, now);
    contactor_update(&p.precharge, p.loop && a.precharge_command, now);

    /* The DC link charges toward the pack through the resistor or our AIR,
     * and drains through the discharge circuit while the loop is open */
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

/* Read the inputs like the board would: shutdown nodes (with glitches), the
 * IMD and the AIR feedback wires */
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

/* True if this CAN message gets lost: during a quiet spell, or 1 in 200 at random */
static bool frame_lost(bool *quiet, uint32_t until) {
    if (*quiet && reached(now, until)) *quiet = false;
    return *quiet || one_in(200);
}

/* Every 10 ms, send a BMS message as real CAN bytes through the firmware's decoder */
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

/* Same for the IVT's DC link voltage message, with 0.3 V of noise */
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

/* The same freshness and health checks the firmware makes on CAN data */
static bool data_healthy(void) {
    return data.bms_seen && (uint32_t)(now - data.bms_ms) < AIR_BMS_TIMEOUT_MS &&
           data.ivt_seen && (uint32_t)(now - data.ivt_ms) < AIR_IVT_TIMEOUT_MS &&
           data.bms_fault == 0 && data.bms_state < 2 && data.pack_mv >= AIR_PACK_MIN_MV &&
           !data.ivt_error && data.tractive_mv >= 0;
}

/* The rules. They must hold after every step, whatever the inputs */
static void check_rules(const air_t *before) {
    const bool driven_closed = AIR_CONTROLLED_NEGATIVE ? in.air_n_closed : in.air_p_closed;
    const bool loop_air_closed = AIR_CONTROLLED_NEGATIVE ? in.air_p_closed : in.air_n_closed;
    const bool all_closed = in.shutdown_closed == AIR_SS_ALL;
    /* A node open for less than the debounce may be a glitch: commands that are
     * already on may stay on through it, whatever the AIR feedback reads. */
    const bool debouncing =
        !all_closed && (uint32_t)(now - loop_closed_ms) < AIR_SHUTDOWN_DEBOUNCE_MS;
    const bool ready =
        ((all_closed && loop_air_closed) || debouncing) && in.imd_ok && data_healthy();
    /* TSMS, the final node, reading closed while an upstream node reads open. */
    const bool implausible = (in.shutdown_closed & AIR_SS_TSMS) && !all_closed;
    const bool implausible_long =
        implausible && (uint32_t)(now - plausible_ms) >= AIR_SHUTDOWN_DEBOUNCE_MS;
    const bool commanding = a.main_command || a.precharge_command;
    const bool turned_on = (a.main_command && !before->main_command) ||
                           (a.precharge_command && !before->precharge_command);

    /* Safety rules */
    if ((unsigned)a.state > AIR_STATE_FAULT) fail("the state is always a valid value");
    if (a.main_command && a.state != AIR_STATE_TS_ACTIVE)
        fail("AIR- is only commanded in TS_ACTIVE");
    if (a.main_command && !ready)
        fail("AIR- is only commanded with all 7 nodes closed (or open for less than the "
             "debounce), the IMD OK, fresh healthy CAN data and the loop's AIR closed");
    if (a.main_command && !before->main_command &&
        (driven_closed ||
         (uint64_t)data.tractive_mv * 100u < (uint64_t)data.pack_mv * AIR_PRECHARGE_PERCENT))
        fail("AIR- is first commanded only at 95% of pack voltage, while it still reads open");
    if (a.precharge_command &&
        !(ready && (a.state == AIR_STATE_PRECHARGE || a.state == AIR_STATE_TS_ACTIVE)))
        fail("precharge is only commanded while closing, with all 7 nodes closed (or open "
             "for less than the debounce), the IMD OK, fresh healthy CAN data and the "
             "loop's AIR closed");
    if (!all_closed && turned_on) fail("no command turns on while a node reads open");
    if (!all_closed && commanding && !debouncing)
        fail("a node open for the whole debounce drops both commands");
    if (a.fault != AIR_FAULT_NONE && (a.state != AIR_STATE_FAULT || commanding))
        fail("a fault means the FAULT state with both commands off");
    if (before->fault != AIR_FAULT_NONE && a.fault != before->fault)
        fail("a latched fault never changes or clears");
    if ((uint32_t)(now - boot_ms) < AIR_IMD_SETTLE_MS && commanding)
        fail("nothing is commanded in the first 4 s after power-up");
    if (before->state == AIR_STATE_INIT && (in.shutdown_closed & AIR_SS_TSMS) &&
        a.fault == AIR_FAULT_NONE)
        fail("TSMS closed during startup always faults");
    if (before->state == AIR_STATE_IDLE && implausible_long && a.fault == AIR_FAULT_NONE)
        fail("in IDLE, TSMS closed with an upstream node open for the whole debounce faults");

    /* No false alarms. */
    if (a.fault != before->fault) {
        const bool weld = a.fault == AIR_FAULT_AIR_N_WELD || a.fault == AIR_FAULT_AIR_P_WELD ||
                          a.fault == AIR_FAULT_BOTH_AIRS_WELD;
        const bool stuck = p.loop_air.welded || p.driven_air.welded ||
                           p.loop_air.reads_closed || p.driven_air.reads_closed;
        if (weld && !stuck) fail("a weld fault means a contactor really is stuck closed");
        if (a.fault == AIR_FAULT_TRACTIVE_VOLTAGE && before->state == AIR_STATE_IDLE &&
            data.tractive_mv >= 0 && p.link_v < 5.5)
            fail("measurement noise near 5 V never latches TRACTIVE_VOLTAGE in IDLE");
        if (a.fault == AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY && before->state == AIR_STATE_IDLE &&
            !implausible_long)
            fail("a reading shorter than the debounce never latches SHUTDOWN_IMPLAUSIBILITY "
                 "in IDLE");
        /* A time constant of 2 s takes 440 V to 36 V by 5 s and 3 V by 10 s. */
        if (a.fault == AIR_FAULT_DISCHARGE_FAIL && p.tau_discharge_ms <= 2000.0)
            fail("DISCHARGE_FAIL means the discharge circuit really is too slow");
    }
}

/* Count states and faults for the summary, and power-cycle the car after a fault */
static void record(const air_t *before) {
    if (a.state != before->state) {
        ++state_entries[a.state];
        if (a.state == AIR_STATE_TS_ACTIVE) ++active_entries;
    }
    if (in.shutdown_closed != AIR_SS_ALL && (a.main_command || a.precharge_command)) ++held_steps;
    if (before->state == AIR_STATE_IDLE && a.state == AIR_STATE_IDLE &&
        (in.shutdown_closed & AIR_SS_TSMS) && in.shutdown_closed != AIR_SS_ALL)
        ++idle_glitch_steps;
    if (a.fault == AIR_FAULT_NONE || before->fault != AIR_FAULT_NONE) return;
    ++faults[a.fault];
    if (a.fault == AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY && before->state == AIR_STATE_IDLE)
        ++idle_implausible_faults;
    p.faulted = true;
    p.restart_ms = now + between(50, 1500);
}

/* Run the steps, then print a summary and make sure the model really reached
 * every fault and state */
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
        /* The same order as the firmware's main loop: inputs, CAN, then one step */
        plant_update(dt);
        sense();
        if (in.shutdown_closed == AIR_SS_ALL) loop_closed_ms = now;
        if (in.shutdown_closed == AIR_SS_ALL || !(in.shutdown_closed & AIR_SS_TSMS))
            plausible_ms = now;
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
    printf("\ncommands held with a node reading open (inside the debounce): %lu steps\n",
           held_steps);
    printf("IDLE kept waiting with TSMS closed before an upstream node (inside the debounce): "
           "%lu steps; faulted on it %lu times\n", idle_glitch_steps, idle_implausible_faults);

    if (steps < DEFAULT_STEPS) {
        puts("all rules held (short run: exploration not checked)");
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
    if (AIR_SHUTDOWN_DEBOUNCE_MS > 1u && !held_steps) {
        puts("the model never held a command through the debounce");
        missed = true;
    }
    if (AIR_SHUTDOWN_DEBOUNCE_MS > 1u && !idle_glitch_steps) {
        puts("the model never kept IDLE waiting through TSMS closed before an upstream node");
        missed = true;
    }
    if (!idle_implausible_faults) {
        puts("the model never produced SHUTDOWN_IMPLAUSIBILITY in IDLE");
        missed = true;
    }
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
    puts("all rules held");
    return 0;
}
