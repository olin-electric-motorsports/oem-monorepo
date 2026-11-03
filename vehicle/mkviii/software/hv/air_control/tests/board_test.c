/* GPIO tests with a fake HAL: a bad or unfinished pin table must not touch any
 * pin, and a good one (the test table below, or a filled-in air_board_config.c)
 * must set safe output levels first and get polarity right */
#include "../air_board.h"
#include "../air_board_config.h"
#include <stdio.h>
#include <stdlib.h>
/* Prints the failing line and stops */
#define CHECK(e) do { if (!(e)) { fprintf(stderr, "line %d: %s\n", __LINE__, #e); exit(1); } } while (0)
GPIO_TypeDef test_ports[4]; /* four fake GPIO ports: A, B, C and F */
static unsigned calls;      /* how many HAL calls were made */

#ifdef TEST_ASSIGNED_BOARD
/* Simulation only. These are NOT PCB assignments. MAIN is active-low on purpose. */
#define PIN(n, active) {GPIOA, 1u << (n), active, GPIO_NOPULL, 0}
const air_pin_t air_pins[AIR_PIN_COUNT] = {
    [AIR_PIN_PRECHARGE_CTL] = PIN(0, GPIO_PIN_SET),
#ifdef TEST_DUPLICATE_PIN
    [AIR_PIN_MAIN_CTL] = PIN(0, GPIO_PIN_RESET),
#else
    [AIR_PIN_MAIN_CTL] = PIN(1, GPIO_PIN_RESET),
#endif
    [AIR_PIN_SS_TSMS] = PIN(2, GPIO_PIN_RESET),
    [AIR_PIN_SS_IMD_LATCH] = PIN(3, GPIO_PIN_RESET),
    [AIR_PIN_SS_MPC] = PIN(4, GPIO_PIN_RESET),
    [AIR_PIN_SS_TSMP] = PIN(5, GPIO_PIN_RESET),
    [AIR_PIN_SS_HVD] = PIN(6, GPIO_PIN_RESET),
    [AIR_PIN_SS_BMS] = PIN(7, GPIO_PIN_RESET),
    [AIR_PIN_SS_EMETER] = PIN(8, GPIO_PIN_RESET),
    [AIR_PIN_IMD_SENSE] = PIN(9, GPIO_PIN_SET),
    [AIR_PIN_AIR_P_FEEDBACK] = PIN(10, GPIO_PIN_SET),
    [AIR_PIN_AIR_N_FEEDBACK] = PIN(11, GPIO_PIN_SET),
    [AIR_PIN_CAN_RX] = {GPIOB, 1u << 8, GPIO_PIN_SET, GPIO_NOPULL, 9},
    [AIR_PIN_CAN_TX] = {GPIOB, 1u << 9, GPIO_PIN_SET, GPIO_NOPULL, 9},
};
#endif

/* Fake HAL: remember what was written, read and set up instead of touching hardware */
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState level) {
    ++calls;
    if (level == GPIO_PIN_SET) p->output |= pin;
    else p->output &= ~pin;
    p->written |= pin;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) {
    ++calls;
    return (p->input & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *cfg) {
    ++calls;
    /* Output latch must already be written before direction changes. */
    if (cfg->Mode == GPIO_MODE_OUTPUT_PP) CHECK(p->written & cfg->Pin);
    p->initialized |= cfg->Pin;
}

#if AIR_BOARD_CONFIGURED && !defined(TEST_ASSIGNED_BOARD)
/* The real table in air_board_config.c, switched on. While a required row is
 * still empty it must be refused like any bad table; once filled it must work */
static bool is_led(unsigned id) { return id >= AIR_PIN_ERROR_LED && id <= AIR_PIN_INIT_LED; }

static bool filled_in(void) {
    for (unsigned id = 0; id < AIR_PIN_COUNT; ++id)
        if (!is_led(id) && air_pins[id].port == NULL) return false;
    return true;
}

/* Is the board driving this output at its active level? */
static bool driven(unsigned id) {
    const air_pin_t *p = &air_pins[id];
    return ((p->port->output & p->pin) != 0) == (p->active_level == GPIO_PIN_SET);
}

/* Put an input at its active (closed / OK) or inactive level */
static void drive(unsigned id, bool active) {
    const air_pin_t *p = &air_pins[id];
    if (active == (p->active_level == GPIO_PIN_SET)) p->port->input |= p->pin;
    else p->port->input &= ~p->pin;
}

/* What air_board_read reports for one input */
static bool reads(air_inputs_t in, unsigned id) {
    if (id <= AIR_PIN_SS_EMETER) return in.shutdown_closed & (1u << (id - AIR_PIN_SS_TSMS));
    if (id == AIR_PIN_IMD_SENSE) return in.imd_ok;
    return id == AIR_PIN_AIR_P_FEEDBACK ? in.air_p_closed : in.air_n_closed;
}

static void check_filled_table(void) {
    CHECK(air_board_init()); /* valid: real ports, one pin per row, no duplicates, SWD free */
    for (unsigned id = 0; id < AIR_PIN_COUNT; ++id) /* every output and LED starts off */
        if ((id <= AIR_PIN_MAIN_CTL || is_led(id)) && air_pins[id].port) CHECK(!driven(id));
    /* Each input on its own: it, and only it, reads closed / OK */
    for (unsigned id = AIR_PIN_SS_TSMS; id <= AIR_PIN_AIR_N_FEEDBACK; ++id) drive(id, false);
    for (unsigned id = AIR_PIN_SS_TSMS; id <= AIR_PIN_AIR_N_FEEDBACK; ++id) {
        drive(id, true);
        air_inputs_t in = air_board_read();
        for (unsigned other = AIR_PIN_SS_TSMS; other <= AIR_PIN_AIR_N_FEEDBACK; ++other)
            CHECK(reads(in, other) == (other == id));
        drive(id, false);
    }
    /* Each output follows its own command, and air_board_safe turns both off */
    air_t a = {.main_command = true};
    air_board_write(&a, 0);
    CHECK(driven(AIR_PIN_MAIN_CTL) && !driven(AIR_PIN_PRECHARGE_CTL));
    a = (air_t){.precharge_command = true};
    air_board_write(&a, 0);
    CHECK(!driven(AIR_PIN_MAIN_CTL) && driven(AIR_PIN_PRECHARGE_CTL));
    air_board_safe();
    CHECK(!driven(AIR_PIN_MAIN_CTL) && !driven(AIR_PIN_PRECHARGE_CTL));
    if (air_pins[AIR_PIN_ERROR_LED].port) CHECK(driven(AIR_PIN_ERROR_LED));
}
#endif

/* Which case runs depends on the -D flags that run_host_tests.py passes */
int main(void) {
#if AIR_BOARD_CONFIGURED && !defined(TEST_ASSIGNED_BOARD)
    if (filled_in()) {
        check_filled_table();
        puts("AIR: GPIO test of the filled-in pin table passed");
        return 0;
    }
#endif
#if !AIR_BOARD_CONFIGURED || !defined(TEST_ASSIGNED_BOARD) || defined(TEST_DUPLICATE_PIN)
    /* Bad, unfinished or switched-off pin table: init refuses and nothing is touched */
    CHECK(!air_board_init());
    air_board_safe();
    air_t a = {.main_command = true, .precharge_command = true};
    air_board_write(&a, 0);
    air_inputs_t in = air_board_read();
    CHECK(!in.shutdown_closed && !in.imd_ok);
    CHECK(calls == 0); /* Incomplete/disabled/duplicate map touches NO GPIO. */
#else
    /* Good pin table: outputs start off, and every pin follows its polarity */
    CHECK(air_board_init());
    CHECK(GPIOA->output == 2); /* Active-low MAIN is inactive/high. */
    GPIOA->input = (1u << 9) | (1u << 10); /* nodes low = closed, IMD OK, AIR+ closed */
    air_inputs_t in = air_board_read();
    CHECK(in.shutdown_closed == AIR_SS_ALL && in.imd_ok && in.air_p_closed && !in.air_n_closed);
    GPIOA->input |= 1u << 7; /* the BMS node goes high = open */
    CHECK(air_board_read().shutdown_closed == (AIR_SS_ALL & ~AIR_SS_BMS));
    air_t a = {.main_command = true, .precharge_command = true};
    air_board_write(&a, 0);
    CHECK(GPIOA->output == 1); /* precharge on (high), MAIN on (low) */
    air_board_safe();
    CHECK(GPIOA->output == 2); /* both off again */
#endif
    puts("AIR: GPIO configuration/interlock test passed");
    return 0;
}
