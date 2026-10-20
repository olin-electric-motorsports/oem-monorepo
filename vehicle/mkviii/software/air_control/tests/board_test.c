/* GPIO tests with a fake HAL: a bad or unfinished pin table must not touch any
 * pin, and a good one must set safe output levels first and get polarity right */
#include "air_board.h"
#include "air_board_config.h"
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

/* Which case runs depends on the -D flags that run_host_tests.py passes */
int main(void) {
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
