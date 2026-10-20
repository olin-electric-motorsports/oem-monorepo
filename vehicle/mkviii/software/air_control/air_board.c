/* the board's pins: checks the pin table, sets up the GPIO, reads the inputs
 * and drives the outputs and LEDs */
#include "air_board.h"
#include "air_board_config.h"
#include "air_config.h"

static bool ready; /* true once the pins are set up; until then no GPIO is touched */

/* LED pins are optional */
static bool is_led(unsigned i) {
    return i >= AIR_PIN_ERROR_LED && i <= AIR_PIN_INIT_LED;
}

/* Check the pin table: real ports, one pin per row, valid settings, no pin
 * used twice, and the debug (SWD) pins left alone */
static bool valid_mapping(void) {
    for (unsigned i = 0; i < AIR_PIN_COUNT; ++i) {
        const air_pin_t *p = &air_pins[i];
        if (is_led(i) && p->port == NULL && p->pin == 0) continue; /* unassigned LED: fine */
        if (p->port != GPIOA && p->port != GPIOB && p->port != GPIOC && p->port != GPIOF)
            return false;
        if (!p->pin || (p->pin & (p->pin - 1u))) return false; /* exactly one pin */
        if (p->active_level != GPIO_PIN_SET && p->active_level != GPIO_PIN_RESET) return false;
        if (p->pull != GPIO_NOPULL && p->pull != GPIO_PULLUP && p->pull != GPIO_PULLDOWN) return false;
        /* Retain SWD access; package availability still needs schematic review. */
        if (p->port == GPIOA && (p->pin & (GPIO_PIN_13 | GPIO_PIN_14))) return false;
        /* CAN pins must use FDCAN1, and no pin may be used twice */
        if (i >= AIR_PIN_CAN_RX && p->alternate != GPIO_AF9_FDCAN1) return false;
        for (unsigned j = 0; j < i; ++j)
            if (p->port == air_pins[j].port && p->pin == air_pins[j].pin) return false;
    }
    return true;
}

/* Turn an output on or off, flipping it for active-low pins. Unassigned pins
 * are skipped */
static void write_pin(air_pin_id_t id, bool on) {
    const air_pin_t *p = &air_pins[id];
    if (p->port != NULL) HAL_GPIO_WritePin(p->port, p->pin,
        on ? p->active_level : (p->active_level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

/* True when an input is in its "on" state (closed / OK); false until set up */
static bool read_pin(air_pin_id_t id) {
    const air_pin_t *p = &air_pins[id];
    return ready && HAL_GPIO_ReadPin(p->port, p->pin) == p->active_level;
}

/* Set up every pin, with outputs starting off. Refuses a bad or unfinished
 * pin table, and then never touches the GPIO */
bool air_board_init(void) {
    if (!AIR_BOARD_CONFIGURED || !valid_mapping()) return false;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    for (unsigned i = 0; i < AIR_PIN_COUNT; ++i) {
        const air_pin_t *p = &air_pins[i];
        if (p->port == NULL) continue;
        bool output = i <= AIR_PIN_MAIN_CTL || is_led(i); /* the two outputs and the LEDs */
        GPIO_InitTypeDef cfg = {0};
        cfg.Pin = p->pin;
        cfg.Pull = p->pull;
        cfg.Speed = GPIO_SPEED_FREQ_LOW;
        cfg.Mode = output ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
        if (output) write_pin((air_pin_id_t)i, false); /* Set latch BEFORE direction. */
        if (i >= AIR_PIN_CAN_RX) { /* the CAN pins go to the CAN peripheral */
            cfg.Mode = GPIO_MODE_AF_PP;
            cfg.Alternate = p->alternate;
            cfg.Speed = GPIO_SPEED_FREQ_HIGH;
        }
        HAL_GPIO_Init(p->port, &cfg);
    }
    ready = true;
    return true;
}

/* Read all the inputs for the state machine */
air_inputs_t air_board_read(void) {
    air_inputs_t in = {0};
    if (!ready) return in;
    for (unsigned i = AIR_PIN_SS_TSMS; i <= AIR_PIN_SS_EMETER; ++i)
        if (read_pin((air_pin_id_t)i)) in.shutdown_closed |= 1u << (i - AIR_PIN_SS_TSMS);
    in.imd_ok = read_pin(AIR_PIN_IMD_SENSE);
    in.air_p_closed = read_pin(AIR_PIN_AIR_P_FEEDBACK);
    in.air_n_closed = read_pin(AIR_PIN_AIR_N_FEEDBACK);
    return in;
}

/* Both outputs off and the error LED on */
void air_board_safe(void) {
    if (!ready) return;
    write_pin(AIR_PIN_MAIN_CTL, false);
    write_pin(AIR_PIN_PRECHARGE_CTL, false);
    write_pin(AIR_PIN_ERROR_LED, true);
}

/* Drive the outputs and LEDs from the state machine */
void air_board_write(const air_t *a, uint32_t now) {
    if (!ready) return;
    write_pin(AIR_PIN_MAIN_CTL, a->main_command);
    write_pin(AIR_PIN_PRECHARGE_CTL, a->precharge_command);
    write_pin(AIR_PIN_ERROR_LED, a->fault != AIR_FAULT_NONE);
    write_pin(AIR_PIN_INIT_LED, a->state == AIR_STATE_INIT);
    write_pin(AIR_PIN_HEARTBEAT_LED, (now / AIR_HEARTBEAT_PERIOD_MS) & 1u);
}
