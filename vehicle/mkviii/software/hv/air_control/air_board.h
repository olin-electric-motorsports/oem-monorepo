/* the board's pins and the functions in air_board.c */
#ifndef AIR_BOARD_H
#define AIR_BOARD_H
#include "stm32g4xx_hal.h"
#include "air.h"

/* Every pin the board uses; air_pins[] says where each one is */
typedef enum {
    AIR_PIN_PRECHARGE_CTL, AIR_PIN_MAIN_CTL,
    AIR_PIN_SS_TSMS, AIR_PIN_SS_IMD_LATCH, AIR_PIN_SS_MPC,
    AIR_PIN_SS_TSMP, AIR_PIN_SS_HVD, AIR_PIN_SS_BMS, AIR_PIN_SS_EMETER,
    AIR_PIN_IMD_SENSE, AIR_PIN_AIR_P_FEEDBACK, AIR_PIN_AIR_N_FEEDBACK,
    AIR_PIN_ERROR_LED, AIR_PIN_HEARTBEAT_LED, AIR_PIN_INIT_LED,
    AIR_PIN_CAN_RX, AIR_PIN_CAN_TX, AIR_PIN_COUNT
} air_pin_id_t;

/* One pin: its port and pin, which level means "on", its pull resistor, and
 * its alternate function (CAN pins only) */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_level;
    uint32_t pull;
    uint32_t alternate;
} air_pin_t;
extern const air_pin_t air_pins[AIR_PIN_COUNT]; /* filled in air_board_config.c */

/* Set up the pins; false if the pin table isn't ready */
bool air_board_init(void);
/* Read every input */
air_inputs_t air_board_read(void);
/* Drive the outputs and LEDs from the state machine */
void air_board_write(const air_t *air, uint32_t now);
/* Both outputs off, error LED on */
void air_board_safe(void);

#endif
