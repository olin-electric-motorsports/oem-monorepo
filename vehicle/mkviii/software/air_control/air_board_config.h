#ifndef AIR_BOARD_CONFIG_H
#define AIR_BOARD_CONFIG_H

/* Set only after filling air_pins[] AND reviewing polarity, CAN and thresholds. */
#ifndef AIR_BOARD_CONFIGURED
#define AIR_BOARD_CONFIGURED 0
#endif

/* HSI16 / (2 * (1 + 13 + 2)) = 500 kbit/s; confirm vehicle bus bitrate.
 * HSI accuracy must be validated on the bench; use a reviewed external clock
 * configuration if required by the board/network timing budget. */
#define AIR_CAN_PRESCALER 2u
#define AIR_CAN_SEG1 13u
#define AIR_CAN_SEG2 2u
#define AIR_CAN_SJW 2u

#endif
