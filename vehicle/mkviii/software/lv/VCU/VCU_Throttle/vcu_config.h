// Port definition (GPIO only) Are those adresses?
#define VCU_SS_IS_GPIO_PORT (GPIOB)
#define VCU_HEARTBEAT_LED_GPIO_PORT (GPIOB)
#define VCU_ERROR_LED_GPIO_PORT (GPIOB)

// Pin definition
#define VCU_SS_IS_GPIO_PIN (GPIO_PIN_14)            // Port B Input
#define VCU_HEARTBEAT_LED_GPIO_PIN (GPIO_PIN_6)     // Port B Output
#define VCU_ERROR_LED_GPIO_PIN (GPIO_PIN_15)        // Port B Output
#define VCU_APPS1_ADC_CHANNEL (ADC_CHANNEL_1) // PA0 or ADC1_IN1?
#define VCU_APPS2_ADC_CHANNEL (ADC_CHANNEL_2) // PA1 or ADC1_IN2?

// implausibility constants according to rule T.4.2
#define IMPLAUSIBILITY_TIME_LIMIT 100

#define TORQUE_REQUEST_SCALE (1)

//BASED ON DIRECTION COMMANDS from the PM100DX DATASHEET...DON'T CHANGE!!
#define MOTOR_CLOCKWISE     (1)
#define MOTOR_ANTICLOCKWISE (0)

#define MIN_THROTTLE_POS 0
#define MAX_THROTTLE_POS 255

/*
Represents value for 25% pedal travel to check for brake implausibility
Set as 25% of 255
*/
#define APPS_BRAKE_IMPLAUSIBILITY_THRESHOLD (0.25 * MAX_THROTTLE_POS)

/*
Represents value for 5% pedal travel to check for brake implausibility
Set as 5% of 255
*/
#define APPS_BRAKE_IMPLAUSIBILITY_THRESHOLD_LOW (0.05 * MAX_THROTTLE_POS)

/*
Represents value for 10% deviation between throttle sensors
Set as 10% of 255
*/
#define APPS_IMPLAUSIBILITY_DEVIATION_THRESHOLD (0.1 * MAX_THROTTLE_POS)

/*
buffer here to make sure we don't request torque when small deviations
in potentiometer happen and we aren't pressing pedal
*/

#define THROTTLE_BUFFER (5)

/*
Minimum and maximum ADC counts representing 0% and 100% pedal travel
Last calibrated 04-24-2025 for MKVII
*/
#define THROTTLE_R_MIN_COUNTS (int16_t)((20 + THROTTLE_BUFFER) >> 2)
#define THROTTLE_R_MAX_COUNTS (int16_t)((667 - THROTTLE_BUFFER) >> 2)
#define THROTTLE_L_MIN_COUNTS (int16_t)((30 + THROTTLE_BUFFER) >> 2)
#define THROTTLE_L_MAX_COUNTS (int16_t)((1023 - THROTTLE_BUFFER) >> 2)

/*
 * Sets the torque request in the motor controller command message
 */
#define SET_TORQUE_REQUEST(torque) \
    (m192_command_message.torque_command = (torque))