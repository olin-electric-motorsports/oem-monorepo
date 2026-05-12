/*
 * Board-specific VCU configuration for MKVIII low-voltage hardware.
 *
 * Keep this file limited to pin mappings, ADC channel mappings, calibration
 * constants, and control thresholds that are consumed by the VCU firmware.
 */

// Pin definition - General
/* User-visible status LEDs driven directly by the MCU. */
#define VCU_HEARTBEAT_LED_GPIO_PORT (GPIOB)
#define VCU_HEARTBEAT_LED_GPIO_PIN (GPIO_PIN_6)     // Output
#define VCU_ERROR_LED_GPIO_PORT (GPIOB)
#define VCU_ERROR_LED_GPIO_PIN (GPIO_PIN_15)        // Output


// Pin definition - Throttle
//////////////////////// GPIO - THROTTLE ////////////////////////
/* Inertia switch shutdown-chain sense input. */
#define VCU_SS_IS_GPIO_PORT (GPIOB)
#define VCU_SS_IS_GPIO_PIN (GPIO_PIN_14)            // Input

////////////////////////// ADC - THROTTLE ////////////////////////
/* Redundant APPS channels sampled from ADC1. */
#define VCU_THROTTLE_L_ADC_INSTANCE (ADC1)
#define VCU_THROTTLE_L_ADC_PORT (GPIOA)
#define VCU_THROTTLE_L_ADC_PIN (GPIO_PIN_0)
#define VCU_THROTTLE_L_ADC_CHANNEL (ADC_CHANNEL_1)
#define VCU_THROTTLE_R_ADC_INSTANCE (ADC1)
#define VCU_THROTTLE_R_ADC_PORT (GPIOA)
#define VCU_THROTTLE_R_ADC_PIN (GPIO_PIN_1)
#define VCU_THROTTLE_R_ADC_CHANNEL (ADC_CHANNEL_2)

#define VCU_THROTTLE_ADC_SAMPLE_TIME (ADC_SAMPLETIME_47CYCLES_5)

// Pin definition - BSPD
//////////////////////// GPIO - BSPD ////////////////////////
// These are digital outputs for avr-controlled LED outputs
/* Diagnostic LEDs showing brake-light logic level and 5 kW BSPD state. */
#define VCU_BRAKE_LL_LED_GPIO_PORT (GPIOB)
#define VCU_BRAKE_LL_LED_GPIO_PIN (GPIO_PIN_8)
#define VCU_MOTOR_5KW_LED_GPIO_PORT (GPIOB)
#define VCU_MOTOR_5KW_LED_GPIO_PIN (GPIO_PIN_3)

// Monitor Pins connected to the logic-level (LL) side of the LSDs
/* Logic-level monitor inputs for the BSPD and brake-light low-side drivers. */
#define VCU_BSPD_LL_GPIO_PORT (GPIOB)
#define VCU_BSPD_LL_GPIO_PIN (GPIO_PIN_10)
#define VCU_BRAKELIGHT_LL_GPIO_PORT (GPIOA)
#define VCU_BRAKELIGHT_LL_GPIO_PIN (GPIO_PIN_8)

// Digital sense pin for 5kW motor "on" state
/* Digital indication that the motor power path has crossed the BSPD threshold. */
#define VCU_MOTOR_CURRENT_SENSE_GPIO_PORT (GPIOB)
#define VCU_MOTOR_CURRENT_SENSE_GPIO_PIN (GPIO_PIN_7)

// Input for shutdown sense line
/* Shutdown-chain monitor for the BSPD segment. */
#define VCU_BSPD_SHUTDOWN_SENSE_GPIO_PORT (GPIOB)
#define VCU_BSPD_SHUTDOWN_SENSE_GPIO_PIN (GPIO_PIN_13)

////////////////////////// ADC - BSPD ////////////////////////
// Monitor Pins for Brake Pressure Signals
/* Raw brake pressure sensor monitor. */
#define VCU_BRAKE_PRESSURE_SENSE_ADC_INSTANCE (ADC1)
#define VCU_BRAKE_PRESSURE_SENSE_ADC_PORT (GPIOA)
#define VCU_BRAKE_PRESSURE_SENSE_ADC_PIN (GPIO_PIN_2)
#define VCU_BRAKE_PRESSURE_SENSE_ADC_CHANNEL (ADC_CHANNEL_3)

/* Filtered brake pressure sensor monitor. */
#define VCU_BRAKE_PRESSURE_SENSE_FILTERED_ADC_INSTANCE (ADC1)
#define VCU_BRAKE_PRESSURE_SENSE_FILTERED_ADC_PORT (GPIOA)
#define VCU_BRAKE_PRESSURE_SENSE_FILTERED_ADC_PIN (GPIO_PIN_3)
#define VCU_BRAKE_PRESSURE_SENSE_FILTERED_ADC_CHANNEL (ADC_CHANNEL_4)

// Monitor Pin for RC Circuit, used to see how close RC circuit is to causing a fault, potentially
/* BSPD RC timing-node monitor, useful for seeing margin before hardware trip. */
#define VCU_RC_TIMER_STATUS_ADC_INSTANCE (ADC2)
#define VCU_RC_TIMER_STATUS_ADC_PORT (GPIOA)
#define VCU_RC_TIMER_STATUS_ADC_PIN (GPIO_PIN_4)
#define VCU_RC_TIMER_STATUS_ADC_CHANNEL (ADC_CHANNEL_17)


// implausibility constants according to rule T.4.2
/* Required continuous APPS implausibility duration before the fault latches. */
#define IMPLAUSIBILITY_TIME_LIMIT 100

/* Multiplier applied to scaled pedal travel before motor-controller limits. */
#define TORQUE_REQUEST_SCALE (1)

//BASED ON DIRECTION COMMANDS from the PM100DX DATASHEET...DON'T CHANGE!!
/* Direction-command encoding expected by the motor controller. */
#define MOTOR_CLOCKWISE     (1)
#define MOTOR_ANTICLOCKWISE (0)

/* Internal pedal travel range used by plausibility and torque calculations. */
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

#define THROTTLE_BUFFER (-5)

/*
Minimum and maximum ADC counts representing 0% and 100% pedal travel
Last calibrated 04-18-2026 for MKVIII 
*/
#define THROTTLE_L_MIN_COUNTS (int16_t)((36 + THROTTLE_BUFFER) >> 2)
#define THROTTLE_L_MAX_COUNTS (int16_t)((1990 - THROTTLE_BUFFER) >> 2)
#define THROTTLE_R_MIN_COUNTS (int16_t)((7 + THROTTLE_BUFFER) >> 2)
#define THROTTLE_R_MAX_COUNTS (int16_t)((3383 - THROTTLE_BUFFER) >> 2)
    
/* Heartbeat LED/CAN toggle period in milliseconds. */
#define HEARTBEAT_TOGGLE_MS 500
