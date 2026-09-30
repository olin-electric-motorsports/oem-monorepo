#ifndef VCU_CONFIG_H
#define VCU_CONFIG_H

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


/* Control calibration. */

/* Internal APPS representation: 0 counts = 0%, 255 counts = 100%. */
#define VCU_PEDAL_MIN_COUNTS (0)
#define VCU_PEDAL_MAX_COUNTS (255)

/* Required continuous APPS implausibility duration before the fault latches. */
#define VCU_APPS_IMPLAUSIBILITY_TIMEOUT_MS (100u)

/* Integer APPS thresholds derived from 25%, 5%, and 10% pedal travel. */
#define VCU_APPS_BRAKE_IMPLAUSIBILITY_SET_COUNTS (64)
#define VCU_APPS_BRAKE_IMPLAUSIBILITY_CLEAR_COUNTS (12)
#define VCU_APPS_MISMATCH_MAX_COUNTS (25)

/* Ignore small pedal movement/noise below this command threshold. */
#define VCU_PEDAL_IDLE_THRESHOLD_COUNTS (20)

/*
 * M192 torque is encoded in 0.1 N.m per raw count. This commissioning limit
 * preserves the previous maximum request of approximately 25.5 N.m; it must
 * be replaced with the approved vehicle torque limit before track operation.
 */
#define VCU_MAX_DRIVE_TORQUE_RAW (255)

/*
 * Preserve the existing raw direction command until a wheels-off-ground test
 * establishes which value produces vehicle-forward rotation.
 */
#define VCU_MOTOR_DIRECTION_COMMAND_RAW (0u)

/* Extra calibration margin applied before converting the 12-bit ADC to 10-bit. */
#define VCU_THROTTLE_CALIBRATION_MARGIN_COUNTS (-5)

/*
 * Minimum and maximum ADC counts representing 0% and 100% pedal travel.
 * Last calibrated 04-18-2026 for MKVIII.
 */
#define VCU_THROTTLE_L_MIN_COUNTS \
    ((36 + VCU_THROTTLE_CALIBRATION_MARGIN_COUNTS) >> 2)
#define VCU_THROTTLE_L_MAX_COUNTS \
    ((1990 - VCU_THROTTLE_CALIBRATION_MARGIN_COUNTS) >> 2)
#define VCU_THROTTLE_R_MIN_COUNTS \
    ((7 + VCU_THROTTLE_CALIBRATION_MARGIN_COUNTS) >> 2)
#define VCU_THROTTLE_R_MAX_COUNTS \
    ((3383 - VCU_THROTTLE_CALIBRATION_MARGIN_COUNTS) >> 2)
    
/* Heartbeat LED/CAN toggle period in milliseconds. */
#define VCU_HEARTBEAT_TOGGLE_MS (500u)

#endif /* VCU_CONFIG_H */
