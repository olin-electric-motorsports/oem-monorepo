#ifndef BRAKE_TEMP_H
#define BRAKE_TEMP_H

#include "stm32g4xx_hal.h"
#include "common/adc/adc.h"


//#define PA1_GPIO_Port   GPIOA
#define PA1_Pin         GPIO_PIN_1

// Define the Sensor Output Pin
#define IR_OUTPUT       GPIO_PIN_0

// Define the Debug & Heartbeat LED Pins
#define DEBUG_LED       GPIO_PIN_6
#define HEARTBEAT_LED   GPIO_PIN_8


// CAN_RX - PA11 

// CAN_TX - PA12 

// SWDIO - PA13 

// SWCLK - PA14 



// Function Prototypes



void GpioInit(void);
void SysTick_Handler(void);
void SystemClockConfig(void);
extern oem_adc_config_t brake_temp_sensor;
#endif // brake_temp