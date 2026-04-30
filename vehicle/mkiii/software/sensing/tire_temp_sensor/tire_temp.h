
//pin definitions
#include "stm32g4xx_hal.h"
#pragma once
//Debug LEDs
#define DEBUG_LED_PORT GPIOA //sigital
#define DEBUG_LED_PIN  GPIO_PIN_6 //digital

#define HEARTBEAT_LED_PORT GPIOA
#define HEARTBEAT_LED_PIN  GPIO_PIN_8
//gpio_t DEBUG_LED = PA6;
//gpio_t HEARTBEAT_LED = PA8;
/*
//sensors pins
#define SDA_PORT GPIOB
#define SDA_PIN  GPIO_PIN_7

//gpio_t SDA = PB7;
//gpio_t SCL = PB8;
#define SCL_PORT GPIOB
#define SCL_PIN  GPIO_PIN_8
*/
// Function Prototypes

void SystemClockConfig();
void GpioInit(void);
void SysTick_Handler(void);
void SystemClockConfig(void);
