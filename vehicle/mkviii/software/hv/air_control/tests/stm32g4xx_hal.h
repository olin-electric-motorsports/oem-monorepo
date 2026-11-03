/* a fake STM32 HAL with only the GPIO parts, so board_test.c runs on a PC.
 * Never use this folder in the firmware build */
#ifndef AIR_TEST_HAL_H
#define AIR_TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
/* One fake port: input levels, output levels, pins set up, outputs written */
typedef struct { uint16_t input, output, initialized, written; } GPIO_TypeDef;
extern GPIO_TypeDef test_ports[4];
#define GPIOA (&test_ports[0])
#define GPIOB (&test_ports[1])
#define GPIOC (&test_ports[2])
#define GPIOF (&test_ports[3])
/* The real HAL's names, with simple values */
#define GPIO_PIN_0 (1u << 0)
#define GPIO_PIN_1 (1u << 1)
#define GPIO_PIN_2 (1u << 2)
#define GPIO_PIN_3 (1u << 3)
#define GPIO_PIN_4 (1u << 4)
#define GPIO_PIN_5 (1u << 5)
#define GPIO_PIN_6 (1u << 6)
#define GPIO_PIN_7 (1u << 7)
#define GPIO_PIN_8 (1u << 8)
#define GPIO_PIN_9 (1u << 9)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_14 (1u << 14)
#define GPIO_PIN_15 (1u << 15)
#define GPIO_NOPULL 0u
#define GPIO_PULLUP 1u
#define GPIO_PULLDOWN 2u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_SPEED_FREQ_HIGH 2u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_INPUT 0u
#define GPIO_MODE_AF_PP 2u
#define GPIO_AF9_FDCAN1 9u
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() ((void)0)
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Pull, Speed, Mode, Alternate; } GPIO_InitTypeDef;
/* Implemented in board_test.c */
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *cfg);
#endif
