#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

/*
#include "common/spi/api.h"

#include "third_party/MCP25625/MCP25625.h"
*/

/*
 * =========================
 * Charger limits
 * =========================
 */
#define TARGET_PACK_VOLTAGE_V   (428.0f)
#define CHARGING_MAX_VOLTAGE_V  (320.1f)
#define CHARGING_MAX_CURRENT_A  (58.2f)

/*
 * =========================
 * Timer selection
 * =========================
 */
#define CHARGER_MAIN_TIM_INSTANCE TIM2

/*
 * =========================
 * LCD geometry / colors
 * =========================
 */
#define LCD_WIDTH   240U
#define LCD_HEIGHT  320U

#define LCD_COLOR_BLACK   0x0000
#define LCD_COLOR_RED     0xF800
#define LCD_COLOR_GREEN   0x07E0
#define LCD_COLOR_BLUE    0x001F
#define LCD_COLOR_YELLOW  0xFFE0

/*
 * =========================
 * LCD ST7789 commands
 * =========================
 */
#define LCD_CMD_SWRESET   0x01
#define LCD_CMD_SLPOUT    0x11
#define LCD_CMD_COLMOD    0x3A
#define LCD_CMD_MADCTL    0x36
#define LCD_CMD_INVON     0x21
#define LCD_CMD_CASET     0x2A
#define LCD_CMD_RASET     0x2B
#define LCD_CMD_RAMWR     0x2C
#define LCD_CMD_DISPON    0x29
#define LCD_CMD_PORCTRL   0xB2
#define LCD_CMD_GCTRL     0xB7
#define LCD_CMD_VCOMS     0xBB
#define LCD_CMD_LCMCTRL   0xC0
#define LCD_CMD_VDVVRHEN  0xC2
#define LCD_CMD_VRHS      0xC3
#define LCD_CMD_VDVS      0xC4
#define LCD_CMD_FRCTRL2   0xC6
#define LCD_CMD_PWCTRL1   0xD0

/*
 * Core control / status pins
 */
#define SWDIO_GPIO_Port                 GPIOA
#define SWDIO_Pin                       GPIO_PIN_13

#define SWCLK_GPIO_Port                 GPIOA
#define SWCLK_Pin                       GPIO_PIN_14

#define CAN_RX_GPIO_Port                GPIOA
#define CAN_RX_Pin                      GPIO_PIN_11

#define CAN_TX_GPIO_Port                GPIOA
#define CAN_TX_Pin                      GPIO_PIN_12

#define CAN_STATUS_GPIO_Port            GPIOC
#define CAN_STATUS_Pin                  GPIO_PIN_13

#define HEARTBEAT_GPIO_Port             GPIOC
#define HEARTBEAT_Pin                   GPIO_PIN_14

#define SPI_STATUS_GPIO_Port            GPIOA
#define SPI_STATUS_Pin                  GPIO_PIN_0

#define CHARGING_STATUS_GPIO_Port       GPIOA
#define CHARGING_STATUS_Pin             GPIO_PIN_1

#define FULLY_CHARGED_STATUS_GPIO_Port  GPIOA
#define FULLY_CHARGED_STATUS_Pin        GPIO_PIN_2

#define ELCON_READY_GPIO_Port           GPIOA
#define ELCON_READY_Pin                 GPIO_PIN_3

#define CAR_READY_GPIO_Port             GPIOA
#define CAR_READY_Pin                   GPIO_PIN_4

#define CHARGING_ON_OFF_STATUS_GPIO_Port GPIOA
#define CHARGING_ON_OFF_STATUS_Pin       GPIO_PIN_5

#define EVSE_PWM_GPIO_Port              GPIOA
#define EVSE_PWM_Pin                    GPIO_PIN_9

/*
 * Charger CAN controller
 */
#define CAN_CTRLR_SPI_SCK_GPIO_Port     GPIOB
#define CAN_CTRLR_SPI_SCK_Pin           GPIO_PIN_13

#define CAN_CTRLR_SPI_MISO_GPIO_Port    GPIOB
#define CAN_CTRLR_SPI_MISO_Pin          GPIO_PIN_14

#define CAN_CTRLR_SPI_MOSI_GPIO_Port    GPIOB
#define CAN_CTRLR_SPI_MOSI_Pin          GPIO_PIN_15

#define CAN_CTRLR_RST_GPIO_Port         GPIOB
#define CAN_CTRLR_RST_Pin               GPIO_PIN_11

#define CAN_CTRLR_SPI_CS_GPIO_Port      GPIOB
#define CAN_CTRLR_SPI_CS_Pin            GPIO_PIN_12

#define CHARGER_STBY_GPIO_Port          GPIOA
#define CHARGER_STBY_Pin                GPIO_PIN_8

/*
 * LCD SPI pins
 */
#define LCD_SPI_CS_GPIO_Port            GPIOA
#define LCD_SPI_CS_Pin                  GPIO_PIN_15

#define LCD_SPI_SCK_GPIO_Port           GPIOB
#define LCD_SPI_SCK_Pin                 GPIO_PIN_3

#define LCD_SPI_MISO_GPIO_Port          GPIOB
#define LCD_SPI_MISO_Pin                GPIO_PIN_4

#define LCD_SPI_MOSI_GPIO_Port          GPIOB
#define LCD_SPI_MOSI_Pin                GPIO_PIN_5

#define LCD_RST_GPIO_Port               GPIOB
#define LCD_RST_Pin                     GPIO_PIN_6

#define LCD_BLK_GPIO_Port               GPIOB
#define LCD_BLK_Pin                     GPIO_PIN_7

typedef enum {
    HARDWARE_FAULT = 0,
    TEMP_PROTECTION,
    INPUT_VOLTAGE,
    STARTING_STATE,
    COMMS_STATE
} charger_check_t;

typedef struct {
    uint8_t rx_msg[5];
    bool checks[5];
    float charging_voltage;
    float charging_current;
    bool valid;
} charger_msg_t;

extern TIM_HandleTypeDef htim2;

/*
extern oem_spi_config_t lcd_spi;
extern oem_spi_config_t charger_spi;
*/

extern MCP25625 charger_CAN_converter;

extern charger_msg_t chgMsg;
extern volatile bool send_can;

void SystemClockConfig(void);
void GpioInit(void);
void TimerInit(void);
void SysTick_Handler(void);
void TIM2_IRQHandler(void);

void charger_can_init(void);
void lcd_spi_init(void);

void spi_send_charger(void);
void spi_receive_charger(void);

void lcd_write_command(uint8_t command);
void lcd_write_data8(uint8_t data);
void lcd_write_data_buffer(const uint8_t *data, uint16_t size);
void lcd_reset(void);
void lcd_init(void);
void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void lcd_fill_screen(uint16_t color);
void lcd_show_status(void);