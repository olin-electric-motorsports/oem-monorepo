#include "charger.h"

#include <string.h>

charger_msg_t chgMsg = {0};
volatile bool send_can = false;
/*
static uint8_t ten_ms_counter = 0;
*/
/*
static uint8_t lcd_counter = 0;
*/
/*
static uint8_t charger_timeout = 0;
*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == CHARGER_MAIN_TIM_INSTANCE) {
        send_can = true;
    }
}

void charger_can_init(void) {
    oem_spi_init(&charger_spi);
    /*
    MCP25625_init(&charger_CAN_converter);
    */
}
/*
void lcd_spi_init(void) {
    oem_spi_init(&lcd_spi);
}
*/
/*
void lcd_write_command(uint8_t command) {
    
     * NOTE:
     * This assumes your LCD write path is valid for your board wiring.
     * If the LCD truly has no D/C line routed, this part will need a protocol change.
     
    oem_spi_select(&lcd_spi);
    uint8_t rx_dummy = 0;
    oem_spi_transmit_receive(&lcd_spi, &command, &rx_dummy, 1);
    oem_spi_deselect(&lcd_spi);
}
*/

/*
void lcd_write_data8(uint8_t data) {
    oem_spi_select(&lcd_spi);
    uint8_t rx_dummy = 0;
    oem_spi_transmit_receive(&lcd_spi, &data, &rx_dummy, 1);
    oem_spi_deselect(&lcd_spi);
}
*/
/*
void lcd_write_data_buffer(const uint8_t *data, uint16_t size) {
    oem_spi_select(&lcd_spi);
    uint8_t rx_dummy = 0;
    for (uint16_t i = 0; i < size; i++) {
        uint8_t tx = data[i];
        oem_spi_transmit_receive(&lcd_spi, &tx, &rx_dummy, 1);
    }
    oem_spi_deselect(&lcd_spi);
}
*/
/*
void lcd_reset(void) {
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
}
*/
/*
void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    lcd_write_command(LCD_CMD_CASET);
    data[0] = (uint8_t)(x0 >> 8);
    data[1] = (uint8_t)(x0 & 0xFF);
    data[2] = (uint8_t)(x1 >> 8);
    data[3] = (uint8_t)(x1 & 0xFF);
    lcd_write_data_buffer(data, 4);

    lcd_write_command(LCD_CMD_RASET);
    data[0] = (uint8_t)(y0 >> 8);
    data[1] = (uint8_t)(y0 & 0xFF);
    data[2] = (uint8_t)(y1 >> 8);
    data[3] = (uint8_t)(y1 & 0xFF);
    lcd_write_data_buffer(data, 4);

    lcd_write_command(LCD_CMD_RAMWR);
}
*/
/*
void lcd_fill_screen(uint16_t color) {
    uint8_t pixel[2];
    uint32_t count = LCD_WIDTH * LCD_HEIGHT;

    pixel[0] = (uint8_t)(color >> 8);
    pixel[1] = (uint8_t)(color & 0xFF);

    lcd_set_addr_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    oem_spi_select(&lcd_spi);
    while (count--) {
        oem_spi_transmit(&lcd_spi, pixel, 2);
    }
    oem_spi_deselect(&lcd_spi);
}
*/

/*
void lcd_init(void) {
    lcd_reset();

    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET);

    lcd_write_command(LCD_CMD_SLPOUT);
    HAL_Delay(100);

    lcd_write_command(LCD_CMD_MADCTL);
    lcd_write_data8(0x80);

    lcd_write_command(LCD_CMD_COLMOD);
    lcd_write_data8(0x55);

    lcd_write_command(LCD_CMD_INVON);

    lcd_write_command(LCD_CMD_PORCTRL);
    lcd_write_data8(0x0C);
    lcd_write_data8(0x0C);
    lcd_write_data8(0x00);
    lcd_write_data8(0x33);
    lcd_write_data8(0x33);

    lcd_write_command(LCD_CMD_GCTRL);
    lcd_write_data8(0x35);

    lcd_write_command(LCD_CMD_VCOMS);
    lcd_write_data8(0x2B);

    lcd_write_command(LCD_CMD_LCMCTRL);
    lcd_write_data8(0x2C);

    lcd_write_command(LCD_CMD_VDVVRHEN);
    lcd_write_data8(0x01);
    lcd_write_data8(0xFF);

    lcd_write_command(LCD_CMD_VRHS);
    lcd_write_data8(0x11);

    lcd_write_command(LCD_CMD_VDVS);
    lcd_write_data8(0x20);

    lcd_write_command(LCD_CMD_FRCTRL2);
    lcd_write_data8(0x0F);

    lcd_write_command(LCD_CMD_PWCTRL1);
    lcd_write_data8(0xA4);
    lcd_write_data8(0xA1);

    lcd_write_command(LCD_CMD_DISPON);
    HAL_Delay(20);

    lcd_fill_screen(LCD_COLOR_BLACK);
}
*/
/*
void lcd_show_status(void) {
    if (charger_timeout >= 100 ||
        charging_fbk.hardware_fault ||
        charging_fbk.temperature_protection ||
        charging_fbk.input_voltage ||
        charging_fbk.starting_state ||
        charging_fbk.communication_state) {
        lcd_fill_screen(LCD_COLOR_RED);
        return;
    }

    if (bms_core.pack_voltage >= (TARGET_PACK_VOLTAGE_V - 5.0f)) {
        lcd_fill_screen(LCD_COLOR_YELLOW);
        return;
    }

    if (charging_cmd.enable) {
        lcd_fill_screen(LCD_COLOR_GREEN);
    } else {
        lcd_fill_screen(LCD_COLOR_BLUE);
    }
}
*/

/*
void spi_send_charger(void) {
    uint8_t bytes[8] = {0};

    HAL_GPIO_TogglePin(SPI_STATUS_GPIO_Port, SPI_STATUS_Pin);

    bytes[0] = (uint8_t)(((uint16_t)(charging_cmd.max_voltage * 10.0f)) >> 8);
    bytes[1] = (uint8_t)((uint16_t)(charging_cmd.max_voltage * 10.0f));
    bytes[2] = (uint8_t)(((uint16_t)(charging_cmd.max_current * 6.0f)) >> 8);
    bytes[3] = (uint8_t)((uint16_t)(charging_cmd.max_current * 6.0f));
    bytes[4] = (uint8_t)(charging_cmd.enable);

    mcp25625_send_message(0x1806E5F4, 8, bytes, true);
}
*/

void spi_receive_charger(void) {
    chgMsg.valid = false;
}

int main(void) {
    HAL_Init();
    SystemClockConfig();
    GpioInit();
    TimerInit();
/*
    lcd_spi_init();
*/    
    charger_can_init();
/*    
    lcd_init();
*/
    /*

    can_init_charger();

    */

    HAL_Delay(20);
    
    /*
    can_receive_charging_cmd();
    can_receive_bms_core();
    */

    while (1) {
        if (!send_can) {
            continue;
        }

        send_can = false;

        HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
/*
        if (can_poll_receive_bms_core() == 0) {
            HAL_GPIO_WritePin(CAN_STATUS_GPIO_Port, CAN_STATUS_Pin, GPIO_PIN_SET);
            can_receive_bms_core();
        } else {
            HAL_GPIO_WritePin(CAN_STATUS_GPIO_Port, CAN_STATUS_Pin, GPIO_PIN_RESET);
        }

        if (can_poll_receive_charging_cmd() == 0) {
            can_receive_charging_cmd();
            charger_timeout = 0;
            HAL_GPIO_WritePin(CHARGING_STATUS_GPIO_Port, CHARGING_STATUS_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(CHARGING_STATUS_GPIO_Port, CHARGING_STATUS_Pin, GPIO_PIN_RESET);

            if (charger_timeout < 100) {
                charger_timeout++;
            } else {
                charging_cmd.max_voltage = 0;
                charging_cmd.max_current = 0;
                charging_cmd.enable = 0;
            }
        }
*/

        spi_receive_charger();
/*
        if (!chgMsg.valid) {
            charging_fbk.charging_voltage = 0;
            charging_fbk.charging_current = 0;
            charging_fbk.hardware_fault = 0;
            charging_fbk.temperature_protection = 0;
            charging_fbk.input_voltage = 0;
            charging_fbk.starting_state = 0;
            charging_fbk.communication_state = 0;
        }

        can_send_charging_fbk();

        if (charging_fbk.charging_voltage >= TARGET_PACK_VOLTAGE_V) {
            HAL_GPIO_WritePin(FULLY_CHARGED_STATUS_GPIO_Port, FULLY_CHARGED_STATUS_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(FULLY_CHARGED_STATUS_GPIO_Port, FULLY_CHARGED_STATUS_Pin, GPIO_PIN_RESET);
        }
*/
/*
        ten_ms_counter++;
        if (ten_ms_counter >= 10) {
            spi_send_charger();
            ten_ms_counter = 0;
        }
*/
/*
        lcd_counter++;
        if (lcd_counter >= 25) {
            lcd_show_status();
            lcd_counter = 0;
        }
*/
    }

    return 0;
}