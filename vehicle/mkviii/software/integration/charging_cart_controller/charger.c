#include "charger.h"
#include "charger_config.h"

#include <string.h>

charger_msg_t chgMsg = {0};

volatile bool send_can = false;
volatile int flag = 0;
volatile bool display_target_voltage = true;
volatile bool display_target_current = false;
volatile bool display_text_debug = false;
volatile bool display_text = true;

static uint8_t ten_ms_counter = 0;
static uint8_t charger_timeout = 0;

void timer0_isr(void) {
    send_can = true;
}

void timer2_isr(void) {
    flag = 1;

    if (display_target_voltage) {
        if (display_text) {
            display_text = false;
        } else {
            display_text = true;
            display_target_voltage = false;
            display_target_current = true;
        }
    } else if (display_target_current) {
        if (display_text) {
            if (display_text_debug) {
                display_text = false;
            }
            display_text_debug = true;
        } else {
            display_text_debug = false;
            display_text = true;
            display_target_current = false;
            display_target_voltage = true;
        }
    }
}

void charger_can_init(void) {
    oem_spi_init(&charger_spi);
    MCP25625_init(&charger_CAN_converter);
}

void spi_bus_init(void) {
    oem_spi_init(&display_spi);
}

void max7221_write(uint8_t address, uint8_t data) {
    oem_spi_select(&display_spi);
    oem_spi_transmit(&display_spi, &address, LENGTH_ADDRESS_SPI);
    oem_spi_transmit(&display_spi, &data, LENGTH_DATA_SPI);
    oem_spi_deselect(&display_spi);
}

void max7221_init(void) {
    HAL_GPIO_WritePin(MAX7221_CS_PORT, MAX7221_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(MAX7221_CS_PORT, MAX7221_CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(MAX7221_CS_PORT, MAX7221_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(20);

    max7221_write(DISPLAY_TEST, DISPLAY_TEST_ON);
    HAL_Delay(500);
    max7221_write(DISPLAY_TEST, 0x00);

    max7221_write(SHUTDOWN, SHUTDOWN_OFF);
    max7221_write(DECODE, DECODE_4_DIGITS);
    max7221_write(SCAN_LIMIT, SCAN_4_DIGITS);
    max7221_write(INTENSITY, SET_MIN_BRIGHTNESS);
}

void display_voltage_on_seven_segment(float voltage) {
    uint16_t voltage_int = (uint16_t)(voltage);

    uint8_t ones = voltage_int % 10;
    uint8_t tens = (voltage_int / 10) % 10;
    uint8_t hundreds = (voltage_int / 100) % 10;
    uint8_t thousands = (voltage_int / 1000) % 10;

    max7221_write(0x01, thousands);
    max7221_write(0x02, hundreds);
    max7221_write(0x03, tens);
    max7221_write(0x04, ones);
}

void display_pack_on_seven_segment(void) {
    max7221_write(0x01, 15);
    max7221_write(0x02, 15);
    max7221_write(0x03, 14);
    max7221_write(0x04, 10);
}

void display_volt_on_seven_segment(void) {
    max7221_write(0x01, 15);
    max7221_write(0x02, 15);
    max7221_write(0x03, 13);
    max7221_write(0x04, 10);
}

void spi_send_charger(void) {
    uint8_t bytes[8] = {0};

    HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN);

    bytes[0] = (uint8_t)(((uint16_t)(charging_cmd.max_voltage * 10.0f)) >> 8);
    bytes[1] = (uint8_t)((uint16_t)(charging_cmd.max_voltage * 10.0f));
    bytes[2] = (uint8_t)(((uint16_t)(charging_cmd.max_current * 6.0f)) >> 8);
    bytes[3] = (uint8_t)((uint16_t)(charging_cmd.max_current * 6.0f));
    bytes[4] = (uint8_t)(charging_cmd.enable);

    mcp25625_send_message(0x1806E5F4, 8, bytes, true);
}

void spi_receive_charger(void) {
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        timer0_isr();
    }

    if (htim->Instance == TIM15) {
        timer2_isr();
    }
}

int main(void) {
    HAL_Init();
    SystemClockConfig();
    GpioInit();
    TimerInit();

    spi_bus_init();
    max7221_init();

    charger_can_init();

    can_init_charger();

    HAL_Delay(20);

    can_receive_charging_cmd();
    can_receive_bms_core();

    while (1) {
        if (display_target_voltage) {
            if (display_text) {
                display_pack_on_seven_segment();
            } else {
                display_voltage_on_seven_segment(bms_core.pack_voltage);
            }
        } else if (display_target_current) {
            if (display_text) {
                display_volt_on_seven_segment();
            } else {
                display_voltage_on_seven_segment(TARGET_PACK_VOLTAGE);
            }
        }

        if (send_can) {
            if (can_poll_receive_bms_core() == 0) {
                HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
                can_receive_bms_core();
            }

            if (can_poll_receive_charging_cmd() == 0) {
                can_receive_charging_cmd();
                charger_timeout = 0;
            } else {
                if (charger_timeout < 100) {
                    charger_timeout++;
                } else {
                    charging_cmd.max_voltage = 0;
                    charging_cmd.max_current = 0;
                    charging_cmd.enable = 0;
                }
            }

            spi_receive_charger();

            charging_fbk.charging_voltage = 0;
            charging_fbk.charging_current = 0;
            charging_fbk.hardware_fault = 0;
            charging_fbk.temperature_protection = 0;
            charging_fbk.input_voltage = 0;
            charging_fbk.starting_state = 0;
            charging_fbk.communication_state = 0;

            can_send_charging_fbk();

            ten_ms_counter++;
            if (ten_ms_counter == 100) {
                spi_send_charger();
                ten_ms_counter = 0;
            }

            send_can = false;
        }
    }

    return 0;
}