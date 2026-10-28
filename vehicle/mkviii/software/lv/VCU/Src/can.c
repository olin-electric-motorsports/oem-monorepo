/* VCU CAN hardware setup and legacy dashboard reception. */
#include "can.h"
#include "can_api.h"
#include "vcu_config.h"
#include "stm32g4xx_hal.h"

/* Board-specific setup called by HAL_FDCAN_Init() before CAN is configured. */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan) {
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    /* The VCU's 16 MHz HSE gives 500 kbit/s with the generated 32-TQ timing. */
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_HSE);
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA11 is CAN RX and PA12 is CAN TX, connected to the onboard transceiver. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

extern FDCAN_HandleTypeDef hfdcan1;

static bool s_valid;
static bool s_pressed;
static bool s_release_seen;
static uint32_t s_last_rx_ms;

void vcu_dashboard_reset(void) {
    s_valid = false;
    s_pressed = false;
    s_release_seen = false;
    s_last_rx_ms = 0u;
}

static void expire_input(uint32_t now_ms) {
    if (s_valid && (uint32_t)(now_ms - s_last_rx_ms) >=
                       VCU_DASHBOARD_TIMEOUT_MS) {
        vcu_dashboard_reset();
    }
}

void vcu_receive_dashboard(void) {
    expire_input(HAL_GetTick());

    /* Bound each visit to the queue depth observed on entry. */
    uint32_t remaining = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    while (remaining > 0u) {
        --remaining;
        FDCAN_RxHeaderTypeDef header = {0};
        /* HAL copies the payload before we can reject an unexpected FD frame. */
        uint8_t data[64] = {0};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
                                  &header, data) != HAL_OK) {
            vcu_dashboard_reset();
            return;
        }
        if (header.IdType != FDCAN_STANDARD_ID ||
            header.Identifier != CAN_TOOLS_DASHBOARD_FRAME_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN ||
            header.BitRateSwitch != FDCAN_BRS_OFF ||
            header.DataLength != FDCAN_DLC_BYTES_1) {
            continue;
        }

        struct can_tools_dashboard_t decoded;
        if (can_tools_dashboard_unpack(&decoded, data,
                                       CAN_TOOLS_DASHBOARD_LENGTH) != 0) {
            continue;
        }

        uint32_t now_ms = HAL_GetTick();
        expire_input(now_ms);
        s_valid = true;
        s_last_rx_ms = now_ms;
        s_pressed = decoded.start_button_state != 0u;
        if (!s_pressed) {
            s_release_seen = true;
        }
    }
}

bool vcu_dashboard_start_button(uint32_t now_ms, bool blocking_fault) {
    expire_input(now_ms);
    if (blocking_fault) {
        /* A release received after fault recovery is required before rearming. */
        s_release_seen = false;
        return false;
    }
    /* Timeout invalidates the button only; the existing RTD latch is unchanged. */
    return s_valid && s_release_seen && s_pressed;
}
