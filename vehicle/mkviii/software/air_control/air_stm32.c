#include "air_board.h"
#include "air_board_config.h"
#include "air_protocol.h"

static FDCAN_HandleTypeDef can;
static IWDG_HandleTypeDef watchdog;

void SysTick_Handler(void) { HAL_IncTick(); }

static void stop(void) {
    air_board_safe();
    __disable_irq();
    for (;;) { /* If started, IWDG resets us; no automatic re-arm with TSMS closed. */ }
}
void HardFault_Handler(void) { stop(); }
void MemManage_Handler(void) { stop(); }
void BusFault_Handler(void) { stop(); }
void UsageFault_Handler(void) { stop(); }
void NMI_Handler(void) { stop(); }

static bool clock_init(void) {
    /* Avoid assuming an external crystal exists on the as-yet-unassigned PCB. */
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef periph = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.LSIState = RCC_LSI_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) return false;
    periph.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    periph.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    return HAL_RCCEx_PeriphCLKConfig(&periph) == HAL_OK;
}

static bool can_init(void) {
    __HAL_RCC_FDCAN_CLK_ENABLE();
    can.Instance = FDCAN1;
    can.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    can.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    can.Init.Mode = FDCAN_MODE_NORMAL;
    can.Init.AutoRetransmission = DISABLE;
    can.Init.TransmitPause = DISABLE;
    can.Init.ProtocolException = DISABLE;
    can.Init.NominalPrescaler = AIR_CAN_PRESCALER;
    can.Init.NominalSyncJumpWidth = AIR_CAN_SJW;
    can.Init.NominalTimeSeg1 = AIR_CAN_SEG1;
    can.Init.NominalTimeSeg2 = AIR_CAN_SEG2;
    can.Init.DataPrescaler = 1;
    can.Init.DataSyncJumpWidth = 1;
    can.Init.DataTimeSeg1 = 1;
    can.Init.DataTimeSeg2 = 1;
    can.Init.StdFiltersNbr = 1;
    can.Init.ExtFiltersNbr = 0;
    can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&can) != HAL_OK) return false;
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = AIR_CAN_BMS_ID;
    filter.FilterID2 = AIR_CAN_IVT_ID;
    if (HAL_FDCAN_ConfigFilter(&can, &filter) != HAL_OK) return false;
    if (HAL_FDCAN_ConfigGlobalFilter(&can, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) return false;
    return HAL_FDCAN_Start(&can) == HAL_OK;
}

static bool can_receive(air_measurements_t *data, uint32_t now) {
    FDCAN_ProtocolStatusTypeDef status;
    if (HAL_FDCAN_GetProtocolStatus(&can, &status) != HAL_OK || status.BusOff) return false;
    if (__HAL_FDCAN_GET_FLAG(&can, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST)) return false;
    /* G4 FIFO has three entries; bound work even if traffic arrives continuously. */
    for (unsigned i = 0; i < 3 && HAL_FDCAN_GetRxFifoFillLevel(&can, FDCAN_RX_FIFO0); ++i) {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t bytes[64]; /* HAL copies the DLC length, including unexpected FD frames. */
        if (HAL_FDCAN_GetRxMessage(&can, FDCAN_RX_FIFO0, &rx, bytes) != HAL_OK) return false;
        if (rx.IdType != FDCAN_STANDARD_ID || rx.RxFrameType != FDCAN_DATA_FRAME ||
            rx.FDFormat != FDCAN_CLASSIC_CAN) continue;
        size_t length;
        if (rx.DataLength == FDCAN_DLC_BYTES_7) length = 7;
        else if (rx.DataLength == FDCAN_DLC_BYTES_6) length = 6;
        else continue;
        (void)air_decode(data, rx.Identifier, bytes, length, now);
    }
    return true;
}

static bool can_send(const air_t *air, const air_inputs_t *in) {
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t bytes[AIR_CAN_STATUS_BYTES];
    air_encode_status(bytes, air, in);
    tx.Identifier = AIR_CAN_STATUS_ID;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_4;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&can, &tx, bytes) == HAL_OK;
}

int main(void) {
    HAL_Init();
    if (!clock_init()) stop();
    air_t air;
    air_measurements_t data = {0};
    air_init(&air, HAL_GetTick());
    bool board_ok = air_board_init();
    bool can_ok = board_ok && can_init();
    if (!board_ok) air_trip(&air, AIR_FAULT_BOARD_CONFIG);
    else if (!can_ok) air_trip(&air, AIR_FAULT_CAN_ERROR);
    air_board_safe();

    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_32;
    watchdog.Init.Reload = 249; /* Nominal 250 ms with 32 kHz LSI. */
    watchdog.Init.Window = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) stop();

    uint32_t last_step = HAL_GetTick();
    uint32_t last_status = last_step;
    for (;;) {
        uint32_t now = HAL_GetTick();
        if (now == last_step) continue;
        last_step = now;
        air_inputs_t in = air_board_read();
        if (can_ok && !can_receive(&data, now)) air_trip(&air, AIR_FAULT_CAN_ERROR);
        air_step(&air, &in, &data, now);
        air_board_write(&air, now);
        if (can_ok && (uint32_t)(now - last_status) >= AIR_STATUS_PERIOD_MS) {
            last_status = now;
            if (!can_send(&air, &in)) {
                air_trip(&air, AIR_FAULT_CAN_ERROR);
                air_board_safe();
            }
        }
        /* A healthy completed loop, including a latched fault, services IWDG. */
        if (HAL_IWDG_Refresh(&watchdog) != HAL_OK) stop();
    }
}
