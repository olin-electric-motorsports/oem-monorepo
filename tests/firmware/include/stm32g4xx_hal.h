#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET=0, GPIO_PIN_SET=1 } GPIO_PinState;
typedef struct { int id; } GPIO_TypeDef;
typedef struct { int id; } ADC_TypeDef;
extern GPIO_TypeDef mock_gpio[3];
extern ADC_TypeDef mock_adc[2];
#define GPIOA (&mock_gpio[0])
#define GPIOB (&mock_gpio[1])
#define GPIOF (&mock_gpio[2])
#define ADC1 (&mock_adc[0])
#define ADC2 (&mock_adc[1])
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_ANALOG 3
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
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
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_2 2
#define ADC_CHANNEL_3 3
#define ADC_CHANNEL_4 4
#define ADC_CHANNEL_17 17

#define ADC_SAMPLETIME_47CYCLES_5 47
#define ADC_CLOCK_SYNC_PCLK_DIV4 4
#define ADC_RESOLUTION_12B 12
#define ADC_DATAALIGN_RIGHT 0
#define ADC_EOC_SINGLE_CONV 0
#define ADC_SINGLE_ENDED 0
#define ADC_REGULAR_RANK_1 1
#define DISABLE 0
#define FDCAN1 ((void*)0x40006400)
#define FDCAN_CLOCK_DIV1 1
#define FDCAN_FRAME_CLASSIC 0
#define FDCAN_MODE_NORMAL 0
#define FDCAN_TX_FIFO_OPERATION 0
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_DLC_BYTES_8 8
#define FDCAN_DLC_BYTES_7 7
#define FDCAN_DLC_BYTES_5 5
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_NO_TX_EVENTS 0
#define FDCAN_RX_FIFO0 0
#define __HAL_RCC_ADC12_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() ((void)0)
#define GPIOC ((GPIO_TypeDef*)0)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
static inline void __disable_irq(void) {}
typedef struct { uint32_t Pin,Mode,Pull,Speed,Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t ClockPrescaler,Resolution,DataAlign,EOCSelection; } MockAdcInit;
typedef struct { ADC_TypeDef *Instance; MockAdcInit Init; uint32_t selected_channel; } ADC_HandleTypeDef;
typedef struct { uint32_t Channel,Rank,SamplingTime,SingleDiff; } ADC_ChannelConfTypeDef;
typedef struct { uint32_t ClockDivider,FrameFormat,AutoRetransmission,TransmitPause,ProtocolException,NominalPrescaler,NominalSyncJumpWidth,NominalTimeSeg1,NominalTimeSeg2,DataPrescaler,DataSyncJumpWidth,DataTimeSeg1,DataTimeSeg2,Mode,StdFiltersNbr,ExtFiltersNbr,TxFifoQueueMode; } MockFdcanInit;
typedef struct { void *Instance; MockFdcanInit Init; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker; } FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t Identifier,DataLength; } FDCAN_RxHeaderTypeDef;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*,uint16_t);
void HAL_GPIO_WritePin(GPIO_TypeDef*,uint16_t,GPIO_PinState);
void HAL_GPIO_Init(GPIO_TypeDef*,GPIO_InitTypeDef*);
void HAL_IncTick(void);
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*,ADC_ChannelConfTypeDef*);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*,uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*,uint32_t,FDCAN_RxHeaderTypeDef*,uint8_t*);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*,FDCAN_TxHeaderTypeDef*,uint8_t*);

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*);
