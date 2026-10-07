#ifndef AIR_CONFIG_H
#define AIR_CONFIG_H

/* Review these application values against the actual accumulator and relays. */
#ifndef AIR_CONTROLLED_NEGATIVE
#define AIR_CONTROLLED_NEGATIVE 1
#endif
#define AIR_PACK_MIN_MV 200000u
#define AIR_TRACTIVE_SAFE_MV 5000u
#define AIR_PRECHARGE_PERCENT 95u
#define AIR_PRECHARGE_TIMEOUT_MS 5000u
#define AIR_CONTACTOR_CLOSE_MS 200u
#define AIR_CONTACTOR_OPEN_MS 100u
#define AIR_DISCHARGE_TIMEOUT_MS 10000u
#define AIR_IMD_SETTLE_MS 4000u
#define AIR_STARTUP_CAN_WAIT_MS 1000u
#define AIR_BMS_TIMEOUT_MS 1000u
#define AIR_IVT_TIMEOUT_MS 500u
#define AIR_STATUS_PERIOD_MS 63u
#define AIR_HEARTBEAT_PERIOD_MS 500u

/* Legacy wire protocol. Update these AND air.yml/legacy_bms.yml together. */
#define AIR_CAN_STATUS_ID 0x00du
#define AIR_CAN_BMS_ID 0x010u
#define AIR_CAN_IVT_ID 0x414u
#define AIR_CAN_STATUS_BYTES 4u
#define AIR_CAN_BMS_BYTES 7u
#define AIR_CAN_IVT_BYTES 6u

#endif
