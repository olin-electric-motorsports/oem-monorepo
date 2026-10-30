#ifndef AIR_CONFIG_H
#define AIR_CONFIG_H

/* settings. Check these against the real accumulator and relays */

/* 1: this board switches AIR- and the shutdown loop powers AIR+.
 * 0: the older setup, the other way round */
#ifndef AIR_CONTROLLED_NEGATIVE
#define AIR_CONTROLLED_NEGATIVE 1
#endif
#define AIR_PACK_MIN_MV 200000u /* a pack under 200 V is a BMS fault */
/* Under 5 V counts as discharged (needed to reach IDLE); IDLE faults at 6 V
 * or more. The 1 V gap keeps measurement noise from causing a fault */
#define AIR_TRACTIVE_SAFE_MV 5000u
#define AIR_TRACTIVE_CHARGED_MV 6000u
#define AIR_PRECHARGE_PERCENT 95u      /* precharge is done at 95% of pack voltage */
#define AIR_PRECHARGE_TIMEOUT_MS 5000u /* and must get there within 5 s */
#define AIR_CONTACTOR_CLOSE_MS 200u    /* AIRs get 200 ms to close */
#define AIR_CONTACTOR_OPEN_MS 100u     /* and 100 ms to open */
/* Shutdown readings shorter than this are treated as noise: a node open while
 * closing, or in IDLE, TSMS closed while an earlier node is open */
#define AIR_SHUTDOWN_DEBOUNCE_MS 3u
/* FSAE 2026 EV.7.2.2.c: 60 V or less within 5 s of the shutdown circuit opening. */
#define AIR_DISCHARGE_LIMIT_MV 60000u
#define AIR_DISCHARGE_LIMIT_MS 5000u
#define AIR_DISCHARGE_TIMEOUT_MS 10000u /* and under 5 V within 10 s */
#define AIR_IMD_SETTLE_MS 4000u       /* the IMD gets 4 s to settle at power-up */
#define AIR_STARTUP_CAN_WAIT_MS 1000u /* then up to 1 s more for the first CAN messages */
#define AIR_BMS_TIMEOUT_MS 1000u      /* BMS data older than this is a fault */
#define AIR_IVT_TIMEOUT_MS 500u       /* IVT data older than this is a fault */
#define AIR_STATUS_PERIOD_MS 63u      /* how often our status goes out on CAN */
#define AIR_HEARTBEAT_PERIOD_MS 500u  /* the heartbeat LED toggles this often */

/* Legacy wire protocol. Update these AND air.yml/legacy_bms.yml together. */
#define AIR_CAN_STATUS_ID 0x00du /* our status message */
#define AIR_CAN_BMS_ID 0x010u    /* BMS pack data */
#define AIR_CAN_IVT_ID 0x414u    /* IVT voltage result */
#define AIR_CAN_STATUS_BYTES 4u  /* message lengths */
#define AIR_CAN_BMS_BYTES 7u
#define AIR_CAN_IVT_BYTES 6u

#endif
