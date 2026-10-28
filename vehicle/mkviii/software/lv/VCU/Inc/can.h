#ifndef VCU_CAN_H
#define VCU_CAN_H

#include <stdbool.h>
#include <stdint.h>

/* Startup, expired input, and fault recovery require a fresh release first. */
void vcu_dashboard_reset(void);
void vcu_receive_dashboard(void);
bool vcu_dashboard_start_button(uint32_t now_ms, bool blocking_fault);

#endif /* VCU_CAN_H */
