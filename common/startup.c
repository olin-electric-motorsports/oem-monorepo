#include <stdint.h>
#include "stm32g4xx.h"

// Linker script symbols
extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

// Initialization functions
extern void SystemInit(void); // Defined in system_stm32g4xx.c
extern int main(void);


void Reset_Handler(void) {
  uint32_t *src, *dst;

  // Copy .data from FLASH to RAM
  src = &_sidata;
  dst = &_sdata;
  while (dst < &_edata) {
    *dst++ = *src++;
  }

  // Zero out .bss
  dst = &_sbss;
  while (dst < &_ebss) {
    *dst++ = 0;
  }

  // Setup Clock and FPU
  SystemInit();

  // Enter main (Removed __libc_init_array since pure C)
  main();

  while (1);
}

void Default_Handler(void) {
  while (1);
}

// Weak aliases
#define WEAK_ALIAS(x) __attribute__((weak, alias("Default_Handler"))) void x(void)

WEAK_ALIAS(NMI_Handler);
WEAK_ALIAS(HardFault_Handler);
WEAK_ALIAS(MemManage_Handler);
WEAK_ALIAS(BusFault_Handler);
WEAK_ALIAS(UsageFault_Handler);
WEAK_ALIAS(SVC_Handler);
WEAK_ALIAS(DebugMon_Handler);
WEAK_ALIAS(PendSV_Handler);
WEAK_ALIAS(SysTick_Handler);

// Vector Table
__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};