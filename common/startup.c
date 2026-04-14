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
WEAK_ALIAS(TIM2_IRQHandler);
WEAK_ALIAS(TIM1_BRK_TIM15_IRQHandler);
WEAK_ALIAS(TIM1_UP_TIM16_IRQHandler);

// Vector Table
__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,                    // -15
    NMI_Handler,                      // -14
    HardFault_Handler,                // -13
    MemManage_Handler,                // -12
    BusFault_Handler,                 // -11
    UsageFault_Handler,               // -10
    0, 0, 0, 0,                       // -9 to -6 reserved
    SVC_Handler,                      // -5
    DebugMon_Handler,                 // -4
    0,                                // -3 reserved
    PendSV_Handler,                   // -2
    SysTick_Handler,                  // -1

    // External interrupts (IRQ0+)
    0, 0,                             // IRQ0: WWDG, IRQ1: PVD
    0, 0, 0, 0, 0,                    // IRQ2-6
    0, 0, 0, 0, 0,                    // IRQ7-11
    0, 0, 0, 0,                       // IRQ12-15
    0, 0, 0, 0, 0,                    // IRQ16-20
    0, 0, 0, 0,                       // IRQ21-24
    TIM1_BRK_TIM15_IRQHandler,        // IRQ25 - FIRES ON ANY TIM15 ANY EVENT OR TIM1 BREAK
    TIM1_UP_TIM16_IRQHandler,         // IRQ26 - FIRES ON ANY TIM16 ANY EVENT OR TIM1 BREAK
    0, 0,                             // IRQ27-28
    TIM2_IRQHandler,                  // IRQ29
};

void ErrorHandler(void) { // this will only happen if error on board
  __disable_irq();
  while (1) {
  }
}

void SystemClockConfig(void) { // if you are messing with this please consult someone
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    ErrorHandler();
  }
}