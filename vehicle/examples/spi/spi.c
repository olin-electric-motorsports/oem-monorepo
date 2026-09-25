#include "stm32g4xx_hal.h"
#include "common/spi/spi.h"
#include "spi.h"                 
#include "vehicle/common/adbms1818/ADBMS1818.h" 


extern oem_spi_config_t bms_spi;

#define TOTAL_IC 1
cell_asic bms_ic[TOTAL_IC];


int main(void) {
    HAL_Init();
    SystemClockConfig();
    SystemCoreClockUpdate();
    oem_spi_init(&bms_spi); 


        
    ADBMS1818_init_cfg(TOTAL_IC, bms_ic);
    ADBMS1818_init_reg_limits(TOTAL_IC, bms_ic);

    // Turn the internal voltage reference ON
    ADBMS1818_set_cfgr_refon(0, bms_ic, true); 

    // Wake the daisy chain and push the configuration to the hardware
    wakeup_sleep(TOTAL_IC);
    ADBMS1818_wrcfg(TOTAL_IC, bms_ic);
    ADBMS1818_wrcfgb(TOTAL_IC, bms_ic);

    //volatile float cell_1_voltage = 0.0f;
    // uint8_t test_tx[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    // uint8_t test_rx[4] = {0x00, 0x00, 0x00, 0x00};

    while (1) {

        
    
        wakeup_sleep(TOTAL_IC);

        // 2. Command the chip to start measuring all 18 cells
        ADBMS1818_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);

        // 3. THE CRITICAL FIX: Force the STM32 to wait for the ADC to finish. 
        // 3ms is required, we use 5ms to be perfectly safe.
        // (Make sure pollAdc() is commented out/deleted!)
        HAL_Delay(5); 

        // 4. Wake the SPI interface back up (10us delay is inside this function)
        wakeup_idle(TOTAL_IC);

        // 5. Ask the chip to send the measured data back to the STM32
        int8_t pec_error = ADBMS1818_rdcv(0, TOTAL_IC, bms_ic);

        if (pec_error == 0) {
            volatile uint16_t cell_1 = bms_ic[0].cells.c_codes[0];
            volatile uint16_t cell_18 = bms_ic[0].cells.c_codes[17];
            
            (void)cell_1; 
            (void)cell_18;
        }

        // Wait before taking the next measurement
        HAL_Delay(100);
    }
    
    return 0;
}