#include "stm32g4xx_hal.h"
#include "common/spi/spi.h"                 
#include "vehicle/common/adbms1818/ADBMS1818.h" 


extern oem_spi_config_t bms_spi;

#define TOTAL_IC 1
cell_asic bms_ic[TOTAL_IC];


int main(void) {
    HAL_Init();
    //SystemClockConfig();
    oem_spi_init(&bms_spi); 

        
    ADBMS1818_init_cfg(TOTAL_IC, bms_ic);
    ADBMS1818_init_reg_limits(TOTAL_IC, bms_ic);

    // Turn the internal voltage reference ON
    ADBMS1818_set_cfgr_refon(0, bms_ic, true); 

    // Wake the daisy chain and push the configuration to the hardware
    wakeup_sleep(TOTAL_IC);
    ADBMS1818_wrcfg(TOTAL_IC, bms_ic);
    ADBMS1818_wrcfgb(TOTAL_IC, bms_ic);

    while (1) {
        wakeup_sleep(TOTAL_IC);

        ADBMS1818_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);

        
        ADBMS1818_pollAdc();

        wakeup_idle(TOTAL_IC);

        // ADBMS1818
        int8_t pec_error = ADBMS1818_rdcv(0, TOTAL_IC, bms_ic);

        if (pec_error == 0) {
            volatile uint16_t cell_1 = bms_ic[0].cells.c_codes[0];
            volatile uint16_t cell_18 = bms_ic[0].cells.c_codes[17];
            
            (void)cell_1; 
            (void)cell_18;
        }

        HAL_Delay(100); 
    }
    
    return 0;
}