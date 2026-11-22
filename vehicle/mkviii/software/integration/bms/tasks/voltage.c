#include "tasks.h"

#include "vehicle/mkviii/software/integration/bms/bms_config.h"
#include "can_api.h"
#include "vehicle/mkviii/software/integration/bms/utils/fault.h"

#define NUM_CELLS_IN_REG (3)
#define NUM_CELL_REG     (6)
#define NUM_BYTES_IN_REG (6)

// Readings below this are the bridged unused channel(s), not real cells
#define UNUSED_CHANNEL_MAX_CODE (5000) // 0.5V

void voltage_task(uint16_t* pack_voltage, uint32_t* ov, uint32_t* uv,
                  uint16_t* pec_errors) {
    // Only one cell register's worth of bms_voltage messages is sent per call
    // so the 3-deep CAN TX FIFO can keep up. Every register is sent every 6
    // calls.
    static uint8_t broadcast_reg = 0;

    *pack_voltage = 0;
    uint16_t min_cell = UINT16_MAX;

    wakeup_sleep(NUM_ICS);

    // Start cell voltage ADC conversions. Discharge is paused during the
    // measurement so cell balancing doesn't pull the readings down.
    ADBMS181x_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);

    // Blocks until all ADCs are done being read
    ADBMS181x_pollAdc(); // Ignore return value because we don't care how long it
                       // took

    wakeup_idle(NUM_ICS);

    /*
     * Read Cell Group Register A for all ICS,
     * then the Register B, etc.
     */
    uint8_t raw_data[NUM_RX_BYT * NUM_ICS] = { 0 };

    for (uint8_t cell_reg = 0; cell_reg < NUM_CELL_REG; cell_reg++) {
        // Read one register at a time for all segments

        wakeup_idle(NUM_ICS);

        // + 1 because of the way _rdcv_reg is written
        ADBMS181x_rdcv_reg(cell_reg + 1, NUM_ICS, raw_data);

        for (uint8_t ic = 0; ic < NUM_ICS; ic++) { // foreach segment/chip
            // Index for the raw data array
            uint8_t raw_idx = ic * NUM_RX_BYT;

            // Get cell voltages
            uint16_t cells[NUM_CELLS_IN_REG];
            for (uint8_t i = 0; i < NUM_CELLS_IN_REG; i++) {
                cells[i] = raw_data[raw_idx + 2 * i]
                           + (raw_data[raw_idx + 2 * i + 1] << 8);
            }

            // Core receives all 1s when the CSC is MIA
            if ((cells[0] == UINT16_MAX) && (cells[1] == UINT16_MAX)
                && (cells[2] == UINT16_MAX)) {
                set_csc_mia(ic);
                continue;
            }
            clear_csc_mia(ic);

            /*
             * The received PEC for the current_ic
             * is transmitted as the 7th and 8th
             * after the 6 cell voltage data bytes
             */
            uint16_t received_pec
                = (raw_data[raw_idx + 6] << 8) + raw_data[raw_idx + 7];

            uint16_t data_pec
                = pec15_calc(NUM_BYTES_IN_REG, &raw_data[raw_idx]);

            if (received_pec != data_pec) {
                // Don't act on corrupted data. Repeated PEC errors raise
                // BMS_FAULT_PEC in bms.c
                *pec_errors += 1;
                continue;
            }

            for (uint8_t i = 0; i < NUM_CELLS_IN_REG; i++) {
                cell_data.cells[ic].cells.c_codes[cell_reg * NUM_CELLS_IN_REG + i]
                    = cells[i];

                // Accumulate voltage (only append if valid SPI response)
                *pack_voltage += cells[i] >> 8;

                // Check under/overvoltage thresholds
                if (cells[i] >= OVERVOLTAGE_THRESHOLD) {
                    *ov += 1;
                } else if (cells[i] <= UNDERVOLTAGE_THRESHOLD) {
                    *uv += 1;
                }

                if (cells[i] > UNUSED_CHANNEL_MAX_CODE && cells[i] < min_cell) {
                    min_cell = cells[i];
                }
            }

            // Put cell voltages in CAN message
            if (cell_reg == broadcast_reg) {
                bms_voltage.ic = ic;
                bms_voltage.cell = cell_reg;
                bms_voltage.voltage_1 = cells[0];
                bms_voltage.voltage_2 = cells[1];
                bms_voltage.voltage_3 = cells[2];
                can_wait_for_tx_slot();
                can_send_bms_voltage();
            }
        } // end foreach adbms1818
    } // end foreach cell reg (A, B, C, D, E, F)

    broadcast_reg = (broadcast_reg + 1) % NUM_CELL_REG;

    // Set CSC fault based on status of all CSCs
    if (!check_csc_state()) {
        clear_fault(BMS_FAULT_CSC_MIA);
    } else {
        set_fault(BMS_FAULT_CSC_MIA);
    }

    if (min_cell != UINT16_MAX) {
        bms_sense.min_cell_voltage = min_cell;
    }
}
