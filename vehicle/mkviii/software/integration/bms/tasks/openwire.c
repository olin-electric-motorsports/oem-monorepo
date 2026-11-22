#include "tasks.h"

#include "vehicle/mkviii/software/integration/bms/bms_config.h"
#include "can_api.h"
#include "vehicle/mkviii/software/integration/bms/utils/fault.h"

#define NUM_CELLS_IN_REG (3)
#define NUM_CELL_REG     (6)
#define NUM_BYTES_IN_REG (6)
#define NUM_CHANNELS     (NUM_CELLS_IN_REG * NUM_CELL_REG) // 18 on the ADBMS1818

// The reference driver runs each ADOW 3 times so the pull-up/down current has
// time to move an open wire's voltage
#define NUM_ADOW_REPEATS (3)

/*
 * Run ADOW with the given pull current and read every cell register.
 *
 * @param codes Output, NUM_CHANNELS codes per IC
 * @param pec_ok Output, cleared for an IC if any of its registers had a bad PEC
 */
static void read_adow(uint8_t pup, uint16_t codes[NUM_ICS][NUM_CHANNELS],
                      bool pec_ok[NUM_ICS]) {
    for (uint8_t i = 0; i < NUM_ADOW_REPEATS; i++) {
        wakeup_idle(NUM_ICS);
        ADBMS181x_adow(MD_7KHZ_3KHZ, pup, CELL_CH_ALL, DCP_DISABLED);
        ADBMS181x_pollAdc();
    }

    uint8_t raw_data[NUM_RX_BYT * NUM_ICS] = { 0 };

    for (uint8_t cell_reg = 0; cell_reg < NUM_CELL_REG; cell_reg++) {
        wakeup_idle(NUM_ICS);
        ADBMS181x_rdcv_reg(cell_reg + 1, NUM_ICS, raw_data);

        for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
            uint8_t* raw = &raw_data[ic * NUM_RX_BYT];

            uint16_t received_pec = (raw[6] << 8) + raw[7];
            if (received_pec != pec15_calc(NUM_BYTES_IN_REG, raw)) {
                pec_ok[ic] = false;
            }

            for (uint8_t i = 0; i < NUM_CELLS_IN_REG; i++) {
                codes[ic][cell_reg * NUM_CELLS_IN_REG + i]
                    = raw[2 * i] + (raw[2 * i + 1] << 8);
            }
        }
    }
}

/*
 * Open wire detection, following ADBMS181x_run_openwire_single() from the
 * reference driver:
 *   - Measure every channel with the pull-up current, then pull-down current
 *   - If pull-up minus pull-down is above OPEN_WIRE_THRESHOLD, the C pin
 *     above that channel is open
 *   - If the bottom channel reads 0 with pull-up, C0 is open
 *   - If the top channel reads 0 with pull-up, the top C pin is open
 *
 * NOTE: if the bridged unused channel is the bottom or top channel, the C0 / top
 * pin checks will report it as open. Mask it out once you know which channel
 * it is.
 */
void openwire_task(uint32_t* ow) {
    // Clear any previous open_wire flags
    *ow = 0;

    uint16_t pull_up[NUM_ICS][NUM_CHANNELS];
    uint16_t pull_down[NUM_ICS][NUM_CHANNELS];
    bool pec_ok[NUM_ICS];
    for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
        pec_ok[ic] = true;
    }

    // Make sure IC's are in the right state.
    wakeup_sleep(NUM_ICS);
    ADBMS181x_clrcell();

    read_adow(PULL_UP_CURRENT, pull_up, pec_ok);
    read_adow(PULL_DOWN_CURRENT, pull_down, pec_ok);

    for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
        if (!pec_ok[ic]) {
            // Can't trust this IC's data, PEC errors are handled elsewhere
            continue;
        }

        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            // A bridged unused channel reads 0 for both, so it never trips this
            if (pull_up[ic][ch] > pull_down[ic][ch]
                && (pull_up[ic][ch] - pull_down[ic][ch]) > OPEN_WIRE_THRESHOLD) {
                *ow += 1;
                bms_metrics.open_wire_ic = ic;
                bms_metrics.open_wire_cell = ch + 1; // C pin number
            }
        }

        if (pull_up[ic][0] == 0) {
            *ow += 1;
            bms_metrics.open_wire_ic = ic;
            bms_metrics.open_wire_cell = 0;
        }

        if (pull_up[ic][NUM_CHANNELS - 1] == 0) {
            *ow += 1;
            bms_metrics.open_wire_ic = ic;
            bms_metrics.open_wire_cell = NUM_CHANNELS;
        }
    }

    // Clear the ADOW results so they aren't mistaken for cell voltages
    wakeup_idle(NUM_ICS);
    ADBMS181x_clrcell();
}
