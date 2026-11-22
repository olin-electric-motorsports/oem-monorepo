#include "cell_balancing.h"
#include "vehicle/mkviii/software/integration/bms/bms_config.h"
#include "can_api.h"
#include "vehicle/mkviii/software/integration/bms/utils/fault.h"

#include <string.h>

#define NUM_CHANNELS     (18)
#define NUM_BYTES_IN_REG (6)

// Readings below this are the bridged unused channel(s), not real cells
#define UNUSED_CHANNEL_MAX_CODE (5000) // 0.5V

// CFGRA byte 0: GPIO5..1 = 1 (pull-downs off, needed for the thermistor and
// I2C GPIOs), REFON = 1 (reference stays on between conversions)
#define CFGRA0 (0xF8 | 0x04)
// CFGRB byte 0 low nibble: GPIO9..6 = 1
#define CFGRB0_GPIO (0x0F)

/*
 * Read every ADBMS1818's internal die temperature (status register A).
 *
 * @return true if every die read back with a good PEC and is below
 * DIE_OVERTEMPERATURE_THRESHOLD
 */
static bool die_temperatures_ok(void) {
    wakeup_sleep(NUM_ICS);

    // Start and wait for ADC conversion for internal temp
    ADBMS181x_adstat(MD_7KHZ_3KHZ, STAT_CH_ITEMP);
    ADBMS181x_pollAdc();
    wakeup_idle(NUM_ICS);

    uint8_t raw_data[NUM_RX_BYT * NUM_ICS] = { 0 };
    // reg = 1, read back status group A
    ADBMS181x_rdstat_reg(1, NUM_ICS, raw_data);

    bool ok = true;
    uint16_t hottest = 0;
    for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
        uint8_t* raw = &raw_data[ic * NUM_RX_BYT];

        uint16_t received_pec = (raw[6] << 8) + raw[7];
        if (received_pec != pec15_calc(NUM_BYTES_IN_REG, raw)) {
            ok = false;
            continue;
        }

        uint16_t die_temp = raw[2] + (raw[3] << 8);
        if (die_temp > hottest) {
            hottest = die_temp;
        }
        if (die_temp >= DIE_OVERTEMPERATURE_THRESHOLD) {
            ok = false;
        }
    }

    bms_debug.internal_die_temp = hottest;
    return ok;
}

/*
 * Write CFGRA and CFGRB on every IC with the given discharge bits.
 *
 * @param dcc Per IC, bit n set = discharge cell n + 1 (DCC1..DCC18)
 */
static void write_discharge(uint32_t dcc[NUM_ICS]) {
    uint8_t cfgra[NUM_BYTES_IN_REG * NUM_ICS];
    uint8_t cfgrb[NUM_BYTES_IN_REG * NUM_ICS];
    memset(cfgra, 0, sizeof(cfgra));
    memset(cfgrb, 0, sizeof(cfgrb));

    for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
        uint8_t* a = &cfgra[ic * NUM_BYTES_IN_REG];
        uint8_t* b = &cfgrb[ic * NUM_BYTES_IN_REG];

        // CFGRA: [0] GPIO/REFON/DTEN/ADCOPT, [1..3] VUV/VOV (unused, left 0),
        // [4] DCC8..1, [5] DCTO[3:0] | DCC12..9
        a[0] = CFGRA0;
        a[4] = dcc[ic] & 0xFF;
        a[5] = (BALANCE_DCTO << 4) | ((dcc[ic] >> 8) & 0x0F);

        // CFGRB: [0] DCC16..13 | GPIO9..6, [1] DCC18 (bit 1), DCC17 (bit 0)
        b[0] = (((dcc[ic] >> 12) & 0x0F) << 4) | CFGRB0_GPIO;
        b[1] = (dcc[ic] >> 16) & 0x03;
    }

    uint8_t wrcfga[2] = { 0x00, 0x01 };
    uint8_t wrcfgb[2] = { 0x00, 0x24 };

    wakeup_idle(NUM_ICS);
    write_68(NUM_ICS, wrcfga, cfgra);
    write_68(NUM_ICS, wrcfgb, cfgrb);
}

void cell_balancing_init(void) {
    uint32_t dcc[NUM_ICS] = { 0 };
    wakeup_sleep(NUM_ICS);
    write_discharge(dcc);

    (void)die_temperatures_ok();
    bms_core.cell_balancing_status = false;
}

void update_cell_balancing(bool enable) {
    uint32_t dcc[NUM_ICS] = { 0 };

    if (enable && die_temperatures_ok()) {
        // Find the lowest real cell in the pack
        uint16_t min_cell = UINT16_MAX;
        for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
            for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
                uint16_t v = cell_data.cells[ic].cells.c_codes[ch];
                if (v > UNUSED_CHANNEL_MAX_CODE && v < min_cell) {
                    min_cell = v;
                }
            }
        }

        if (min_cell != UINT16_MAX) {
            for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
                for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
                    uint16_t v = cell_data.cells[ic].cells.c_codes[ch];
                    if (v > UNUSED_CHANNEL_MAX_CODE
                        && v > min_cell + BALANCE_THRESHOLD) {
                        dcc[ic] |= (1UL << ch);
                    }
                }
            }
        }
    }

    bool balancing = false;
    for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
        if (dcc[ic]) {
            balancing = true;
        }
    }

    write_discharge(dcc);
    bms_core.cell_balancing_status = balancing;
}
