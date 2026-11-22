#include "tasks.h"

#include "vehicle/mkviii/software/integration/bms/bms_config.h"
#include "can_api.h"
#include "vehicle/mkviii/software/integration/bms/utils/i2c_helpers.h"
#include "vehicle/mkviii/software/integration/bms/utils/fault.h"
#include <stdint.h>
#include <string.h>

#define NUM_MUX_CHANNELS              (8)
#define NUM_BYTES_IN_REG              (6)
#define NUM_TEMPS_PER_READ            (6)

#define AUX_REG_GROUP_A (1)
#define AUX_REG_GROUP_C (3)

/*
 * We are using negative-temperature-coefficient thermistors.
 * TL;DR is as the temperature increases, the voltage decreases.
 * These variables refer the the voltages, so the minimum temperature is the
 * highest voltage, and vice versa.
 */
static void update_min_max_temps(uint16_t* min_temp, uint16_t* max_temp,
                                 uint16_t temps[], uint8_t num_temps) {
    for (uint8_t i = 0; i < num_temps; i++) {
        if (temps[i] > *min_temp) {
            *min_temp = temps[i];
        }
        if (temps[i] < *max_temp) {
            *max_temp = temps[i];
        }
    }
}

static uint16_t read_temp(uint8_t* raw, uint8_t idx) {
    return raw[idx] | (raw[idx + 1] << 8);
}

static uint16_t check_pec(uint8_t* raw) {
    uint16_t received_pec = (raw[6] << 8) | raw[7];
    return received_pec != pec15_calc(NUM_BYTES_IN_REG, raw);
}

void temperature_task(uint32_t* ot, uint32_t* ut, uint16_t* min_temp,
                     uint16_t* max_temp, uint16_t* pec_errors) {
    static uint8_t channel = 0;

    // Accumulated over one sweep of all mux channels
    static uint32_t sweep_ot = 0;
    static uint32_t sweep_ut = 0;
    static uint16_t sweep_min = 0;
    static uint16_t sweep_max = UINT16_MAX;

    bms_temperature.channel = channel;

    // All four muxes are switched to the same channel. Each mux output goes
    // to its own ADBMS1818 GPIO, so one ADAX reads 4 muxed thermistors.
    wakeup_sleep(NUM_ICS);
    uint8_t mux_ack = 0;
    for (uint8_t mux = 0; mux < NUM_MUXES; mux++) {
        if (configure_mux_until_ack(NUM_ICS, MUXES[mux], MUX_ENABLE, channel, 10)) {
            mux_ack |= (1 << mux);
        }
    }

    bms_mux.num_mux = NUM_MUXES;
    bms_mux.mux_ack = mux_ack;
    if (mux_ack == ((1 << NUM_MUXES) - 1)) {
        clear_fault(BMS_FAULT_MUX_MIA);
    } else {
        set_fault(BMS_FAULT_MUX_MIA);
        bms_metrics.i2c_error_count++;
    }

    ADBMS181x_adax(MD_7KHZ_3KHZ, AUX_CH_ALL);
    (void)ADBMS181x_pollAdc();

    uint8_t aux_reg_a_raw[NUM_RX_BYT * NUM_ICS];
    uint8_t aux_reg_c_raw[NUM_RX_BYT * NUM_ICS];

    wakeup_idle(NUM_ICS);
    ADBMS181x_rdaux_reg(AUX_REG_GROUP_A, NUM_ICS, aux_reg_a_raw); // for GPIOS 1-3
    ADBMS181x_rdaux_reg(AUX_REG_GROUP_C, NUM_ICS, aux_reg_c_raw); // for GPIOS 6-8

    for (uint8_t ic = 0; ic < NUM_ICS; ic++) {
        uint8_t* reg_a = &aux_reg_a_raw[ic * NUM_RX_BYT];
        uint8_t* reg_c = &aux_reg_c_raw[ic * NUM_RX_BYT];

        bool pec_bad = false;
        if (check_pec(reg_a)) {
            *pec_errors += 1;
            pec_bad = true;
        }
        if (check_pec(reg_c)) {
            *pec_errors += 1;
            pec_bad = true;
        }
        if (pec_bad) {
            // Don't act on corrupted data. Repeated PEC errors raise
            // BMS_FAULT_PEC in bms.c
            continue;
        }

        // GPIO1, GPIO2, GPIO3 | GPIO6, GPIO7, GPIO8
        uint16_t temps[NUM_TEMPS_PER_READ] = {
            read_temp(reg_a, 0), read_temp(reg_a, 2), read_temp(reg_a, 4),
            read_temp(reg_c, 0), read_temp(reg_c, 2), read_temp(reg_c, 4),
        };

        bms_temperature.ic = ic;

        bms_temperature.mux_chips = MUX_CHIPS_MUX_CHIPS_12;
        bms_temperature.temperature_1 = temps[0];
        bms_temperature.temperature_2 = temps[1];
        can_wait_for_tx_slot();
        can_send_bms_temperature();

        bms_temperature.mux_chips = MUX_CHIPS_MUX_CHIPS_34;
        bms_temperature.temperature_1 = temps[2];
        bms_temperature.temperature_2 = temps[3];
        can_wait_for_tx_slot();
        can_send_bms_temperature();

        bms_temperature.mux_chips = MUX_CHIPS_NO_MUX;
        bms_temperature.temperature_1 = temps[4];
        bms_temperature.temperature_2 = temps[5];
        can_wait_for_tx_slot();
        can_send_bms_temperature();

        update_min_max_temps(&sweep_min, &sweep_max, temps, NUM_TEMPS_PER_READ);

        for (uint8_t i = 0; i < NUM_TEMPS_PER_READ; i++) {
            // GPIO7 and GPIO8 aren't muxed, so only count them once per sweep
            if (i >= 4 && channel != 0) {
                break;
            }
            // Lower voltage = hotter
            if (temps[i] < OVERTEMPERATURE_THRESHOLD) {
                sweep_ot += 1;
            } else if (temps[i] > UNDERTEMPERATURE_THRESHOLD) {
                sweep_ut += 1;
            }
        }
    }

    channel += 1;
    // Publish and reset once every channel has been read
    if (channel == NUM_MUX_CHANNELS) {
        channel = 0;

        *ot = sweep_ot;
        *ut = sweep_ut;
        *min_temp = sweep_min;
        *max_temp = sweep_max;
        bms_sense.min_temperature = sweep_min;
        bms_sense.max_temperature = sweep_max;

        sweep_ot = 0;
        sweep_ut = 0;
        sweep_min = 0;
        sweep_max = UINT16_MAX;
    }
}
