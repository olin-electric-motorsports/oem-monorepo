
#pragma once

#include <stdint.h>

/*
 * Task to read the voltages from the battery pack
 *
 * This function will start the ADC conversion, wait until it completes, and
 * then read the voltages of all the cells. Cell codes are also stored in
 * cell_data for cell balancing.
 *
 * @param pack_voltage Pointer to a 16-bit number where the pack voltage will
 * be stored (sum of cell codes >> 8)
 *
 * @param ov Number of cells over OVERVOLTAGE_THRESHOLD
 *
 * @param uv Number of cells under UNDERVOLTAGE_THRESHOLD. Includes the bridged
 * unused channels, which read 0V
 *
 * @param pec_errors Incremented once for every register read with a bad PEC
 */
void voltage_task(uint16_t* pack_voltage, uint32_t* ov, uint32_t* uv, uint16_t* pec_errors);

/*
 * Task to read the temperature from the battery pack
 *
 * This function handles communication with multiplexers, ADC reading, and
 * temperature accumulating. Each call reads one mux channel; ot, ut, min_temp
 * and max_temp are only updated once all NUM_MUX_CHANNELS channels have been
 * read (every 8 calls).
 *
 * Temperatures are raw thermistor voltages (100uV units). The thermistors are
 * NTC, so the hottest reading is the LOWEST value.
 *
 * @param ot Number of thermistors over OVERTEMPERATURE_THRESHOLD in the last
 * full sweep
 *
 * @param ut Number of thermistors under UNDERTEMPERATURE_THRESHOLD in the last
 * full sweep
 *
 * @param min_temp Coldest reading (highest voltage) of the last full sweep
 *
 * @param max_temp Hottest reading (lowest voltage) of the last full sweep
 *
 * @param pec_errors Incremented once for every register read with a bad PEC
 */
void temperature_task(uint32_t* ot, uint32_t* ut, uint16_t* min_temp,
                     uint16_t* max_temp, uint16_t* pec_errors);

/*
 * Read the pack current sensor
 *
 * @param current Pack current in centiamps (positive = discharging)
 */
void current_task(int16_t* current);

/*
 * Run the open wire check on every cell sense wire. This takes a few tens of
 * milliseconds, so don't run it every loop.
 *
 * @param ow Number of open wires found
 */
void openwire_task(uint32_t* ow);
