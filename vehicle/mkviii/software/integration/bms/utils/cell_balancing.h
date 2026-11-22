#pragma once

#include <stdbool.h>

/**
 * Write the ADBMS1818 configuration registers with every discharge switch off
 * and read the internal die temperature. Should be run on hardware init.
 */
void cell_balancing_init(void);

/**
 * Update the discharge switches. When enabled, every cell more than
 * BALANCE_THRESHOLD above the lowest cell in the pack is discharged. When
 * disabled, or if any ADBMS1818 die is above DIE_OVERTEMPERATURE_THRESHOLD,
 * every switch is turned off.
 *
 * Uses the cell voltages stored in cell_data by voltage_task(). Call this
 * regularly: the switches turn themselves off after BALANCE_DCTO if the
 * configuration isn't rewritten.
 *
 * @param enable true to balance (only while charging)
 */
void update_cell_balancing(bool enable);
