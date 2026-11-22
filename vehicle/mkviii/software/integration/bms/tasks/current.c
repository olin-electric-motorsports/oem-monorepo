#include "tasks.h"
#include "vehicle/mkviii/software/integration/bms/bms_config.h"
#include "can_api.h"
#include <stdint.h>

#define NUM_SAMPLES (16)

// 12-bit ADC, VDDA = 3.3V
#define ADC_FULL_SCALE (4095)
#define ADC_VREF_MV    (3300)

// R2 = 5.1k, R4 = 10k: V_sensor = V_pin * (5.1k + 10k) / 10k
#define DIVIDER_NUM (151)
#define DIVIDER_DEN (100)

void current_task(int16_t* current) {
    uint32_t raw_sum = 0;
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        raw_sum += oem_adc_read(&current_sense_vout);
    }

    // Average, then convert to the sensor output voltage in mV
    int32_t pin_mv = (int32_t)(raw_sum * ADC_VREF_MV / ADC_FULL_SCALE / NUM_SAMPLES);
    int32_t sensor_mv = pin_mv * DIVIDER_NUM / DIVIDER_DEN;
    bms_sense.current_vout = (uint16_t)sensor_mv;

    // Remove offset, then convert to cA (centiAmps)
    int32_t current_ca = (sensor_mv - CURRENT_SENSE_ZERO_MV) * 100 / CURRENT_SENSE_MV_PER_A;

    if (current_ca > INT16_MAX) {
        current_ca = INT16_MAX;
    } else if (current_ca < INT16_MIN) {
        current_ca = INT16_MIN;
    }
    *current = (int16_t)current_ca;
}
