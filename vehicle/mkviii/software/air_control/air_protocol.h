#ifndef AIR_PROTOCOL_H
#define AIR_PROTOCOL_H
#include "air.h"
#include "air_config.h"
#include <stddef.h>

/* Caller must reject extended, remote and CAN-FD frames before decoding. */
bool air_decode(air_measurements_t *data, uint32_t id,
                const uint8_t *bytes, size_t length, uint32_t now);
void air_encode_status(uint8_t bytes[AIR_CAN_STATUS_BYTES],
                       const air_t *air, const air_inputs_t *inputs);
#endif
