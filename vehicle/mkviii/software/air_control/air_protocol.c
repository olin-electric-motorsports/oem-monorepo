/* packs and unpacks the CAN messages, using the old car's byte layout */
#include "air_protocol.h"

/* Read a BMS or IVT message into d and note when it arrived. Anything else
 * (another ID, a wrong length, or an IVT result other than U1) returns false
 * and changes nothing */
bool air_decode(air_measurements_t *d, uint32_t id,
                const uint8_t *p, size_t n, uint32_t now) {
    if (id == AIR_CAN_BMS_ID && n == AIR_CAN_BMS_BYTES) {
        /* Legacy BMS: little endian fault at bit 2, voltage at bit 18. */
        uint32_t raw_mv = ((uint32_t)p[2] >> 2) | ((uint32_t)p[3] << 6) |
                          (((uint32_t)p[4] & 3u) << 14);
        d->bms_state = p[0] & 3u;
        d->bms_fault = ((uint32_t)p[0] >> 2) | ((uint32_t)p[1] << 6) |
                       (((uint32_t)p[2] & 3u) << 14);
        /* DBC: 0.0256 volts/count = 25.6 millivolts/count. */
        d->pack_mv = (raw_mv * 256u + 5u) / 10u;
        d->bms_ms = now;
        d->bms_seen = true;
        return true;
    }
    if (id == AIR_CAN_IVT_ID && n == AIR_CAN_IVT_BYTES && p[0] == 1u) { /* byte 0 = 1: result U1 */
        uint32_t raw = ((uint32_t)p[2] << 24) | ((uint32_t)p[3] << 16) |
                       ((uint32_t)p[4] << 8) | p[5];
        /* DBC: signed big endian millivolts. Avoid implementation-defined casts. */
        d->tractive_mv = raw <= INT32_MAX ? (int32_t)raw : -1 - (int32_t)(~raw);
        d->ivt_error = (p[1] & 0xf0u) != 0; /* any error flag in the top 4 bits */
        d->ivt_ms = now;
        d->ivt_seen = true;
        return true;
    }
    return false;
}

/* Build our 4-byte status message */
void air_encode_status(uint8_t p[AIR_CAN_STATUS_BYTES],
                       const air_t *a, const air_inputs_t *in) {
    p[0] = (uint8_t)a->fault; /* fault code */
    p[1] = (uint8_t)a->state; /* state */
    /* AIR+ and AIR- closed, then the first 6 node bits (TSMS to BMS) */
    p[2] = (uint8_t)in->air_p_closed | ((uint8_t)in->air_n_closed << 1) |
           ((in->shutdown_closed & 0x3fu) << 2);
    /* the last node (e-meter), then IMD OK */
    p[3] = ((in->shutdown_closed >> 6) & 1u) | ((uint8_t)in->imd_ok << 1);
}
