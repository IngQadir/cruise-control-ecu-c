#include "can_signals.h"

#define CAN_FRAME_BITS 64U
#define CAN_MAX_SIGNAL_LEN 32U

const CanSignal CAN_SIG_VEHICLE_SPEED    = { 0U, 16U, CAN_INTEL, 0.01f, 0.0f };
const CanSignal CAN_SIG_BRAKE_PEDAL      = { 16U, 1U, CAN_INTEL, 1.0f,  0.0f };
const CanSignal CAN_SIG_THROTTLE_REQUEST = { 24U, 8U, CAN_INTEL, 0.5f,  0.0f };

/* Frame bit position of signal bit "bit_index" (0 = LSB of the signal).
 *
 * Intel:    the signal grows upwards from the start bit.
 * Motorola: the start bit is the MSB; walking towards the LSB we go down
 *           inside a byte and, after bit 0, jump to bit 7 of the next byte
 *           (DBC "sawtooth" numbering). */
static uint8_t bit_position(uint8_t start_bit, uint8_t length,
                            CanByteOrder order, uint8_t bit_index)
{
    uint8_t pos = start_bit;

    if (order == CAN_INTEL) {
        pos = (uint8_t)(start_bit + bit_index);
    } else {
        const uint8_t steps = (uint8_t)(length - 1U - bit_index);
        uint8_t k;
        for (k = 0U; k < steps; k++) {
            if ((pos % 8U) == 0U) {
                pos = (uint8_t)(pos + 15U);
            } else {
                pos = (uint8_t)(pos - 1U);
            }
        }
    }
    return pos;
}

/* 1 if every bit of the signal lies inside the 64-bit frame. */
static uint8_t layout_valid(uint8_t start_bit, uint8_t length,
                            CanByteOrder order)
{
    uint8_t idx;

    if ((length == 0U) || (length > CAN_MAX_SIGNAL_LEN) ||
        (start_bit >= CAN_FRAME_BITS)) {
        return 0U;
    }
    for (idx = 0U; idx < length; idx++) {
        if (bit_position(start_bit, length, order, idx) >= CAN_FRAME_BITS) {
            return 0U;
        }
    }
    return 1U;
}

uint8_t can_pack_raw(uint8_t data[CAN_FRAME_LEN], uint8_t start_bit,
                     uint8_t length, CanByteOrder order, uint32_t raw)
{
    uint8_t idx;

    if (layout_valid(start_bit, length, order) == 0U) {
        return 0U;
    }
    for (idx = 0U; idx < length; idx++) {
        const uint8_t pos  = bit_position(start_bit, length, order, idx);
        const uint8_t byte = (uint8_t)(pos / 8U);
        const uint8_t mask = (uint8_t)(1U << (pos % 8U));

        if (((raw >> idx) & 1U) != 0U) {
            data[byte] = (uint8_t)(data[byte] | mask);
        } else {
            data[byte] = (uint8_t)(data[byte] & (uint8_t)(~mask));
        }
    }
    return 1U;
}

uint8_t can_unpack_raw(const uint8_t data[CAN_FRAME_LEN], uint8_t start_bit,
                       uint8_t length, CanByteOrder order, uint32_t *raw)
{
    uint32_t value = 0U;
    uint8_t idx;

    if (layout_valid(start_bit, length, order) == 0U) {
        return 0U;
    }
    for (idx = 0U; idx < length; idx++) {
        const uint8_t pos  = bit_position(start_bit, length, order, idx);
        const uint8_t byte = (uint8_t)(pos / 8U);
        const uint8_t mask = (uint8_t)(1U << (pos % 8U));

        if ((data[byte] & mask) != 0U) {
            value |= ((uint32_t)1U << idx);
        }
    }
    *raw = value;
    return 1U;
}

uint8_t can_encode(uint8_t data[CAN_FRAME_LEN], const CanSignal *sig,
                   float physical)
{
    uint32_t max_raw;
    uint32_t raw;
    float scaled;

    if ((sig->factor <= 0.0f) ||
        (layout_valid(sig->start_bit, sig->length, sig->order) == 0U)) {
        return 0U;
    }

    max_raw = (sig->length == CAN_MAX_SIGNAL_LEN)
                  ? 0xFFFFFFFFU
                  : (((uint32_t)1U << sig->length) - 1U);

    scaled = (physical - sig->offset) / sig->factor;

    if (scaled <= 0.0f) {
        raw = 0U;
    } else if (scaled >= (float)max_raw) {
        raw = max_raw;
    } else {
        raw = (uint32_t)(scaled + 0.5f);   /* round to nearest */
    }

    return can_pack_raw(data, sig->start_bit, sig->length, sig->order, raw);
}

uint8_t can_decode(const uint8_t data[CAN_FRAME_LEN], const CanSignal *sig,
                   float *physical)
{
    uint32_t raw = 0U;

    if (can_unpack_raw(data, sig->start_bit, sig->length, sig->order,
                       &raw) == 0U) {
        return 0U;
    }
    *physical = ((float)raw * sig->factor) + sig->offset;
    return 1U;
}
