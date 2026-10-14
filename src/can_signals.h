#ifndef CAN_SIGNALS_H
#define CAN_SIGNALS_H

#include <stdint.h>

#define CAN_FRAME_LEN 8U

typedef enum {
    CAN_INTEL = 0,     /* little endian, DBC "@1": start bit = LSB  */
    CAN_MOTOROLA = 1   /* big endian,    DBC "@0": start bit = MSB  */
} CanByteOrder;

/* Description of one signal inside an 8-byte CAN frame (like a DBC entry). */
typedef struct {
    uint8_t      start_bit;  /* 0..63, DBC bit numbering                */
    uint8_t      length;     /* 1..32 bits                              */
    CanByteOrder order;
    float        factor;     /* physical = raw * factor + offset        */
    float        offset;
} CanSignal;

/* Signals of the cruise control frame. */
extern const CanSignal CAN_SIG_VEHICLE_SPEED;     /* bit 0,  16 bit, 0.01 km/h */
extern const CanSignal CAN_SIG_BRAKE_PEDAL;       /* bit 16,  1 bit            */
extern const CanSignal CAN_SIG_THROTTLE_REQUEST;  /* bit 24,  8 bit, 0.5 %     */

/* Raw (unscaled) access. Return 1 on success, 0 if the layout is invalid.
 * On failure the frame / output value is left unchanged. */
uint8_t can_pack_raw(uint8_t data[CAN_FRAME_LEN], uint8_t start_bit,
                     uint8_t length, CanByteOrder order, uint32_t raw);

uint8_t can_unpack_raw(const uint8_t data[CAN_FRAME_LEN], uint8_t start_bit,
                       uint8_t length, CanByteOrder order, uint32_t *raw);

/* Physical access: scaling, rounding and limiting to the raw range. */
uint8_t can_encode(uint8_t data[CAN_FRAME_LEN], const CanSignal *sig,
                   float physical);

uint8_t can_decode(const uint8_t data[CAN_FRAME_LEN], const CanSignal *sig,
                   float *physical);

#endif /* CAN_SIGNALS_H */
