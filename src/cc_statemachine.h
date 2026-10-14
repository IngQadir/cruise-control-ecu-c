#ifndef CC_STATEMACHINE_H
#define CC_STATEMACHINE_H

#include <stdint.h>
#include "pi_controller.h"

/* Speed window in which cruise control may be active (km/h). */
#define CC_SPEED_MIN_KMH   30.0f
#define CC_SPEED_MAX_KMH  180.0f

/* PI controller tuning and throttle limits (percent). */
#define CC_KP              2.0f
#define CC_KI              0.5f
#define CC_THROTTLE_MIN    0.0f
#define CC_THROTTLE_MAX  100.0f

typedef enum {
    CC_STATE_OFF = 0,
    CC_STATE_STANDBY,
    CC_STATE_ACTIVE,
    CC_STATE_FAULT
} CcState;

typedef struct {
    uint8_t main_switch;     /* 1 = driver switched cruise control on  */
    uint8_t set_pressed;     /* 1 = SET button pressed                 */
    uint8_t cancel_pressed;  /* 1 = CANCEL button pressed              */
    uint8_t brake_pressed;   /* 1 = brake pedal pressed                */
    uint8_t speed_valid;     /* 1 = speed signal is valid              */
    float   speed_kmh;       /* vehicle speed                          */
} CcInputs;

typedef struct {
    float throttle_pct;      /* throttle request, 0..100 %             */
} CcOutputs;

typedef struct {
    CcState      state;
    float        target_kmh;
    PiController pi;
} CcContext;

/* Put the module into its start-up condition (state OFF). */
void cc_init(CcContext *ctx);

/* One cycle. dt_s = cycle time in seconds (e.g. 0.01f). */
void cc_step(CcContext *ctx, const CcInputs *in, CcOutputs *out, float dt_s);

#endif /* CC_STATEMACHINE_H */
