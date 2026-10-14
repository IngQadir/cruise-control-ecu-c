#include "cc_statemachine.h"

static uint8_t speed_in_range(float speed_kmh)
{
    return (uint8_t)((speed_kmh >= CC_SPEED_MIN_KMH) &&
                     (speed_kmh <= CC_SPEED_MAX_KMH));
}

void cc_init(CcContext *ctx)
{
    ctx->state = CC_STATE_OFF;                         /* REQ-01 */
    ctx->target_kmh = 0.0f;
    pi_init(&ctx->pi, CC_KP, CC_KI, CC_THROTTLE_MIN, CC_THROTTLE_MAX);
}

void cc_step(CcContext *ctx, const CcInputs *in, CcOutputs *out, float dt_s)
{
    out->throttle_pct = 0.0f;

    /* REQ-05: an invalid speed signal forces FAULT from any state. */
    if (in->speed_valid == 0U) {
        ctx->state = CC_STATE_FAULT;
        return;
    }

    switch (ctx->state) {
    case CC_STATE_OFF:
        if (in->main_switch != 0U) {
            ctx->state = CC_STATE_STANDBY;
        }
        break;

    case CC_STATE_STANDBY:
        if (in->main_switch == 0U) {
            ctx->state = CC_STATE_OFF;
        } else if ((in->set_pressed != 0U) &&
                   (in->brake_pressed == 0U) &&
                   (speed_in_range(in->speed_kmh) != 0U)) {   /* REQ-02 */
            ctx->target_kmh = in->speed_kmh;
            pi_reset(&ctx->pi);
            ctx->state = CC_STATE_ACTIVE;
        } else {
            /* stay in STANDBY */
        }
        break;

    case CC_STATE_ACTIVE:
        if ((in->brake_pressed != 0U) || (in->cancel_pressed != 0U) ||
            (in->main_switch == 0U)) {                         /* REQ-03, REQ-06 */
            ctx->state = CC_STATE_OFF;
        } else if (speed_in_range(in->speed_kmh) == 0U) {      /* REQ-07 */
            ctx->state = CC_STATE_STANDBY;
        } else {
            out->throttle_pct = pi_step(&ctx->pi, ctx->target_kmh,
                                        in->speed_kmh, dt_s);  /* REQ-04 */
        }
        break;

    case CC_STATE_FAULT:
        /* Recovery: signal valid again AND driver has switched the system off. */
        if (in->main_switch == 0U) {
            ctx->state = CC_STATE_OFF;
        }
        break;

    default:
        ctx->state = CC_STATE_FAULT;
        break;
    }
}
