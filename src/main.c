/* Cruise control demo: simple vehicle model + state machine + CAN frame.
 * Runs a 60 s scenario in a 10 ms cycle and prints a CSV to stdout.
 *
 * Scenario:
 *   t =  1 s  main switch on          -> STANDBY
 *   t =  2 s  SET pressed (0.2 s)     -> ACTIVE, target = current speed
 *   t = 20 s  uphill (15 s)           -> PI controller raises the throttle
 *   t = 45 s  brake pressed (2 s)     -> OFF, then STANDBY again
 *   t = 55 s  speed signal invalid (1 s) -> FAULT
 *   t = 58 s  main switch off         -> OFF
 */
#include <stdio.h>
#include <stdint.h>

#include "cc_statemachine.h"
#include "can_signals.h"

#define DT_S                 0.01f
#define STEPS_PER_SECOND     100U
#define SIM_SECONDS          60U
#define SIM_STEPS            (SIM_SECONDS * STEPS_PER_SECOND)
#define LOG_EVERY_N_STEPS    10U      /* log every 100 ms */

/* Simple longitudinal vehicle model (units: km/h, km/h per second). */
#define START_SPEED_KMH      80.0f
#define THROTTLE_GAIN        0.10f    /* km/h/s per % throttle        */
#define DRAG_GAIN            0.01f    /* 1/s, speed-proportional drag */
#define DRIVER_THROTTLE_PCT   8.0f    /* manual throttle holding 80 km/h */
#define UPHILL_DECEL         0.50f    /* km/h/s                        */
#define BRAKE_DECEL          5.00f    /* km/h/s                        */

static uint8_t in_window(uint32_t step, uint32_t from_s, uint32_t to_s)
{
    return (uint8_t)((step >= (from_s * STEPS_PER_SECOND)) &&
                     (step <  (to_s   * STEPS_PER_SECOND)));
}

static const char *state_name(CcState s)
{
    switch (s) {
    case CC_STATE_OFF:     return "OFF";
    case CC_STATE_STANDBY: return "STANDBY";
    case CC_STATE_ACTIVE:  return "ACTIVE";
    case CC_STATE_FAULT:   return "FAULT";
    default:               return "UNKNOWN";
    }
}

int main(void)
{
    CcContext ctx;
    CcInputs  in;
    CcOutputs out;
    uint8_t   frame[CAN_FRAME_LEN] = {0U};
    float     speed = START_SPEED_KMH;
    uint32_t  step;
    uint8_t   i;

    cc_init(&ctx);
    out.throttle_pct = 0.0f;

    printf("time_s,speed_kmh,state,throttle_pct,brake,can_frame\n");

    for (step = 0U; step < SIM_STEPS; step++) {
        float throttle;
        float accel;

        /* ---- scenario: driver inputs ---- */
        in.main_switch    = (uint8_t)(in_window(step, 1U, 58U));
        in.set_pressed    = (uint8_t)((step >= 200U) && (step < 220U));
        in.cancel_pressed = 0U;
        in.brake_pressed  = in_window(step, 45U, 47U);
        in.speed_valid    = (uint8_t)(in_window(step, 55U, 56U) == 0U);
        in.speed_kmh      = speed;

        /* ---- ECU software ---- */
        cc_step(&ctx, &in, &out, DT_S);

        /* ---- CAN frame the ECU would send ---- */
        (void)can_encode(frame, &CAN_SIG_VEHICLE_SPEED, speed);
        (void)can_encode(frame, &CAN_SIG_BRAKE_PEDAL, (float)in.brake_pressed);
        (void)can_encode(frame, &CAN_SIG_THROTTLE_REQUEST, out.throttle_pct);

        /* ---- vehicle model ---- */
        throttle = (ctx.state == CC_STATE_ACTIVE) ? out.throttle_pct
                                                  : DRIVER_THROTTLE_PCT;
        accel = (THROTTLE_GAIN * throttle) - (DRAG_GAIN * speed);
        if (in_window(step, 20U, 35U) != 0U) {
            accel -= UPHILL_DECEL;
        }
        if (in.brake_pressed != 0U) {
            accel -= BRAKE_DECEL;
        }
        speed += accel * DT_S;
        if (speed < 0.0f) {
            speed = 0.0f;
        }

        /* ---- logging ---- */
        if ((step % LOG_EVERY_N_STEPS) == 0U) {
            printf("%.2f,%.2f,%s,%.1f,%u,",
                   (double)((float)step * DT_S), (double)speed,
                   state_name(ctx.state), (double)out.throttle_pct,
                   (unsigned int)in.brake_pressed);
            for (i = 0U; i < CAN_FRAME_LEN; i++) {
                printf("%02X", (unsigned int)frame[i]);
            }
            printf("\n");
        }
    }
    return 0;
}
