#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdint.h>

/* PI controller with output saturation and anti-windup.
 * All state lives in the struct: no globals, no dynamic memory. */
typedef struct {
    float kp;        /* proportional gain */
    float ki;        /* integral gain */
    float integral;  /* accumulated error (error * seconds) */
    float out_min;   /* lower output limit */
    float out_max;   /* upper output limit */
} PiController;

/* Set gains and limits, reset the integral state to zero. */
void pi_init(PiController *pi, float kp, float ki, float out_min, float out_max);

/* Reset only the integral state (e.g. when the controller is re-activated). */
void pi_reset(PiController *pi);

/* One controller step. dt_s is the cycle time in seconds (e.g. 0.01f).
 * Returns the output, limited to out_min..out_max. */
float pi_step(PiController *pi, float target, float actual, float dt_s);

#endif /* PI_CONTROLLER_H */