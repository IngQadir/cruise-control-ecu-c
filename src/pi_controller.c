#include "pi_controller.h"

void pi_init(PiController *pi, float kp, float ki, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->integral = 0.0f;
}

void pi_reset(PiController *pi)
{
    pi->integral = 0.0f;
}

float pi_step(PiController *pi, float target, float actual, float dt_s)
{
    const float error = target - actual;
    const float integral_candidate = pi->integral + (error * dt_s);
    float output = (pi->kp * error) + (pi->ki * integral_candidate);

    if (output > pi->out_max) {
        output = pi->out_max;
        /* Anti-windup: while saturated high, only accept the new integral
         * value if it moves the output back towards the limits. */
        if (error < 0.0f) {
            pi->integral = integral_candidate;
        }
    } else if (output < pi->out_min) {
        output = pi->out_min;
        /* Same logic for the lower limit. */
        if (error > 0.0f) {
            pi->integral = integral_candidate;
        }
    } else {
        pi->integral = integral_candidate;
    }

    return output;
}
