#include "car_pi.h"

/* Existing anti-windup PI, with the same expression order and limits. */
float CarPI_Update(float target, float measured, float kp, float ki,
                   float seconds, int limit, float *integral)
{
    float error = target - measured;
    float candidate = (*integral) + ki * error * seconds;
    float trial = kp * error + candidate;
    if (!((trial > limit && error > 0.0f) ||
          (trial < 0.0f && error < 0.0f))) {
        if (candidate < 0.0f) candidate = 0.0f;
        if (candidate > limit) candidate = (float)limit;
        (*integral) = candidate;
    }
    float out = kp * error + (*integral);
    if (out < 0.0f) out = 0.0f;
    if (out > limit) out = (float)limit;
    return out;
}
