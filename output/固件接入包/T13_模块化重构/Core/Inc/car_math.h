#ifndef CAR_MATH_H
#define CAR_MATH_H
#include <stdint.h>

static inline int32_t CarMath_Delta16(uint16_t now, uint16_t before)
{
    uint32_t d = ((uint32_t)now - (uint32_t)before) & 0xffffU;
    return d < 32768U ? (int32_t)d : (int32_t)d - 65536;
}

static inline int CarMath_Round(float value)
{
    return (int)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static inline float CarMath_WrapDegrees(float x)
{
    if (x > 180.0f) x -= 360.0f;
    if (x < -180.0f) x += 360.0f;
    return x;
}

#endif
