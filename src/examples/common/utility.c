#include "utility.h"


inline uint32_t map(uint8_t value, uint32_t min, uint32_t max)
{
    return (value * (max - min) / 100 + min);
}


inline uint8_t pam(uint32_t value, uint32_t min, uint32_t max)
{
    return ((value - min) * 100 / (max - min));
}
