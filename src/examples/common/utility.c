#include "utility.h"


inline uint8_t map(uint8_t value, uint8_t min, uint8_t max)
{
    return (value * (max - min) / 100 + min);
}


inline uint8_t pam(uint8_t value, uint8_t min, uint8_t max)
{
    return ((value - min) * 100 / (max - min));
}
