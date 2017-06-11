#ifndef UTILITY_H
#define UTILITY_H

#include "stdint.h"


/**
 * Maps a value from the range [0, 100] to [min, max].
 */
uint32_t map(uint8_t value, uint32_t min, uint32_t max);

/**
 * Maps a value from the range [min, max] to [0, 100].
 */
uint8_t pam(uint32_t value, uint32_t min, uint32_t max);


#endif
