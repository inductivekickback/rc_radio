#ifndef UTILITY_H
#define UTILITY_H

#include "stdint.h"


/**
 * Maps a value from the range [0, 100] to [min, max].
 */
uint8_t map(uint8_t value, uint8_t min, uint8_t max);

/**
 * Maps a value from the range [min, max] to [0, 100].
 */
uint8_t pam(uint8_t value, uint8_t min, uint8_t max);


#endif
