/**
 * A simple wrapper around the nrf_drv_pwm driver's nrf_drv_pwm_simple_playback
 * functionality.
 */
#ifndef ESC_H
#define ESC_H

#include "nrf_pwm.h"
#include "nrf_drv_pwm.h"


#define ESC_THROTTLE_MIN_VALUE     (0L)
#define ESC_THROTTLE_MAX_VALUE     (100UL)

#define ESC_THROTTLE_PIN_NOT_USED  (NRF_DRV_PWM_PIN_NOT_USED)


// Structs of this type need to kept in the global portion (static) of RAM
// (not const) because they are accessed by EasyDMA.
typedef struct
{
    nrf_drv_pwm_t pwm_instance;
    nrf_pwm_values_individual_t pwm_values;
} esc_throttle_group_t;


// The pwm_instance should be in the range [0, 2] on the nRF52832. The chX_pins
// can be set to any GPIO or set to ESC_THROTTLE_PIN_NOT_USED if the channel is
// not required.
uint32_t esc_throttle_group_init(esc_throttle_group_t * p_group,
                                     uint8_t pwm_instance_index,
                                     uint8_t ch0_pin,
                                     uint8_t ch1_pin,
                                     uint8_t ch2_pin,
                                     uint8_t ch3_pin);


// Returns NRF_ERROR_INVALID_PARAM if the given ch_index is not assigned to a
// pin. The p_value variable will be scaled to the range
// [ESC_THROTTLE_MIN_VALUE, ESC_THROTTLE_MAX_VALUE].
uint32_t esc_throttle_value_get(esc_throttle_group_t * p_group,
                                    uint8_t ch_index,
                                    uint8_t * p_value);


// Returns NRF_ERROR_INVALID_PARAM if the given ch_index is not assigned to a
// pin. The value variable shouldbe be in the range
// [ESC_THROTTLE_MIN_VALUE, ESC_THROTTLE_MAX_VALUE].
uint32_t esc_throttle_value_set(esc_throttle_group_t * p_group,
                                    uint8_t ch_index,
                                    uint8_t value);

#endif
