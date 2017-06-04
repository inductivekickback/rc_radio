/**
 * A simple wrapper around the nrf_drv_pwm driver's nrf_drv_pwm_simple_playback
 * functionality.
 */
#ifndef SERVO_H
#define SERVO_H

#include "nrf_pwm.h"
#include "nrf_drv_pwm.h"


#define SERVO_MIN_VALUE      (0UL)
#define SERVO_NEUTRAL_VALUE  (50UL)
#define SERVO_MAX_VALUE      (100UL)

#define SERVO_PIN_NOT_USED  (NRF_DRV_PWM_PIN_NOT_USED)


// Structs of this type need to kept in the global portion (static) of RAM
// (not const) because they are accessed by EasyDMA.
typedef struct
{
    nrf_drv_pwm_t pwm_instance;
    nrf_pwm_values_individual_t pwm_values;
} servo_group_t;


// The pwm_instance should be in the range [0, 2] on the nRF52832. The chX_pins
// can be set to any GPIO or set to SERVO_PIN_NOT_USED if the channel is not
// required.
uint32_t servo_group_init(servo_group_t * p_group,
                              uint8_t pwm_instance_index,
                              uint8_t ch0_pin,
                              uint8_t ch1_pin,
                              uint8_t ch2_pin,
                              uint8_t ch3_pin);


// Returns NRF_ERROR_INVALID_PARAM if the given ch_index is not assigned to a
// pin. The p_value variable will be scaled to the range
// [SERVO_MIN_VALUE, SERVO_MAX_VALUE].
uint32_t servo_value_get(servo_group_t * p_group,
                             uint8_t ch_index,
                             uint8_t * p_value);


// Returns NRF_ERROR_INVALID_PARAM if the given ch_index is not assigned to a
// pin. The value variable shouldbe be in the range
// [SERVO_MIN_VALUE, SERVO_MAX_VALUE].
uint32_t servo_value_set(servo_group_t * p_group,
                             uint8_t ch_index,
                             uint8_t value);

#endif
