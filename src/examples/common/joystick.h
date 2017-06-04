/**
 * A simple wrapper that uses a timer and the PPI to trigger ADC reads. The
 * values are scaled before the joystick_event_handler is called.
 */
#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "stdint.h"
#include "nrf52_bitfields.h"


#define JOYSTICK_MIN_VALUE      (0UL)
#define JOYSTICK_NEUTRAL_VALUE  (50UL)
#define JOYSTICK_MAX_VALUE      (100UL)
#define JOYSTICK_INVALID_VALUE  (0xFFUL)


typedef enum
{
    JOYSTICK_PIN_0        = SAADC_CH_PSELP_PSELP_AnalogInput0, // P0.2 (XL2)
    JOYSTICK_PIN_1        = SAADC_CH_PSELP_PSELP_AnalogInput1, // P0.3
    JOYSTICK_PIN_2        = SAADC_CH_PSELP_PSELP_AnalogInput2, // P0.4
    JOYSTICK_PIN_3        = SAADC_CH_PSELP_PSELP_AnalogInput3, // P0.5
    JOYSTICK_PIN_4        = SAADC_CH_PSELP_PSELP_AnalogInput4, // P0.28
    JOYSTICK_PIN_5        = SAADC_CH_PSELP_PSELP_AnalogInput5, // P0.29
    JOYSTICK_PIN_6        = SAADC_CH_PSELP_PSELP_AnalogInput6, // P0.30
    JOYSTICK_PIN_7        = SAADC_CH_PSELP_PSELP_AnalogInput7, // P0.31
    JOYSTICK_PIN_NOT_USED = 0xFF,
    JOYSTICK_PIN_AIN_COUNT
} joystick_pin_t;


typedef void (*joystick_event_handler_t)(uint8_t l_axis_x_value,
	                                         uint8_t l_axis_y_value,
	                                         uint8_t r_axis_x_value,
	                                         uint8_t r_axis_y_value);


/**
 * Inits a timer that triggers the SAADC at the given rate. The timer instance
 * is used to select a free timer peripheral (e.g. 0 is converted to TIMER0).
 */
uint32_t joystick_init(uint8_t timer_instance_index,
	                       uint8_t update_rate_hz,
	                       joystick_event_handler_t joystick_event_handler,
	                       joystick_pin_t l_x_axis_pin,
	                       joystick_pin_t l_y_axis_pin,
	                       joystick_pin_t r_x_axis_pin,
	                       joystick_pin_t r_y_axis_pin);

#endif
