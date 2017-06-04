#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "joystick.h"
#include "servo.h"
#include "rc_radio.h"
#include "brushed_dc_motor.h"


#define RADIO_TIMER_INSTANCE    (0UL)
#define SERVO_PWM_INSTANCE      (0UL)
#define MOTOR_PWM_INSTANCE      (1UL)

#define BOUND_LED_PIN           (7UL)

#define L_AIL_SERVO_PIN         (31UL)
#define R_AIL_SERVO_PIN         (30UL)
#define ELEVATOR_SERVO_PIN      (29UL)
#define PROP_MOTOR_PIN          (28UL)

#define L_AIL_SERVO_CHAN        (0UL)
#define R_AIL_SERVO_CHAN        (1UL)
#define ELEVATOR_SERVO_CHAN     (2UL)
#define PROP_MOTOR_CHAN         (0UL)

// Don't allow the servos to swing further than MAX_SERVO_DELTA
// in either direction from SERVO_NEUTRAL_VALUE.
#define MAX_SERVO_DELTA         (15UL)

// Don't consider the throttle to be non-zero until it reaches
// (JOYSTICK_NEUTRAL_VALUE + THROTTLE_SAFETY_MARGIN).
#define THROTTLE_SAFETY_MARGIN  (10UL)


static servo_group_t            m_servo_group;
static brushed_dc_motor_group_t m_brushed_dc_motor_group;


static void m_controls_reset(void)
{
    uint32_t err_code;

    err_code = servo_value_set(&m_servo_group,
                                   L_AIL_SERVO_CHAN,
                                   SERVO_NEUTRAL_VALUE);
    APP_ERROR_CHECK(err_code);

    err_code = servo_value_set(&m_servo_group,
                                   R_AIL_SERVO_CHAN,
                                   SERVO_NEUTRAL_VALUE);
    APP_ERROR_CHECK(err_code);

    err_code = servo_value_set(&m_servo_group,
                                   ELEVATOR_SERVO_CHAN,
                                   SERVO_NEUTRAL_VALUE);
    APP_ERROR_CHECK(err_code);

    err_code = brushed_dc_motor_value_set(&m_brushed_dc_motor_group,
                                              PROP_MOTOR_CHAN,
                                              BRUSHED_DC_MOTOR_MIN_VALUE);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Controls reset.\r\n");
}


static inline uint8_t m_pam(uint8_t value, uint8_t min, uint8_t max)
{
    return ((value - min) * 100 / (max - min));
}


static inline uint8_t m_map(uint8_t value, uint8_t min, uint8_t max)
{
    return (value * (max - min) / 100 + min);
}


static void m_ailerons_set(uint8_t raw_roll)
{
    uint32_t err_code;
    uint8_t  roll;

    raw_roll = (JOYSTICK_MAX_VALUE - raw_roll);

    if (JOYSTICK_NEUTRAL_VALUE < raw_roll)
    {
        roll = m_pam(raw_roll, JOYSTICK_NEUTRAL_VALUE, JOYSTICK_MAX_VALUE);
        roll = m_map(roll,
                         SERVO_NEUTRAL_VALUE,
                         (SERVO_NEUTRAL_VALUE + MAX_SERVO_DELTA));
    }
    else
    {
        roll = m_pam(raw_roll, JOYSTICK_MIN_VALUE, JOYSTICK_NEUTRAL_VALUE);
        roll = m_map(roll,
                         (SERVO_NEUTRAL_VALUE - MAX_SERVO_DELTA),
                         SERVO_NEUTRAL_VALUE);
    }

    err_code = servo_value_set(&m_servo_group,
                                   L_AIL_SERVO_CHAN,
                                   roll);
    APP_ERROR_CHECK(err_code);

    err_code = servo_value_set(&m_servo_group,
                                   R_AIL_SERVO_CHAN,
                                   roll);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("  Roll: (%d) -> (%d)\r\n",  raw_roll, roll);
}


static void m_elevator_set(uint8_t raw_pitch)
{
    uint32_t err_code;
    uint8_t  pitch;

    if (JOYSTICK_NEUTRAL_VALUE < raw_pitch)
    {
        pitch = m_pam(raw_pitch, JOYSTICK_NEUTRAL_VALUE, JOYSTICK_MAX_VALUE);
        pitch = m_map(pitch,
                         SERVO_NEUTRAL_VALUE,
                         (SERVO_NEUTRAL_VALUE + MAX_SERVO_DELTA));
    }
    else
    {
        pitch = m_pam(raw_pitch, JOYSTICK_MIN_VALUE, JOYSTICK_NEUTRAL_VALUE);
        pitch = m_map(pitch,
                         (SERVO_NEUTRAL_VALUE - MAX_SERVO_DELTA),
                         SERVO_NEUTRAL_VALUE);
    }

    err_code = servo_value_set(&m_servo_group,
                                   ELEVATOR_SERVO_CHAN,
                                   pitch);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("  Pitch: (%d) -> (%d)\r\n",  raw_pitch, pitch);
}


static void m_propeller_set(uint8_t raw_throttle)
{
    uint32_t err_code;
    uint8_t  throttle;

    if ((JOYSTICK_NEUTRAL_VALUE + THROTTLE_SAFETY_MARGIN) < raw_throttle)
    {
        throttle = m_pam(raw_throttle,
                             JOYSTICK_NEUTRAL_VALUE,
                             JOYSTICK_MAX_VALUE);
        throttle = m_map(throttle,
                             BRUSHED_DC_MOTOR_MIN_VALUE,
                             BRUSHED_DC_MOTOR_MAX_VALUE);
    }
    else
    {
        throttle = 0;
    }

    err_code = brushed_dc_motor_value_set(&m_brushed_dc_motor_group,
                                              PROP_MOTOR_CHAN,
                                              throttle);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("  Throttle: (%d) -> (%d)\r\n",  raw_throttle, throttle);
}


static void m_rc_radio_handler(rc_radio_event_t event, const void * const p_context)
{
    switch (event)
    {
    case RC_RADIO_EVENT_BINDING:
        // Reset all of the control values so the plane doesn't keep flying
        // if the transmitter drops.
        m_controls_reset();

        nrf_gpio_pin_set(BOUND_LED_PIN);
        NRF_LOG_INFO("Binding...\r\n");
        break;
    case RC_RADIO_EVENT_BOUND:
    {
        rc_radio_bind_info_t * p_bind_info;
        p_bind_info = (rc_radio_bind_info_t*) p_context;

        nrf_gpio_pin_clear(BOUND_LED_PIN);
        NRF_LOG_INFO("Bound. (%d, %d)\r\n",
                         p_bind_info->transmitter_channel,
                         p_bind_info->transmit_rate_hz);
    }
        break;
    case RC_RADIO_EVENT_DATA_RECEIVED:
    {
        rc_radio_data_t * p_rc_data;
        p_rc_data = (rc_radio_data_t*) p_context;

        nrf_gpio_pin_clear(BOUND_LED_PIN);
        NRF_LOG_INFO("Data recieved:\r\n");

        m_ailerons_set(p_rc_data->roll);
        m_elevator_set(p_rc_data->pitch);
        m_propeller_set(p_rc_data->throttle);
    }
        break;
    case RC_RADIO_EVENT_PACKET_DROPPED:
        nrf_gpio_pin_set(BOUND_LED_PIN);
        NRF_LOG_INFO("Packet dropped.\r\n");
        break;
    default:
        break;
    };
}


static void m_gpio_init(void)
{
    nrf_gpio_cfg_output(BOUND_LED_PIN);
    nrf_gpio_pin_set(BOUND_LED_PIN);
}


int main(void)
{
    uint32_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    m_gpio_init();

    err_code = servo_group_init(&m_servo_group,
                                    SERVO_PWM_INSTANCE,
                                    L_AIL_SERVO_PIN,
                                    R_AIL_SERVO_PIN,
                                    ELEVATOR_SERVO_PIN,
                                    SERVO_PIN_NOT_USED);
    APP_ERROR_CHECK(err_code);

    err_code = brushed_dc_motor_group_init(&m_brushed_dc_motor_group,
                                              MOTOR_PWM_INSTANCE,
                                              PROP_MOTOR_PIN,
                                              BRUSHED_DC_MOTOR_PIN_NOT_USED,
                                              BRUSHED_DC_MOTOR_PIN_NOT_USED,
                                              BRUSHED_DC_MOTOR_PIN_NOT_USED);
    APP_ERROR_CHECK(err_code);

    err_code = rc_radio_receiver_init(RADIO_TIMER_INSTANCE,
                                          m_rc_radio_handler);
    APP_ERROR_CHECK(err_code);

    err_code = rc_radio_enable();
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        if (false == NRF_LOG_PROCESS())
        {
            __WFE();
        }
    }
}
