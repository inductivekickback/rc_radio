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
#include "utility.h"


#define RADIO_TIMER_INSTANCE    (0UL)
#define SERVO_PWM_INSTANCE      (0UL)
#define THROTTLE_PWM_INSTANCE   (1UL)

#define BOUND_LED_PIN           (7UL)

#define YAW_SERVO_PIN           (31UL)
#define ROLL_SERVO_PIN          (30UL)
#define PITCH_SERVO_PIN         (29UL)
#define THROTTLE_PIN            (28UL)

#define YAW_SERVO_CHAN          (0UL)
#define ROLL_SERVO_CHAN         (1UL)
#define PITCH_SERVO_CHAN        (2UL)

#define THROTTLE_CHAN           (0UL)

// Don't allow the servos to swing further than MAX_SERVO_DELTA
// in either direction from SERVO_NEUTRAL_VALUE.
#define MAX_SERVO_DELTA         (15UL)


#ifndef INVERT_ROLL
#define INVERT_ROLL 0
#endif

#ifndef INVERT_YAW
#define INVERT_YAW 0
#endif

#ifndef INVERT_PITCH
#define INVERT_PITCH 0
#endif

#ifndef BDCM
#define BDCM 0
#endif

#ifndef ESC
#define ESC 0
#endif

#if (0 == BDCM)
  #if (0 == ESC)
    #error Either BDCM or ESC needs to be specified.
  #endif
#endif

#if BDCM
  #if ESC
    #error Either BDCM or ESC needs to be specified but not both.
  #endif
#endif

#if BDCM
#include "brushed_dc_motor.h"
#else
#include "electronic_speed_controller.h"
#endif


static servo_group_t m_servo_group;

#if BDCM
static brushed_dc_motor_group_t m_brushed_dc_motor_group;
#else
static esc_throttle_group_t     m_esc_group;
#endif


static void m_controls_reset(void)
{
    uint32_t err_code;

    err_code = servo_value_set(&m_servo_group,
                                   ROLL_SERVO_CHAN,
                                   SERVO_NEUTRAL_VALUE);
    APP_ERROR_CHECK(err_code);

    err_code = servo_value_set(&m_servo_group,
                                   PITCH_SERVO_CHAN,
                                   SERVO_NEUTRAL_VALUE);
    APP_ERROR_CHECK(err_code);

    err_code = servo_value_set(&m_servo_group,
                                   YAW_SERVO_CHAN,
                                   SERVO_NEUTRAL_VALUE);
    APP_ERROR_CHECK(err_code);

#if BDCM
    err_code = brushed_dc_motor_value_set(&m_brushed_dc_motor_group,
                                              THROTTLE_CHAN,
                                              BRUSHED_DC_MOTOR_MIN_VALUE);
    APP_ERROR_CHECK(err_code);
#else
    err_code = esc_throttle_value_set(&m_esc_group,
                                              THROTTLE_CHAN,
                                              ESC_THROTTLE_MIN_VALUE);
    APP_ERROR_CHECK(err_code);
#endif

    NRF_LOG_INFO("Controls reset.\r\n");
}


static void m_roll_set(uint8_t raw_roll)
{
    uint32_t err_code;
    uint8_t  roll;

#if INVERT_ROLL
    raw_roll = (JOYSTICK_MAX_VALUE - raw_roll);
#endif

    if (SERVO_NEUTRAL_VALUE < raw_roll)
    {
        roll = pam(raw_roll, SERVO_NEUTRAL_VALUE, SERVO_MAX_VALUE);
        roll = map(roll,
                         SERVO_NEUTRAL_VALUE,
                         (SERVO_NEUTRAL_VALUE + MAX_SERVO_DELTA));
    }
    else
    {
        roll = pam(raw_roll, SERVO_MIN_VALUE, SERVO_NEUTRAL_VALUE);
        roll = map(roll,
                         (SERVO_NEUTRAL_VALUE - MAX_SERVO_DELTA),
                         SERVO_NEUTRAL_VALUE);
    }

    err_code = servo_value_set(&m_servo_group,
                                   ROLL_SERVO_CHAN,
                                   roll);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("  Roll: (%d) -> (%d)\r\n",  raw_roll, roll);
}


static void m_pitch_set(uint8_t raw_pitch)
{
    uint32_t err_code;
    uint8_t  pitch;

#if INVERT_PITCH
    raw_pitch = (JOYSTICK_MAX_VALUE - raw_pitch);
#endif

    if (SERVO_NEUTRAL_VALUE < raw_pitch)
    {
        pitch = pam(raw_pitch, SERVO_NEUTRAL_VALUE, SERVO_MAX_VALUE);
        pitch = map(pitch,
                         SERVO_NEUTRAL_VALUE,
                         (SERVO_NEUTRAL_VALUE + MAX_SERVO_DELTA));
    }
    else
    {
        pitch = pam(raw_pitch, SERVO_MIN_VALUE, SERVO_NEUTRAL_VALUE);
        pitch = map(pitch,
                         (SERVO_NEUTRAL_VALUE - MAX_SERVO_DELTA),
                         SERVO_NEUTRAL_VALUE);
    }

    err_code = servo_value_set(&m_servo_group,
                                   PITCH_SERVO_CHAN,
                                   pitch);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("  Pitch: (%d) -> (%d)\r\n",  raw_pitch, pitch);
}


static void m_yaw_set(uint8_t raw_yaw)
{
    uint32_t err_code;
    uint8_t  yaw;

#if INVERT_YAW
    raw_yaw = (JOYSTICK_MAX_VALUE - raw_yaw);
#endif

    if (SERVO_NEUTRAL_VALUE < raw_yaw)
    {
        yaw = pam(raw_yaw, SERVO_NEUTRAL_VALUE, SERVO_MAX_VALUE);
        yaw = map(yaw,
                     SERVO_NEUTRAL_VALUE,
                     (SERVO_NEUTRAL_VALUE + MAX_SERVO_DELTA));
    }
    else
    {
        yaw = pam(raw_yaw, SERVO_MIN_VALUE, SERVO_NEUTRAL_VALUE);
        yaw = map(yaw,
                     (SERVO_NEUTRAL_VALUE - MAX_SERVO_DELTA),
                     SERVO_NEUTRAL_VALUE);
    }

    err_code = servo_value_set(&m_servo_group,
                                   YAW_SERVO_CHAN,
                                   yaw);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("  Yaw: (%d) -> (%d)\r\n",  raw_yaw, yaw);
}


static void m_throttle_set(uint8_t raw_throttle)
{
    uint32_t err_code;
    uint8_t  throttle;

#if BDCM
    throttle = map(raw_throttle,
                       BRUSHED_DC_MOTOR_MIN_VALUE,
                       BRUSHED_DC_MOTOR_MAX_VALUE);

    err_code = brushed_dc_motor_value_set(&m_brushed_dc_motor_group,
                                              THROTTLE_CHAN,
                                              throttle);
    APP_ERROR_CHECK(err_code);
#else
    throttle = map(raw_throttle,
                       ESC_THROTTLE_MIN_VALUE,
                       ESC_THROTTLE_MAX_VALUE);

    err_code = esc_throttle_value_set(&m_esc_group,
                                          THROTTLE_CHAN,
                                          throttle);
    APP_ERROR_CHECK(err_code);
#endif

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

        m_roll_set(p_rc_data->roll);
        m_pitch_set(p_rc_data->pitch);
        m_throttle_set(p_rc_data->throttle);
        m_yaw_set(p_rc_data->yaw);
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
                                    YAW_SERVO_PIN,
                                    ROLL_SERVO_PIN,
                                    PITCH_SERVO_PIN,
                                    SERVO_PIN_NOT_USED);
    APP_ERROR_CHECK(err_code);

#if BDCM
    err_code = brushed_dc_motor_group_init(&m_brushed_dc_motor_group,
                                              THROTTLE_PWM_INSTANCE,
                                              THROTTLE_PIN,
                                              BRUSHED_DC_MOTOR_PIN_NOT_USED,
                                              BRUSHED_DC_MOTOR_PIN_NOT_USED,
                                              BRUSHED_DC_MOTOR_PIN_NOT_USED);
    APP_ERROR_CHECK(err_code);
#else
    err_code = esc_throttle_group_init(&m_esc_group,
                                           THROTTLE_PWM_INSTANCE,
                                           THROTTLE_PIN,
                                           ESC_THROTTLE_PIN_NOT_USED,
                                           ESC_THROTTLE_PIN_NOT_USED,
                                           ESC_THROTTLE_PIN_NOT_USED);
    APP_ERROR_CHECK(err_code);
#endif

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
