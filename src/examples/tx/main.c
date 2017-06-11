 /**
 * The throttle is kept at neutral until its value exceeds
 * THROTTLE_SAFETY_MARGIN. This is meant to keep noise on the joystick ADC pin
 * from activating the throttle unexpectedly.
 *
 * There are typically two types of throttle joysticks on RC transmitters:
 *   1) Ones that automatically return to the center position on both X
 *      and Y axis
 *   2) Ones that automatically return to the center position on the X
 *      axis but can be left at an arbitrary position on the Y axis
 *
 * This module assumes that the throttle control is linked to the left
 * joystick's Y axis. There are three throttle control modes:
 *    1) THROTTLE_CTL_FWD_ONLY_NEUTRAL_0 - Throttle is neutral at joystick
 *       position 0 and values match the joystick's position.
 *    2) THROTTLE_CTL_FWD_ONLY_NEUTRAL_50 - Throttle is neutral at joystick
 *       position 50. Joystick values below 50 are considered 0. Values are
 *       mapped from joystick range [50, 100] to [0, 100].
 *    3) THROTTLE_CTL_FWD_BKWD_NEUTRAL_50 - The same as
 *       THROTTLE_CTL_FWD_ONLY_NEUTRAL_0 except THROTTLE_SAFETY_MARGIN is
 *       applied to values on both sides of the neutral point.
 *
 * BOUND_LED_PIN selects an LED to light after a receiver has ACK'd and the
 * transmitter begins transmitting data.
 *
 * INVERT_PITCH_BUTTON_PIN is used as a toggle to invert the pitch data (right
 * joystick's Y axis). INVERTED_PITCH_LED_PIN selects an LED to light when the
 * pitch is inverted.
 *
 * THROT_CTL_BUTTON_PIN is used as a cycle through the possible throttle_ctl_t
 * modes. THROT_CTL_CHANGED_LED_PIN selects an LED to light when the mode
 * has been changed from the default mode (i.e. a warning).
 *
 * BIND_RESET_BUTTON_PIN is used to reset the transmitter back to binding mode.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "app_button.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "joystick.h"
#include "rc_radio.h"
#include "utility.h"


#define RADIO_TIMER_INSTANCE      (0UL)
#define JOYSTICK_TIMER_INSTANCE   (1UL)

#define RADIO_UPDATE_RATE_HZ      (100UL)
#define JOYSTICK_UPDATE_RATE_HZ   (50UL)

#define THROTTLE_CTL_DEFAULT      (THROTTLE_CTL_FWD_ONLY_NEUTRAL_50)
#define THROTTLE_SAFETY_MARGIN    (5UL)

#define INVERTED_PITCH_LED_PIN    (LED_1)
#define THROT_CTL_CHANGED_LED_PIN (LED_3)
#define BOUND_LED_PIN             (LED_4)
#define INVERT_PITCH_BUTTON_PIN   (BUTTON_1)
#define THROT_CTL_BUTTON_PIN      (BUTTON_3)
#define BIND_RESET_BUTTON_PIN     (BUTTON_4)
#define LEFT_X_JS_PIN             (JOYSTICK_PIN_1) // P0.3
#define LEFT_Y_JS_PIN             (JOYSTICK_PIN_2) // P0.4
#define RIGHT_X_JS_PIN            (JOYSTICK_PIN_4) // P0.28
#define RIGHT_Y_JS_PIN            (JOYSTICK_PIN_5) // P0.29

#define NEUTRAL_50_JOYSTICK_VALUE (50UL)


typedef enum
{
    THROTTLE_CTL_FWD_ONLY_NEUTRAL_0,
    THROTTLE_CTL_FWD_ONLY_NEUTRAL_50,
    THROTTLE_CTL_FWD_BKWD_NEUTRAL_50,
    THROTTLE_CTL_COUNT
} throttle_ctl_t;


static void m_button_handler(uint8_t pin_no, uint8_t button_action);

static rc_radio_data_t  m_radio_data;
static bool             m_invert_y_axis=false;
static throttle_ctl_t   m_throttle_ctl=THROTTLE_CTL_DEFAULT;
static app_button_cfg_t m_buttons[] =
{
    {
        INVERT_PITCH_BUTTON_PIN,
        BUTTONS_ACTIVE_STATE,
        BUTTON_PULL,
        m_button_handler
    },
    {
        THROT_CTL_BUTTON_PIN,
        BUTTONS_ACTIVE_STATE,
        BUTTON_PULL,
        m_button_handler
    },
    {
        BIND_RESET_BUTTON_PIN,
        BUTTONS_ACTIVE_STATE,
        BUTTON_PULL,
        m_button_handler
    }
};


static void m_joystick_handler(uint8_t l_x,
                                   uint8_t l_y,
                                   uint8_t r_x,
                                   uint8_t r_y)
{
    NRF_LOG_INFO("-----Raw joystick data-----\r\n");
    NRF_LOG_INFO("Left X:\t%d\r\n",  l_x);
    NRF_LOG_INFO("Left Y:\t%d\r\n",  l_y);
    NRF_LOG_INFO("Right X:\t%d\r\n", r_x);
    NRF_LOG_INFO("Right Y:\t%d\r\n", r_y);

    m_radio_data.yaw  = l_x;
    m_radio_data.roll = r_x;

    if (!m_invert_y_axis)
    {
        m_radio_data.pitch = r_y;
    }
    else
    {
        m_radio_data.pitch = (JOYSTICK_MAX_VALUE - r_y);
    }

    switch (m_throttle_ctl)
    {
    case THROTTLE_CTL_FWD_ONLY_NEUTRAL_0:
        if (THROTTLE_SAFETY_MARGIN <= l_y)
        {
            m_radio_data.throttle = l_y;
        }
        else
        {
            m_radio_data.throttle = 0;
        }
        break;
    case THROTTLE_CTL_FWD_ONLY_NEUTRAL_50:
        if ((NEUTRAL_50_JOYSTICK_VALUE + THROTTLE_SAFETY_MARGIN) <= l_y)
        {
            // Convert from [50, 100] to [0, 100].
            m_radio_data.throttle = pam(l_y,
                                            NEUTRAL_50_JOYSTICK_VALUE,
                                            JOYSTICK_MAX_VALUE);
        }
        else
        {
            m_radio_data.throttle = 0;
        }
        break;
    case THROTTLE_CTL_FWD_BKWD_NEUTRAL_50:
        if (((NEUTRAL_50_JOYSTICK_VALUE + THROTTLE_SAFETY_MARGIN) > l_y) &&
               ((NEUTRAL_50_JOYSTICK_VALUE - THROTTLE_SAFETY_MARGIN) < l_y))
        {
            m_radio_data.throttle = NEUTRAL_50_JOYSTICK_VALUE;
        }
        else
        {
            m_radio_data.throttle = l_y;
        }
        break;
    default:
        break;
    }

    NRF_LOG_INFO("-----Channel data-----\r\n");
    NRF_LOG_INFO("Yaw:\t\t%d\r\n",    m_radio_data.yaw);
    NRF_LOG_INFO("Throttle:\t%d\r\n", m_radio_data.throttle);
    NRF_LOG_INFO("Roll:\t%d\r\n",     m_radio_data.roll);
    NRF_LOG_INFO("Pitch:\t%d\r\n",    m_radio_data.pitch);

    APP_ERROR_CHECK(rc_radio_data_set(&m_radio_data));
}


static void m_rc_radio_handler(rc_radio_event_t event,
                                    const void * const p_context)
{
    switch (event)
    {
    case RC_RADIO_EVENT_BINDING:
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
    case RC_RADIO_EVENT_DATA_SENT:
        NRF_LOG_INFO("Data sent.\r\n");
        break;
    default:
        break;
    };
}


static void m_button_handler(uint8_t pin_no, uint8_t button_action)
{
    NRF_LOG_INFO("Button %d action %d.\r\n", pin_no, button_action);

    if (button_action)
    {
        switch (pin_no)
        {
        case INVERT_PITCH_BUTTON_PIN:
            m_invert_y_axis = !m_invert_y_axis;

            if (m_invert_y_axis)
            {
                nrf_gpio_pin_clear(INVERTED_PITCH_LED_PIN);
            }
            else
            {
                nrf_gpio_pin_set(INVERTED_PITCH_LED_PIN);
            }
            break;
        case THROT_CTL_BUTTON_PIN:
            m_throttle_ctl = ((m_throttle_ctl + 1) % THROTTLE_CTL_COUNT);
            if (THROTTLE_CTL_DEFAULT != m_throttle_ctl)
            {
                nrf_gpio_pin_clear(THROT_CTL_CHANGED_LED_PIN);
            }
            else
            {
                nrf_gpio_pin_set(THROT_CTL_CHANGED_LED_PIN);
            }
            NRF_LOG_INFO("Throttle ctl mode set to: %d\r\n", m_throttle_ctl);
            break;
        case BIND_RESET_BUTTON_PIN:
            rc_radio_disable();
            rc_radio_enable();
            break;
        default:
            break;
        }
    }
}


static void m_gpio_init(void)
{
    nrf_gpio_cfg_output(BOUND_LED_PIN);
    nrf_gpio_cfg_output(INVERTED_PITCH_LED_PIN);
    nrf_gpio_cfg_output(THROT_CTL_CHANGED_LED_PIN);
    nrf_gpio_pin_set(BOUND_LED_PIN);
    nrf_gpio_pin_set(INVERTED_PITCH_LED_PIN);
    nrf_gpio_pin_set(THROT_CTL_CHANGED_LED_PIN);
}


static void lfclk_start(void)
{
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (0 == NRF_CLOCK->EVENTS_LFCLKSTARTED)
    {
    }
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}


int main(void)
{
    uint32_t err_code;

    NRF_POWER->DCDCEN = true;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    m_gpio_init();

    // The app_timer module requires an LFCLK source.
    lfclk_start();
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_button_init(m_buttons,
                                   (sizeof(m_buttons) / sizeof(m_buttons[0])),
                                   APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

    memset((uint8_t*)&m_radio_data, 0, sizeof(m_radio_data));

    err_code = rc_radio_transmitter_init(RADIO_TIMER_INSTANCE,
                                             RADIO_UPDATE_RATE_HZ,
                                             RC_RADIO_TRANSMITTER_CHANNEL_A,
                                             m_rc_radio_handler);
    APP_ERROR_CHECK(err_code);

    err_code = rc_radio_enable();
    APP_ERROR_CHECK(err_code);

    err_code = joystick_init(JOYSTICK_TIMER_INSTANCE,
                                 JOYSTICK_UPDATE_RATE_HZ,
                                 m_joystick_handler,
                                 LEFT_X_JS_PIN,
                                 LEFT_Y_JS_PIN,
                                 RIGHT_X_JS_PIN,
                                 RIGHT_Y_JS_PIN);
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        if (false == NRF_LOG_PROCESS())
        {
            __WFE();
        }
    }
}
