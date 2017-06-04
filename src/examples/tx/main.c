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


#define RADIO_TIMER_INSTANCE    (0UL)
#define JOYSTICK_TIMER_INSTANCE (1UL)

#define RADIO_UPDATE_RATE_HZ    (100UL)
#define JOYSTICK_UPDATE_RATE_HZ (50UL)

#define BOUND_LED_PIN           (17UL)
#define INVERTED_Y_AXIS_LED_PIN (20UL)
#define LEFT_X_JS_PIN           (JOYSTICK_PIN_1) // P0.3
#define LEFT_Y_JS_PIN           (JOYSTICK_PIN_2) // P0.4
#define RIGHT_X_JS_PIN          (JOYSTICK_PIN_4) // P0.28
#define RIGHT_Y_JS_PIN          (JOYSTICK_PIN_5) // P0.29


static void m_button_handler(uint8_t pin_no, uint8_t button_action);

static rc_radio_data_t  m_radio_data;
static bool             m_invert_y_axis=false;
static app_button_cfg_t m_buttons[] =
{
    {
        BUTTON_1,
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
    NRF_LOG_INFO("-----\r\n");
    NRF_LOG_INFO("%d\r\n", l_x);
    NRF_LOG_INFO("%d\r\n", l_y);
    NRF_LOG_INFO("%d\r\n", r_x);
    NRF_LOG_INFO("%d\r\n", r_y);

    m_radio_data.throttle = l_y;
    m_radio_data.yaw      = l_x;
    m_radio_data.roll     = r_x;

    if (!m_invert_y_axis)
    {
        m_radio_data.pitch = r_y;
    }
    else
    {
        m_radio_data.pitch = (JOYSTICK_MAX_VALUE - r_y);
    }

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
        m_invert_y_axis = !m_invert_y_axis;

        if (m_invert_y_axis)
        {
            nrf_gpio_pin_clear(INVERTED_Y_AXIS_LED_PIN);
        }
        else
        {
            nrf_gpio_pin_set(INVERTED_Y_AXIS_LED_PIN);
        }
    }
}


static void m_gpio_init(void)
{
    uint32_t err_code;

    nrf_gpio_cfg_output(BOUND_LED_PIN);
    nrf_gpio_cfg_output(INVERTED_Y_AXIS_LED_PIN);
    nrf_gpio_pin_set(BOUND_LED_PIN);
    nrf_gpio_pin_set(INVERTED_Y_AXIS_LED_PIN);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_button_init(m_buttons,
                                   (sizeof(m_buttons) / sizeof(m_buttons[0])),
                                   APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
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

    lfclk_start();
    m_gpio_init();

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
