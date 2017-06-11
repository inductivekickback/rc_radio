#include "nrf_saadc.h"
#include "nrf_drv_saadc.h"
#include "nrf_ppi.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "joystick.h"


#ifndef INVERT_L_X_AXIS
#define INVERT_L_X_AXIS 0
#endif

#ifndef INVERT_L_Y_AXIS
#define INVERT_L_Y_AXIS 0
#endif

#ifndef INVERT_R_X_AXIS
#define INVERT_R_X_AXIS 0
#endif

#ifndef INVERT_R_Y_AXIS
#define INVERT_R_Y_AXIS 0
#endif


#define JOYSTICK_MAX_CHANNELS     (4UL)

#define SAINSMART_MIN_VALUE        (0UL)
#define SAINSMART_MAX_VALUE        (3300UL)
#define SAINSMART_NEUTRAL_VALUE    (1620UL)


// Simple function for mapping a value betwen [0, 100] to the
// range [JOYSTICK_MIN_VALUE, JOYSTICK_MAX_VALUE].
#define PAM(x) (((x) - SAINSMART_MIN_VALUE) * 100 / (SAINSMART_MAX_VALUE - SAINSMART_MIN_VALUE))


static nrf_saadc_value_t        m_buffer_pool[2][JOYSTICK_MAX_CHANNELS];
static nrf_ppi_channel_t        m_ppi_channel;
static nrf_drv_timer_t          m_timer;
static joystick_event_handler_t m_handler;
static uint8_t                  m_enabled_channels;
static bool                     m_l_x_enabled=false;
static bool                     m_l_y_enabled=false;
static bool                     m_r_x_enabled=false;
static bool                     m_r_y_enabled=false;


static void m_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    // Required but not used.
}


static uint32_t m_saadc_sampling_event_init(uint32_t sampling_interval_ms)
{
    ret_code_t err_code;
    uint32_t   timer_compare_event_addr;
    uint32_t   saadc_sample_task_addr;

    err_code = nrf_drv_ppi_init();
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width              = NRF_TIMER_BIT_WIDTH_32;

    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, m_timer_handler);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, sampling_interval_ms);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                       NRF_TIMER_CC_CHANNEL0);
    saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    // Setup PPI channel so that timer compare event is triggering sample
    // task in SAADC.
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    return err_code;
}


static void m_saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        uint32_t   i = 0;
        uint32_t   l_x, l_y, r_x, r_y;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, m_enabled_channels);
        APP_ERROR_CHECK(err_code);

        if (m_l_x_enabled)
        {
            l_x = p_event->data.done.p_buffer[i];
#if INVERT_L_X_AXIS
            if (l_x >= SAINSMART_MAX_VALUE)
            {
                l_x = 0;
            }
            else
            {
                l_x = PAM(SAINSMART_MAX_VALUE - l_x);
            }
#else
        	l_x = PAM(l_x);
#endif
        	i++;
        }
        else
        {
        	l_x = JOYSTICK_INVALID_VALUE;
        }

        if (m_l_y_enabled)
        {
        	l_y = p_event->data.done.p_buffer[i];
#if INVERT_L_Y_AXIS
            if (l_y >= SAINSMART_MAX_VALUE)
            {
                l_y = 0;
            }
            else
            {
                l_y = PAM(SAINSMART_MAX_VALUE - l_y);
            }
#else
            l_y = PAM(l_y);
#endif
        	i++;
        }
        else
        {
        	l_y = JOYSTICK_INVALID_VALUE;
        }

        if (m_r_x_enabled)
        {
            r_x = p_event->data.done.p_buffer[i];
#if INVERT_R_X_AXIS
            if (r_x >= SAINSMART_MAX_VALUE)
            {
                r_x = 0;
            }
            else
            {
                r_x = PAM(SAINSMART_MAX_VALUE - r_x);
            }
#else
            r_x = PAM(r_x);
#endif
            i++;
        }
        else
        {
            r_x = JOYSTICK_INVALID_VALUE;
        }

        if (m_r_y_enabled)
        {
            r_y = p_event->data.done.p_buffer[i];
#if INVERT_R_Y_AXIS
            if (r_y >= SAINSMART_MAX_VALUE)
            {
                r_y = 0;
            }
            else
            {
                r_y = PAM(SAINSMART_MAX_VALUE - r_y);
            }
#else
            r_y = PAM(r_y);
#endif
            i++;
        }
        else
        {
            r_y = JOYSTICK_INVALID_VALUE;
        }

        m_handler(l_x, l_y, r_x, r_y);
    }
}


uint32_t joystick_init(uint8_t timer_instance_index,
	                       uint8_t update_rate_hz,
	                       joystick_event_handler_t joystick_event_handler,
	                       joystick_pin_t l_x_axis_pin,
	                       joystick_pin_t l_y_axis_pin,
	                       joystick_pin_t r_x_axis_pin,
	                       joystick_pin_t r_y_axis_pin)
{
    ret_code_t err_code;

    if (NULL == joystick_event_handler)
    {
    	return NRF_ERROR_INVALID_PARAM;
    }

    m_handler = joystick_event_handler;

    nrf_drv_saadc_config_t saadc_config = {
        .resolution         = NRF_SAADC_RESOLUTION_12BIT,     \
        .oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,  \
        .interrupt_priority = 7,                              \
        .low_power_mode     = false                           \
    };

    err_code = nrf_drv_saadc_init(&saadc_config, m_saadc_callback);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    m_enabled_channels = 0;

    if (JOYSTICK_PIN_NOT_USED != l_x_axis_pin)
    {
    	nrf_saadc_channel_config_t channel_config =
    	    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(l_x_axis_pin);

   	    err_code = nrf_drv_saadc_channel_init(m_enabled_channels, &channel_config);
        if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }
   	    m_enabled_channels++;
   	    m_l_x_enabled = true;
    }

    if (JOYSTICK_PIN_NOT_USED != l_y_axis_pin)
    {
    	nrf_saadc_channel_config_t channel_config =
    	    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(l_y_axis_pin);

   	    err_code = nrf_drv_saadc_channel_init(m_enabled_channels, &channel_config);
   	    if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }
   	    m_enabled_channels++;
   	    m_l_y_enabled = true;
    }

    if (JOYSTICK_PIN_NOT_USED != r_x_axis_pin)
    {
    	nrf_saadc_channel_config_t channel_config =
    	    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(r_x_axis_pin);

   	    err_code = nrf_drv_saadc_channel_init(m_enabled_channels, &channel_config);
   	    if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }
   	    m_enabled_channels++;
   	    m_r_x_enabled = true;
    }

    if (JOYSTICK_PIN_NOT_USED != r_y_axis_pin)
    {
    	nrf_saadc_channel_config_t channel_config =
    	    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(r_y_axis_pin);

   	    err_code = nrf_drv_saadc_channel_init(m_enabled_channels, &channel_config);
   	    if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }
   	    m_enabled_channels++;
   	    m_r_y_enabled = true;
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], m_enabled_channels);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], m_enabled_channels);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

	switch (timer_instance_index)
    {
    case 0:
        m_timer.p_reg            = NRF_TIMER0;
        m_timer.instance_id      = TIMER0_INSTANCE_INDEX;
        m_timer.cc_channel_count = NRF_TIMER_CC_CHANNEL_COUNT(0);
        break;
    case 1:
        m_timer.p_reg            = NRF_TIMER1;
        m_timer.instance_id      = TIMER1_INSTANCE_INDEX;
        m_timer.cc_channel_count = NRF_TIMER_CC_CHANNEL_COUNT(1);
        break;
    case 2:
        m_timer.p_reg            = NRF_TIMER2;
        m_timer.instance_id      = TIMER2_INSTANCE_INDEX;
        m_timer.cc_channel_count = NRF_TIMER_CC_CHANNEL_COUNT(2);
        break;
    case 3:
        m_timer.p_reg            = NRF_TIMER3;
        m_timer.instance_id      = TIMER3_INSTANCE_INDEX;
        m_timer.cc_channel_count = NRF_TIMER_CC_CHANNEL_COUNT(3);
        break;
    case 4:
        m_timer.p_reg            = NRF_TIMER4;
        m_timer.instance_id      = TIMER4_INSTANCE_INDEX;
        m_timer.cc_channel_count = NRF_TIMER_CC_CHANNEL_COUNT(4);
        break;
    default:
        return NRF_ERROR_INVALID_PARAM;
    }

    err_code = m_saadc_sampling_event_init(1000 / update_rate_hz);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}
