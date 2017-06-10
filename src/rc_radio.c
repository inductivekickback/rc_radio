#include "string.h"

#include "nrf_esb.h"
#include "app_error.h"
#include "nrf_clock.h"
#include "nrf_drv_timer.h"
#include "nrf_gpio.h"

#include "rc_radio.h"


#define ENABLE_GPIO_DBG    (1UL)
#define GPIO_DBG_PIN_1     (26UL)
#define GPIO_DBG_PIN_2     (27UL)

#define BITRATE            (1000000UL)
#define ADDR_LEN           (5UL)
#define CHANNEL_MAP_LEN    (10UL)
#define BIND_CHANNEL       (10UL)
#define DATA_BUFF_COUNT    (2UL)
#define MIN_TX_RATE_HZ     (10UL)
#define MAX_TX_RATE_HZ     (500UL)
#define TIMER_ISR_PRIORITY (1UL)

#define ADDR_BITS          (ADDR_LEN * 8UL)
#define DATA_BITS          (sizeof(rc_radio_data_t) * 8UL)
#define PREAMBLE_BITS      (8UL)
#define PCF_BITS           (11UL) /* Packet Control Field from ESB */
#define CRC_BITS           (16UL)
#define PKT_OVERHEAD_BITS  (PREAMBLE_BITS + PCF_BITS + CRC_BITS)

#define CEILING(n,d)       (((n) + (d) - 1) / (d))
#define LEN_US(x_bits)     CEILING((x_bits * 10000000), (BITRATE*10))

#define PKT_LEN_US         LEN_US(PKT_OVERHEAD_BITS + ADDR_BITS + DATA_BITS)
#define OVERHEAD_US        (300UL) /* Empirical, includes f.e. radio ramp up. */
#define RX_WIDENING_US     (100UL)
#define RX_SAFETY_US       (100UL)


typedef enum
{
    RC_RADIO_STATE_DISABLED,
    RC_RADIO_STATE_ENABLED,
    RC_RADIO_STATE_BINDING,
    RC_RADIO_STATE_STARTED,
    RC_RADIO_STATE_COUNT
} rc_radio_state_t;


static const uint8_t
BIND_ADDRESS[ADDR_LEN] = {0xAA, 0xBB, 0x55, 0xAA, 0x5A};

static const uint8_t
CHANNEL_MAP[RC_RADIO_TRANSMITTER_CHANNEL_COUNT][CHANNEL_MAP_LEN] = {
    {0,  32,  62, 92,  22, 52, 82, 12, 42, 72},
    {2,  34,  64, 94,  24, 54, 84, 14, 44, 74},
    {4,  36,  66, 96,  26, 56, 86, 16, 46, 76},
    {6,  38,  68, 98,  28, 58, 88, 18, 48, 78},
    {8,  40,  70, 100, 30, 60, 90, 20, 50, 80}
};

static const uint8_t
ADDRESSES[RC_RADIO_TRANSMITTER_CHANNEL_COUNT][ADDR_LEN] = {
    {0xAA, 0xBB, 0xD5, 0x95, 0x55},
    {0xAA, 0xBB, 0x6A, 0x4A, 0xAA},
    {0xAA, 0xBB, 0xB5, 0x52, 0x5A},
    {0xAA, 0xBB, 0xAD, 0xA9, 0xA5},
    {0xAA, 0xBB, 0x56, 0x54, 0x2A}
};

static const uint8_t
BINDING_ACK_PAYLOAD[] = "RC_RADIO";


static nrf_esb_payload_t              m_rx_payload;
static nrf_esb_payload_t              m_tx_payload;

static nrf_esb_mode_t                 m_mode;
static rc_radio_event_handler_t       m_callback;
static nrf_drv_timer_t                m_timer;
static bool                           m_hfclk_was_running;
static uint8_t                        m_tx_data_index;
static rc_radio_data_t                m_tx_data[DATA_BUFF_COUNT];

static volatile rc_radio_state_t      m_state=RC_RADIO_STATE_DISABLED;
static volatile rc_radio_bind_info_t  m_bind_info;
static volatile uint32_t              m_missed_packets;
static volatile uint8_t               m_channel_index;


static uint32_t m_radio_start(void);


static inline uint32_t m_channel_lookup(void)
{
    return CHANNEL_MAP[m_bind_info.transmitter_channel][m_channel_index];
}


static inline void m_channel_increment(void)
{
    m_channel_index = ((m_channel_index + 1) % CHANNEL_MAP_LEN);
}


static inline uint32_t m_timer_interval_calc(void)
{
    return (1000000UL / m_bind_info.transmit_rate_hz);
}


static inline uint32_t m_write_bind_info_pl(void)
{
    m_tx_payload.length = sizeof(rc_radio_bind_info_t);
    m_tx_payload.noack  = false;
    memcpy(m_tx_payload.data,
               (uint8_t*)&m_bind_info,
               sizeof(rc_radio_bind_info_t));

    return nrf_esb_write_payload(&m_tx_payload);
}


static inline uint8_t m_write_ack_pl(void)
{
    // Preload a particular ACK payload so the transmitter will know
    // that it's binding with a valid receiver.
    m_tx_payload.length = sizeof(BINDING_ACK_PAYLOAD);
    memcpy(m_tx_payload.data,
               BINDING_ACK_PAYLOAD,
               sizeof(BINDING_ACK_PAYLOAD));

    return nrf_esb_write_payload(&m_tx_payload);
}


static void m_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
#if ENABLE_GPIO_DBG
    nrf_gpio_pin_set(GPIO_DBG_PIN_1);
#endif

    switch (event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:
        if (NRF_ESB_MODE_PTX == m_mode)
        {
            // Writing a payload starts the transmission immediately.
            if (RC_RADIO_STATE_BINDING == m_state)
            {
                uint32_t err_code = m_write_bind_info_pl();

                if (NRF_ERROR_NO_MEM == err_code)
                {
                    nrf_esb_flush_tx();
                }
                else
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            else
            {
                m_tx_payload.length = sizeof(rc_radio_data_t);
                m_tx_payload.noack  = true;
                memcpy(&m_tx_payload.data[0],
                           (uint8_t*)&m_tx_data[m_tx_data_index],
                           sizeof(rc_radio_data_t));
                APP_ERROR_CHECK(nrf_esb_write_payload(&m_tx_payload));
            }
        }
        else
        {
            APP_ERROR_CHECK(nrf_esb_start_rx());
        }
        break;
    case NRF_TIMER_EVENT_COMPARE2:
#if ENABLE_GPIO_DBG
        nrf_gpio_pin_clear(GPIO_DBG_PIN_1);
        nrf_gpio_pin_set(GPIO_DBG_PIN_1);
        nrf_gpio_pin_clear(GPIO_DBG_PIN_1);
        nrf_gpio_pin_set(GPIO_DBG_PIN_1);
#endif

        APP_ERROR_CHECK(nrf_esb_start_rx());
        break;
    case NRF_TIMER_EVENT_COMPARE1:
#if ENABLE_GPIO_DBG
        nrf_gpio_pin_clear(GPIO_DBG_PIN_1);
        nrf_gpio_pin_set(GPIO_DBG_PIN_1);
#endif

        m_missed_packets++;

        if (RC_RADIO_MISSED_PACKET_TOLERANCE > m_missed_packets)
        {
            if (1 == m_missed_packets)
            {
                uint32_t ticks;

                ticks = nrf_drv_timer_capture_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
                nrf_timer_cc_write(m_timer.p_reg,
                                       NRF_TIMER_CC_CHANNEL0,
                                       (ticks - RX_SAFETY_US));

                ticks = nrf_drv_timer_capture_get(&m_timer, NRF_TIMER_CC_CHANNEL1);
                nrf_timer_cc_write(m_timer.p_reg,
                                       NRF_TIMER_CC_CHANNEL1,
                                       (ticks - RX_SAFETY_US));
            }

            m_channel_increment();

            APP_ERROR_CHECK(nrf_esb_stop_rx());
            APP_ERROR_CHECK(nrf_esb_set_rf_channel(m_channel_lookup()));

            m_callback(RC_RADIO_EVENT_PACKET_DROPPED, NULL);
        }
        else
        {
            // The transmitter has gone away.
            nrf_drv_timer_disable(&m_timer);
            APP_ERROR_CHECK(nrf_esb_stop_rx());

            m_callback(RC_RADIO_EVENT_PACKET_DROPPED, NULL);

            // NOTE: Calling m_esb_init seems like a good idea. Unfortunately,
            //       that sometimes causes a situation where the receiver does
            //       not appear to be properly enabled.
            APP_ERROR_CHECK(nrf_esb_set_base_address_0(BIND_ADDRESS));
            APP_ERROR_CHECK(nrf_esb_set_prefixes(&BIND_ADDRESS[ADDR_LEN-1],1));
            APP_ERROR_CHECK(nrf_esb_set_rf_channel(BIND_CHANNEL));
            APP_ERROR_CHECK(m_write_ack_pl());
            APP_ERROR_CHECK(nrf_esb_start_rx());

            m_state = RC_RADIO_STATE_BINDING;
            m_callback(RC_RADIO_EVENT_BINDING, NULL);
        }
        break;
    default:
        break;
    }

#if ENABLE_GPIO_DBG
    nrf_gpio_pin_clear(GPIO_DBG_PIN_1);
#endif
}


static inline bool m_reciver_ackd(void)
{
    if (sizeof(BINDING_ACK_PAYLOAD) != m_rx_payload.length)
    {
        return false;
    }

    return (0 == memcmp(m_rx_payload.data,
                            BINDING_ACK_PAYLOAD,
                            sizeof(BINDING_ACK_PAYLOAD)));
}


static inline void m_bind_info_received(void)
{
    uint32_t             interval_us;
    uint32_t             ticks;
    const uint8_t        *addr;
    rc_radio_bind_info_t *p_info;

    if (sizeof(rc_radio_bind_info_t) != m_rx_payload.length)
    {
        m_write_ack_pl();
        return;
    }

    p_info = (rc_radio_bind_info_t*)m_rx_payload.data;

    if ((RC_RADIO_TRANSMITTER_CHANNEL_COUNT <= p_info->transmitter_channel) ||
            (MIN_TX_RATE_HZ > p_info->transmit_rate_hz) ||
            (MAX_TX_RATE_HZ < p_info->transmit_rate_hz))
    {
        m_write_ack_pl();
        return;
    }

    m_bind_info.transmitter_channel = p_info->transmitter_channel;
    m_bind_info.transmit_rate_hz    = p_info->transmit_rate_hz;
    m_channel_index                 = 0;
    m_missed_packets                = 0;

    // CC0 fires when it's time to put the radio into receiver mode.
    // If a packet is received then the timer is cleared.
    //
    // If no packet is received, CC1 moves the receiver to the next channel.

    interval_us = m_timer_interval_calc();
    
    ticks = (interval_us + RX_SAFETY_US);
    nrf_drv_timer_extended_compare(&m_timer,
                                       NRF_TIMER_CC_CHANNEL1,
                                       ticks,
                                       NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK,
                                       true);

    ticks = (interval_us -
                 OVERHEAD_US - 
                 PKT_LEN_US -
                 RX_WIDENING_US);
    nrf_drv_timer_compare(&m_timer,
                              NRF_TIMER_CC_CHANNEL0,
                              ticks,
                              true);

    // Clear the events in case this is a re-binding event.
    nrf_timer_event_clear(m_timer.p_reg, NRF_TIMER_EVENT_COMPARE0);
    nrf_timer_event_clear(m_timer.p_reg, NRF_TIMER_EVENT_COMPARE1);

    nrf_drv_timer_enable(&m_timer);

    while (NRF_SUCCESS != nrf_esb_stop_rx())
    {
        // The radio still needs to send an ACK payload. The ESB library
        // can't be disabled until this completes. In the meantime,
        // nrf_esb_stop_rx will return NRF_ESB_ERROR_NOT_IN_RX_MODE.
        // 
        // NOTE: The NRF_ESB_EVENT_TX_SUCCESS event won't be received until
        //       another packet is received from the transmitter.
    }

    addr = ADDRESSES[m_bind_info.transmitter_channel];

    APP_ERROR_CHECK(nrf_esb_set_base_address_0(addr));
    APP_ERROR_CHECK(nrf_esb_set_prefixes(&addr[ADDR_LEN - 1], 1));
    APP_ERROR_CHECK(nrf_esb_set_rf_channel(m_channel_lookup()));

    m_state = RC_RADIO_STATE_STARTED;
    m_callback(RC_RADIO_EVENT_BOUND, (void*)&m_bind_info);
}


static inline void m_data_received(void)
{
    if (sizeof(rc_radio_data_t) == m_rx_payload.length)
    {
        // Reset the timer to keep it in sync.
        nrf_drv_timer_clear(&m_timer);

        m_channel_increment();

        APP_ERROR_CHECK(nrf_esb_stop_rx());
        APP_ERROR_CHECK(nrf_esb_set_rf_channel(m_channel_lookup()));

        if (m_missed_packets)
        {
            uint32_t ticks;

            ticks = nrf_drv_timer_capture_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
            nrf_timer_cc_write(m_timer.p_reg,
                                   NRF_TIMER_CC_CHANNEL0,
                                   (ticks + RX_SAFETY_US));

            ticks = nrf_drv_timer_capture_get(&m_timer, NRF_TIMER_CC_CHANNEL1);
            nrf_timer_cc_write(m_timer.p_reg,
                                   NRF_TIMER_CC_CHANNEL1,
                                   (ticks + RX_SAFETY_US));

            m_missed_packets = 0;
        }

        m_callback(RC_RADIO_EVENT_DATA_RECEIVED, m_rx_payload.data);
    }
}


static void m_nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
#if ENABLE_GPIO_DBG
    nrf_gpio_pin_set(GPIO_DBG_PIN_2);
#endif

    switch (p_event->evt_id)
    {
    case NRF_ESB_EVENT_TX_SUCCESS:
        // NOTE: The NRF_ESB_EVENT_TX_SUCCESS event will also be delivered for
        //       the receiver when it gets a packet after ACK'ing the bind
        //       packet.
        if (NRF_ESB_MODE_PTX == m_mode)
        {
            // NOTE: The NRF_ESB_EVENT_TX_SUCCESS event is delivered before
            //       the NRF_ESB_EVENT_RX_RECEIVED event when binding.
            if (RC_RADIO_STATE_STARTED == m_state)
            {
                m_channel_increment();
                nrf_esb_set_rf_channel(m_channel_lookup());

                if (NULL != m_callback)
                {
                    m_callback(RC_RADIO_EVENT_DATA_SENT, NULL);
                }
            }
        }
        break;
    case NRF_ESB_EVENT_TX_FAILED:
        nrf_esb_flush_tx();
        break;
    case NRF_ESB_EVENT_RX_RECEIVED:
        APP_ERROR_CHECK(nrf_esb_read_rx_payload(&m_rx_payload));
        if (NRF_ESB_MODE_PRX == m_mode)
        {
            if (RC_RADIO_STATE_BINDING == m_state)
            {
                m_bind_info_received();
            }
            else
            {
                m_data_received();
            }
        }
        else if (m_reciver_ackd())
        {
            // A valid response to the bind packet was received so it's time
            // to move to the data address and channels.
            const uint8_t * addr = ADDRESSES[m_bind_info.transmitter_channel];

            m_channel_index = 0;

            APP_ERROR_CHECK(nrf_esb_set_base_address_0(addr));
            APP_ERROR_CHECK(nrf_esb_set_prefixes(&addr[ADDR_LEN - 1], 1));
            APP_ERROR_CHECK(nrf_esb_set_tx_power(RC_RADIO_TX_POWER));
            APP_ERROR_CHECK(nrf_esb_set_rf_channel(m_channel_lookup()));

            m_state = RC_RADIO_STATE_STARTED;
            if (NULL != m_callback)
            {
                m_callback(RC_RADIO_EVENT_BOUND, (void*)&m_bind_info);
            }
        }
        break;
    }

#if ENABLE_GPIO_DBG
    nrf_gpio_pin_clear(GPIO_DBG_PIN_2);
#endif
}


static uint32_t m_esb_init(void)
{
    uint32_t err_code;
 
    nrf_esb_config_t nrf_esb_config   = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length     = sizeof(rc_radio_data_t);
    nrf_esb_config.bitrate            = NRF_ESB_BITRATE_1MBPS;
    nrf_esb_config.mode               = m_mode;
    nrf_esb_config.event_handler      = m_nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack = true;
    nrf_esb_config.tx_output_power    = RC_RADIO_BINDING_TX_POWER;
    nrf_esb_config.retransmit_count   = 0;
    nrf_esb_config.radio_irq_priority = 0;
    nrf_esb_config.event_irq_priority = TIMER_ISR_PRIORITY;

    err_code = nrf_esb_init(&nrf_esb_config);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    err_code = nrf_esb_set_base_address_0(BIND_ADDRESS);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    err_code = nrf_esb_set_prefixes(&BIND_ADDRESS[ADDR_LEN - 1], 1);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    err_code = nrf_esb_set_rf_channel(BIND_CHANNEL);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


static void m_clocks_start(void)
{
    nrf_clock_hfclk_t clk_src = nrf_clock_hf_src_get();

    m_hfclk_was_running = (NRF_CLOCK_HFCLK_HIGH_ACCURACY==clk_src);

    if (!m_hfclk_was_running)
    {
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
        NRF_CLOCK->TASKS_HFCLKSTART    = 1;

        while (0 == NRF_CLOCK->EVENTS_HFCLKSTARTED)
        {
        };
    }
}


static void m_clocks_stop(void)
{
    if (!m_hfclk_was_running)
    {
        NRF_CLOCK->TASKS_HFCLKSTOP = 1;

        while (NRF_CLOCK_HFCLK_HIGH_ACCURACY == nrf_clock_hf_src_get())
        {
        };
    }
}


static uint32_t m_radio_start(void)
{
    uint32_t err_code;

    m_state = RC_RADIO_STATE_BINDING;

    if (NRF_ESB_MODE_PRX == m_mode)
    {
        err_code = m_esb_init();
        if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }

        err_code = m_write_ack_pl();
        if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }

        err_code = nrf_esb_start_rx();
        if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }
    }
    else
    {
        uint32_t delay_us = m_timer_interval_calc();

        err_code = m_esb_init();
        if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }

        err_code = m_write_bind_info_pl();
        if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }

        nrf_drv_timer_extended_compare(&m_timer,
                                           NRF_TIMER_CC_CHANNEL0,
                                           delay_us,
                                           NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                           true);
        nrf_drv_timer_enable(&m_timer);
    }

    if (NULL != m_callback)
    {
        m_callback(RC_RADIO_EVENT_BINDING, NULL);
    }

    return NRF_SUCCESS;
}


static uint32_t m_rc_radio_init(uint8_t timer_instance_index)
{
    uint32_t               err_code;
    nrf_drv_timer_config_t timer_cfg;

    timer_cfg.mode               = TIMER_MODE_MODE_Timer;
    timer_cfg.frequency          = NRF_TIMER_FREQ_1MHz;
    timer_cfg.bit_width          = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.interrupt_priority = TIMER_ISR_PRIORITY;

    // Only pipe 0 is used in this library.
    m_tx_payload.pipe = 0;

    if (RC_RADIO_STATE_DISABLED != m_state)
    {
        return NRF_ERROR_INVALID_STATE;
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

#if ENABLE_GPIO_DBG
    nrf_gpio_cfg_output(GPIO_DBG_PIN_1);
    nrf_gpio_cfg_output(GPIO_DBG_PIN_2);
    nrf_gpio_pin_clear(GPIO_DBG_PIN_1);
    nrf_gpio_pin_clear(GPIO_DBG_PIN_2);
#endif

    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, m_timer_handler);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t rc_radio_transmitter_init(uint8_t timer_instance_index,
                                       uint16_t transmit_rate_hz,
                                       rc_radio_transmitter_channel_t channel,
                                       rc_radio_event_handler_t callback)
{
    if ((MIN_TX_RATE_HZ>transmit_rate_hz) || (MAX_TX_RATE_HZ<transmit_rate_hz))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (RC_RADIO_TRANSMITTER_CHANNEL_COUNT <= channel)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_mode          = NRF_ESB_MODE_PTX;
    m_callback      = callback;
    m_tx_data_index = DATA_BUFF_COUNT;

    m_bind_info.transmitter_channel = channel;
    m_bind_info.transmit_rate_hz    = transmit_rate_hz;

    return m_rc_radio_init(timer_instance_index);
}


uint32_t rc_radio_receiver_init(uint8_t timer_instance_index,
                                    rc_radio_event_handler_t callback)
{
    if (NULL == callback)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
 
    m_mode     = NRF_ESB_MODE_PRX;
    m_callback = callback;

    return m_rc_radio_init(timer_instance_index);
}


uint32_t rc_radio_enable(void)
{
    uint32_t err_code;

    if (RC_RADIO_STATE_DISABLED != m_state)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_clocks_start();

    if (NRF_ESB_MODE_PRX == m_mode)
    {
        err_code = m_radio_start();
        if (NRF_SUCCESS != err_code)
        {
            return err_code;
        }
    }
    else
    {
        m_state = RC_RADIO_STATE_ENABLED;
    }

    return NRF_SUCCESS;
}


void rc_radio_disable(void)
{
    switch (m_state)
    {
    case RC_RADIO_STATE_DISABLED:
        return;
    case RC_RADIO_STATE_BINDING:
    case RC_RADIO_STATE_STARTED:
        nrf_drv_timer_disable(&m_timer);
        (void)nrf_esb_disable();
    case RC_RADIO_STATE_ENABLED:
        m_clocks_stop();
        break;
    default:
        break;
    }

    m_state = RC_RADIO_STATE_DISABLED;
}


uint32_t rc_radio_data_set(const rc_radio_data_t * const p_data)
{
    // NOTE: m_tx_data_index is only updated after the memcpy has completed. This
    //       is done to ensure that m_tx_data_index is always valid regardless of
    //       when this function is preempted by the timer interrupt handler.
    uint8_t index;

    if (RC_RADIO_STATE_DISABLED == m_state)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (NRF_ESB_MODE_PTX != m_mode)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    index = ((m_tx_data_index + 1) % DATA_BUFF_COUNT);

    memcpy((uint8_t*)&m_tx_data[index],
               (uint8_t*)p_data,
               sizeof(rc_radio_data_t));

    m_tx_data_index = index;

    if (RC_RADIO_STATE_ENABLED == m_state)
    {
        return m_radio_start();
    }

    return NRF_SUCCESS;
}
