#ifndef RC_RADIO_H
#define RC_RADIO_H

#include "stdint.h"


// Any valid RADIO_TXPOWER_* values can be used.
#ifdef NRF52840_XXAA
#include "nrf52840_bitfields.h"
#define RC_RADIO_TX_POWER         (RADIO_TXPOWER_TXPOWER_Pos8dBm)
#else
#include "nrf52_bitfields.h"
#define RC_RADIO_TX_POWER         (RADIO_TXPOWER_TXPOWER_Pos4dBm)
#endif
#define RC_RADIO_BINDING_TX_POWER (RADIO_TXPOWER_TXPOWER_Neg12dBm)

// This is the number of consecutive missed packets before the
// receiver concludes that the transmitter has gone away.
#define RC_RADIO_MISSED_PACKET_TOLERANCE (50UL)


/**
 * The following events are delivered to the application via the
 * rc_radio_event_handler_t callback.
 * 
 * NOTE: The RC_RADIO_EVENT_BOUND event is delivered along with a pointer to a
 *       rc_radio_bind_info_t struct.
 *
 * NOTE: The RC_RADIO_EVENT_DATA_RECEIVED event is delivered along with a
 *       pointer to a rc_radio_data_t struct.
 */
typedef enum
{
    RC_RADIO_EVENT_BINDING,
    RC_RADIO_EVENT_BOUND,          // p_context is set to *rc_radio_bind_info_t
    RC_RADIO_EVENT_DATA_SENT,      // Only delivered to transmitter
    RC_RADIO_EVENT_DATA_RECEIVED,  // p_context is set to *rc_radio_data_t
    RC_RADIO_EVENT_PACKET_DROPPED, // Only delivered to receiver
    RC_RADIO_EVENT_COUNT
} rc_radio_event_t;


/**
 * Each "transmitter channel" uses a different address and a unique channel
 * map. Seperate transmitter channels can be used to allow multiple
 * transmitters to coexist.
 */
typedef enum
{
    RC_RADIO_TRANSMITTER_CHANNEL_A,
    RC_RADIO_TRANSMITTER_CHANNEL_B,
    RC_RADIO_TRANSMITTER_CHANNEL_C,
    RC_RADIO_TRANSMITTER_CHANNEL_D,
    RC_RADIO_TRANSMITTER_CHANNEL_E,
    RC_RADIO_TRANSMITTER_CHANNEL_COUNT
} rc_radio_transmitter_channel_t;


/**
 * This payload can be customized as long as sizeof(rc_radio_data_t) does not
 * exceed NRF_ESB_MAX_PAYLOAD_LENGTH.
 */
typedef struct
{
    uint8_t throttle;
    int8_t  pitch;
    int8_t  roll;
    int8_t  yaw;
} rc_radio_data_t;


typedef struct
{
    rc_radio_transmitter_channel_t transmitter_channel;
    uint16_t                       transmit_rate_hz;
} rc_radio_bind_info_t;


typedef void (*rc_radio_event_handler_t)(rc_radio_event_t event,
                                             const void * const p_context);


/**
 * The callback can be NULL if the app doesn't care. The timer instance
 * is used to select a free timer peripheral (e.g. 0 is converted to TIMER0).
 * The transmit_rate_hz paramter must be in the range [10, 500].
 */
uint32_t rc_radio_transmitter_init(uint8_t timer_instance_index,
                                       uint16_t transmit_rate_hz,
                                       rc_radio_transmitter_channel_t channel,
                                       rc_radio_event_handler_t callback);

/**
 * The callback can not be NULL for a receiver (otherwise no data would be
 * delivered to the app). The update rate and channel will be set by the
 * transmiter during binding. The timer instance is used to select a free
 * timer peripheral (e.g. 0 is converted to TIMER0).
 */
uint32_t rc_radio_receiver_init(uint8_t timer_instance_index,
                                    rc_radio_event_handler_t callback);

/**
 * This function must be called to initiate the binding procedure. Note that
 * rc_radio_data_set must also be called for the transmitter before binding
 * can begin.
 */
uint32_t rc_radio_enable(void);

/**
 * This function must be called to set the data before the transmitter
 * will initiate the binding procedure. Returns NRF_ERROR_INVALID_PARAM if
 * rc_radio_receiver_init was used to init the module. Data will be copied
 * to an internal buffer.
 */
uint32_t rc_radio_data_set(const rc_radio_data_t * const p_data);

/**
 * Shuts down the radio immediately.
 */
void rc_radio_disable(void);

#endif
