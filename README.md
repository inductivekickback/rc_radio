The nrf_esb library in the [nRF5 SDK](http://developer.nordicsemi.com/nRF5_SDK/) provides a straightfoward path for developing proprietary protocols based on Nordic's [Enhanced ShockBurst](https://devzone.nordicsemi.com/blogs/783/intro-to-shockburstenhanced-shockburst/) link layer. In this project, the nrf_esb library is used to build a customizable remote control library.

### Features
The main features of the Remote Control Radio (rc_radio) library include:

 - A simple binding protocol where the transmitter reduces its output power, broadcasts on a known address and frequency, and waits for a reciever to ACK with a specific payload.
 - 5 different "transmitter channels", each with a unique address and channel map
 - Payloads can be customized by modifying the rc_radio_data_t struct and recompiling.
 - The transmit rate is configurable between 10 and 500 hertz.
 - The radio is disabled between events to save energy.
 - A Frequency Hopping Spread Spectrum (FHSS) mechanism is used to move the radio to a different channel after each packet is sent or received.

### Parameters
 To maximize range and robustness the following radio parameters are used:
 
 - 1mbps datarate
 - 5 address bytes
 - 2 CRC bytes
 - Only pipe 0 is used in order to utilize the better radio front-end.
 - Acknowledgements are only used when binding.

### SoC Resources
The rc_radio library uses one of the nRF52's high-speed timer peripherals. Unfortunately, the nrf_esb library cannot be controlled via the [PPI](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fppi.html) so the timer is used to generate interrupts. The timer is configured for 1MHz operation to save energy and simplify timer arithmetic. The nrf_esb library itself uses **TIMER2** by default.

The default NRF_ESB_MAX_PAYLOAD_LENGTH in nrf_esb.h is set to 32 bytes. The rc_radio_data_t payload is 4 bytes long by default. If rc_radio_data_t is modified then NRF_ESB_MAX_PAYLOAD_LENGTH may need to be increased (up to a maximum of 252 bytes). 

The nrf_esb library contains a FIFO mechanism for handling payloads and the NRF_ESB_TX_FIFO_SIZE and NRF_ESB_RX_FIFO_SIZE symbols are set to 8 by default in nrf_esb.h. The rc_radio library does not ever put more than one payload into the transmit FIFO and it processes the received payloads as they arrive so the default FIFO sizes can be reduced to save RAM if necessary.

The nrf_esb library allows for configuration of the radio and callback interrupt priorities. The rc_radio library sets the radio interrupt priority to 0 and the callback priority to 1. The rc_radio itself uses priority 1 for its timer interrupts. It is recommended to use priorities [2, 7] for the remainder of the application.

### Usage
There are unique init functions for the receiver and transmitter modes. Both init functions require an index of a high-speed timer to use (e.g. 0 for TIMER0) as well as a rc_radio_event_handler_t function pointer.

For the transmitter, the event handler might look like this:

```
void rc_radio_handler(rc_radio_event_t event, const void * const p_context)
{
  switch (event)
  {
  case RC_RADIO_EVENT_BINDING:
    NRF_LOG_INFO("Binding...\r\n");
    break;
  case RC_RADIO_EVENT_BOUND:
  {
    rc_radio_bind_info_t * p_bind_info;
    p_bind_info = (rc_radio_bind_info_t*) p_context;

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
```
The receiver's version of the event handler might look like this:
```
void rc_radio_handler(rc_radio_event_t event, const void * const p_context)
{
  uint32_t err_code;

  switch (event)
  {
  case RC_RADIO_EVENT_BINDING:
    NRF_LOG_INFO("Binding...\r\n");
    break;
  case RC_RADIO_EVENT_BOUND:
  {
    rc_radio_bind_info_t * p_bind_info;
    p_bind_info = (rc_radio_bind_info_t*) p_context;

    NRF_LOG_INFO("Bound. (%d, %d)\r\n",
                  p_bind_info->transmitter_channel,
                  p_bind_info->transmit_rate_hz);

  }
    break;
  case RC_RADIO_EVENT_DATA_RECEIVED:
  {
    rc_radio_data_t * p_rc_data;
    p_rc_data = (rc_radio_data_t*) p_context;

    // Apply the data from the payload.

    NRF_LOG_INFO("Data received.\r\n");
  }
    break;
  case RC_RADIO_EVENT_PACKET_DROPPED:
    NRF_LOG_INFO("Packet dropped.\r\n");
    break;
  default:
    break;
  };
}
```
The transmitter also decides which "transmitter channel" to use (e.g. RC_RADIO_TRANSMITTER_CHANNEL_D) as well as the transmit frequency in hertz. Note that when operating as a transmitter, the most recent payload will be reused automatically as needed; this allows the rc_radio_data_set to be called at a lower frequency than the transmit frequency.

### Operation
The transmit frequency is converted to a TRANSMIT_INTERVAL_US value. The transmitter uses this interval to trigger its timer's Capture Compare channel 0 (CC0) interrupt. The payload is written to the nrf_esb library during this interrupt to trigger the transmission. The receiver can't compute the TRANSMIT_INTERVAL_US until after it has binded to a transmitter. It then configures its timer to trigger its CC0 interrupt RX_WIDENING_US microseconds before it expects the transmitter to transmit.

![Figure 1. Transmit Interval](https://cloud.githubusercontent.com/assets/6494431/26183685/306c44fc-3b35-11e7-9f98-c0bf76848927.png)

Typically, the packet transmission is successful, the packet is received, and the nrf_esb library executes a callback in the rc_radio library that clears the timer. This keeps the timer in sync with the transmitter. The recieved payload is reported to the application immediately.

![Figure 2. Packet Received](https://cloud.githubusercontent.com/assets/6494431/26183687/323d3098-3b35-11e7-8a89-53b7f9db348c.png)

The receiver also configures its timer to trigger its CC1 interrupt RX_SAFETY_US microseconds after it expects the packet to be received. If the packet is not received then the CC1 interrupt disables the receiver, moves it to the next RF frequency, and notifies the application that a packet was dropped.

![Figure 3. Packet Missed](https://cloud.githubusercontent.com/assets/6494431/26183688/33bd8cce-3b35-11e7-90d7-b8356425945b.png)

A single packet is sent/received per RF channel before moving to the next channel.
