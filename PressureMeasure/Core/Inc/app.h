/*
 * app.h
 *
 *  Created on: Apr 17, 2024
 *      Author: jakub
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "main.h"

#define RADIO_RX_SINGLE      0 	 // Single address receiver (1 pipe)
#define RADIO_TX_SINGLE      1	 // Single address transmitter (1 pipe)
#define RADIO_ENABLE_SYNC    1

#if (RADIO_TX_SINGLE + RADIO_RX_SINGLE) == 2
#error Select only single mode for radio
#endif

#define NRF_ADDRESS_RX_0  { 'i', 'n', 'p', 'r', 0x01 }
#define NRF_ADDRESS_TX    { 'i', 'n', 'p', 'r', 0x01 }


#define NRF_ADDRESS_WIDTH 5
#define NRF_CRC_SCHEME nRF24_CRC_2byte
#define NRF_CHANNEL	55
#define NRF_PAYLOAD_LENGTH 32
#define NRF_BITRATE nRF24_DR_2Mbps
#define NRF_DISABLE_SHOCKBURST_CHANNELS 0xFF
#define NRF_TX_POWER nRF24_TXPWR_18dBm

#ifdef __cplusplus
extern "C" {
#endif

void appReceiver();
void appTransmitter();
void callbackTimer(TIM_HandleTypeDef *tim);

#ifdef __cplusplus
}
#endif


#endif /* INC_APP_H_ */
