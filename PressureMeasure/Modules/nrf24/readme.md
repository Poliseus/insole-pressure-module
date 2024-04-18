# NRF24L01 Driver for STM32

OOP written driver of NRF24L01

## Depenedencies
- SPI Manager - simple bringe over SPI implementation in STM32 

## Example of use

```c
	SpiManager rManager(&hspi1, SPI_MODE_0, NRF_CSN_GPIO_Port, NRF_CSN_Pin);

    NrfSettings_t settings;
    settings.addrWidth = NRF_ADDRESS_WIDTH;
    settings.crcScheme = NRF_CRC_SCHEME;
    settings.channel = NRF_CHANNEL;
    settings.payoladLength = NRF_PAYLOAD_LENGTH;
    settings.datarate = NRF_BITRATE;
    settings.disableShockBurstChannels = NRF_DISABLE_SHOCKBURST_CHANNELS;
    settings.txPower = nRF24_TXPWR_6dBm;
    settings.pipe = nRF24_PIPE0;
#if (RADIO_RX_SINGLE)
    uint8_t nRF24_addr[] = NRF_ADDRESS_RX;
    settings.operationalMode = nRF24_MODE_RX;
#endif
#if (RADIO_TX_SINGLE)
    uint8_t nRF24_addr[] = NRF_ADDRESS_TX;
    settings.operationalMode = nRF24_MODE_TX;
    settings.pipe = nRF24_PIPETX;
#endif
    settings.address = nRF24_addr;

    Nrf24L01 radio(&settings, &rManager, NRF_CE_GPIO_Port, NRF_CE_Pin);


#if (RADIO_RX_SINGLE)
     uint8_t nRF24_payload[32];
     // Pipe number
     nRF24_RXResult pipe;

     radio.enable();
     uint8_t payload_length;
     while (1) {
    	 if (flag_IRQ_nrf == 1) {	// HW interrupt from IRQ pin
    		 flag_IRQ_nrf = 0;
             // Get a payload from the transceiver
             pipe = radio.receivePayload(nRF24_payload, &payload_length);
             radio.clearIRQFlags();

             // process of payload
         }
     }
#endif
#if (RADIO_TX_SINGLE)
    // obratin some data
     uint8_t dataTx[16];
     while(1) {
		radio.transmitPayload(dataTx, 16);	// 16 is payload length
		// some delay
	}
#endif

```