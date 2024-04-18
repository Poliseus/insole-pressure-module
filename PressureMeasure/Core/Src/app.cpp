/*
 * app.cpp
 *
 *  Created on: Apr 17, 2024
 *      Author: jakub
 */
#include "app.h"
#include "dataframe.h"
#include "spiManager.h"
#include "nrf24.h"


volatile uint8_t adc_ready = 0;
uint16_t pData[16];

extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim16;
extern SPI_HandleTypeDef hspi1;


volatile uint8_t start_measure = 0;
volatile uint8_t flag_ADC_done;
volatile uint8_t flag_IRQ_nrf = 0;

inline void Toggle_LED() {
	HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
}

void callbackTimer(TIM_HandleTypeDef *tim){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&pData[3], 8);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	 // HAL_GPIO_TogglePin(LD3_GPIO_Port);
	  adc_ready = 1;

}

uint8_t getNrfIrg(void) {
	return flag_IRQ_nrf == 0 ? 0x0E : 0x2E;
}

void clearNrfIrg(void) {
	flag_IRQ_nrf = 0;
}

void appReceiver(){
	pData[0]=0xAABB;

	SpiManager rManager(&hspi1, SPI_MODE_0, SPI1_CS_GPIO_Port, SPI1_CS_Pin);

	NrfSettings_t settings;
		settings.addrWidth = NRF_ADDRESS_WIDTH;
		settings.crcScheme = NRF_CRC_SCHEME;
		settings.channel = NRF_CHANNEL;
		settings.payoladLength = NRF_PAYLOAD_LENGTH;
		settings.datarate = NRF_BITRATE;
		settings.disableShockBurstChannels = NRF_DISABLE_SHOCKBURST_CHANNELS;
		settings.txPower = nRF24_TXPWR_6dBm;
		settings.operationalMode = nRF24_MODE_RX;
	    uint8_t nRF24_addr_rx[] = NRF_ADDRESS_RX_0;
	    settings.address_rx_A = nRF24_addr_rx;

		Nrf24L01 radio(&settings, &rManager, SPI1_CE_GPIO_Port, SPI1_CE_Pin);

		radio.enable();
		while (1){
				if(adc_ready == 1){
					uint32_t cas = HAL_GetTick();
					pData[1] = cas & 0xFFFF;
					pData[2] = (cas >> 16) & 0xFFFF;
					HAL_UART_Transmit(&huart2, (uint8_t*)pData, 20+2, 100);
					radio.transmitPayload((uint8_t*)pData, settings.payoladLength);
					adc_ready = 0;
				}
			}

	rManager.setIrqCallbacks(getNrfIrg, clearNrfIrg);
}
void appTransmitter(){
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_TIM_RegisterCallback(&htim16, HAL_TIM_PERIOD_ELAPSED_CB_ID, callbackTimer);
	HAL_TIM_Base_Start_IT(&htim16);

	SpiManager rManager(&hspi1, SPI_MODE_0, SPI1_CS_GPIO_Port, SPI1_CS_Pin);

	NrfSettings_t settings;
	settings.addrWidth = NRF_ADDRESS_WIDTH;
	settings.crcScheme = NRF_CRC_SCHEME;
	settings.channel = NRF_CHANNEL;
	settings.payoladLength = NRF_PAYLOAD_LENGTH;
	settings.datarate = NRF_BITRATE;
	settings.disableShockBurstChannels = NRF_DISABLE_SHOCKBURST_CHANNELS;
	settings.txPower = nRF24_TXPWR_6dBm;
	settings.operationalMode = nRF24_MODE_TX;
    uint8_t nRF24_addr_tx[] = NRF_ADDRESS_TX;
    settings.address_tx = nRF24_addr_tx;

	Nrf24L01 radio(&settings, &rManager, SPI1_CE_GPIO_Port, SPI1_CE_Pin);

	if(!radio.check()){
		while(1){
			Toggle_LED();
			Delay_ms(1000);
	    }
	}

	while (1){
		if(adc_ready == 1){
			radio.transmitPayload((uint8_t*)pData, settings.payoladLength);
			adc_ready = 0;
		}
	}
}

