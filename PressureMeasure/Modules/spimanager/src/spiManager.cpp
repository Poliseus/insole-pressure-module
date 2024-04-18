#include "spiManager.h"


SpiManager::SpiManager(SPI_HandleTypeDef* spi, SPI_Mode_t spi_mode, GPIO_TypeDef *port_cs, uint16_t pin_cs){
	_spi = spi;
	_spi_mode = spi_mode;
	_csn_port = port_cs;
	_csn_pin = pin_cs;
	irqCallback = NULL;
	irqCallbackClear = NULL;
	PIN_HIGH(_csn_port, _csn_pin);
	this->SetSpiMode(_spi_mode);
}

/**
 * Set the callback functions to these events (pointer to functions):
 * @param irqCallback: request for IRQ status from HW
 * @param irqCallbackClear: clear the IRC flag after IRQ occurs.
 */
void SpiManager::setIrqCallbacks(uint8_t (*function)(void), void (*clear)(void)) {
	irqCallback = function;
	irqCallbackClear = clear;
}


uint8_t SpiManager::hasIrqCallback(void) {
	return this->irqCallback != NULL;
}

uint8_t SpiManager::hasIrqClearCallback(void) {
	return this->irqCallbackClear != NULL;
}

/**
 * Send and receive 1Byte.
 * There is NO CSpin change!
 */
uint8_t SpiManager::SPI_ReadWriteSingle(uint8_t data) {

    if(HAL_SPI_TransmitReceive(_spi, &data, rxbuffer, 1, 20) != HAL_OK) {
        return 0xFF;
    };
    return rxbuffer[0];
}

uint8_t SpiManager::SPI_ReadReg(uint8_t reg) {

	txbuffer[0] = reg;
	txbuffer[1] = 0xFF;
	PIN_LOW(this->_csn_port, this->_csn_pin);
	HAL_SPI_TransmitReceive(_spi, txbuffer, rxbuffer, 2, 10);
	PIN_HIGH(this->_csn_port, this->_csn_pin);
    return rxbuffer[1];
}

void SpiManager::SPI_WritedReg(uint8_t reg, uint8_t value) {
	txbuffer[0] = reg;
	txbuffer[1] = value;
	PIN_LOW(this->_csn_port, this->_csn_pin);
	HAL_SPI_TransmitReceive(_spi, txbuffer, rxbuffer, 2, 10);
	PIN_HIGH(this->_csn_port, this->_csn_pin);
}

void SpiManager::SPI_WritedRegNoCSN(uint8_t reg, uint8_t value) {
	txbuffer[0] = reg;
	txbuffer[1] = value;

	HAL_SPI_TransmitReceive(_spi, txbuffer, rxbuffer, 2, 10);

}

void SpiManager::SPI_ReadRegMulti(uint8_t reg, uint8_t* dataIn, uint8_t dummy, uint32_t count) {
	txbuffer[0] = reg;
	memset(txbuffer+1, dummy, count);

	PIN_LOW(this->_csn_port, this->_csn_pin);
	HAL_SPI_TransmitReceive(_spi, txbuffer, rxbuffer, count+1, count);
	PIN_HIGH(this->_csn_port, this->_csn_pin);

	memcpy(dataIn, rxbuffer+1, count);
}

void SpiManager::SPI_WriteRegMulti(uint8_t reg, uint8_t* toWrite, uint8_t count) {
	txbuffer[0] = reg;
	memcpy(txbuffer+1, toWrite, count);

	PIN_LOW(this->_csn_port, this->_csn_pin);
	HAL_SPI_TransmitReceive(_spi, txbuffer, rxbuffer, count+1, count);
	PIN_HIGH(this->_csn_port, this->_csn_pin);
}

void SpiManager::SPI_ReadMulti(uint8_t* dataIn, uint8_t dummy, uint32_t count) {
	for(uint8_t i=0;i<count;i++){
		txbuffer[i] = dummy;
	}
	PIN_LOW(this->_csn_port, this->_csn_pin);
	HAL_SPI_TransmitReceive(_spi, txbuffer, dataIn, count, count);
	PIN_HIGH(this->_csn_port, this->_csn_pin);
}

void SpiManager::SPI_WriteMulti(uint8_t* dataOut, uint32_t count) {
	PIN_LOW(this->_csn_port, this->_csn_pin);
	HAL_SPI_TransmitReceive(_spi, dataOut, rxbuffer, count, count);
	PIN_HIGH(this->_csn_port, this->_csn_pin);
}


void SpiManager::SPI_SendMulti(uint8_t* dataOut, uint8_t* dataIn, uint32_t count) {
	PIN_LOW(this->_csn_port, this->_csn_pin);
	HAL_SPI_TransmitReceive(_spi, dataOut, dataIn, count, count);
	PIN_HIGH(this->_csn_port, this->_csn_pin);
}

SPI_HandleTypeDef* SpiManager::getSpi(){
	return _spi;
}

void SpiManager::SetSpiMode(uint8_t mode){

	switch(mode){
	case 0:
		_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
		_spi->Init.CLKPhase = SPI_PHASE_1EDGE;
		break;
	case 1:
		_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
		_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
		break;
	case 2:
		_spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
		_spi->Init.CLKPhase = SPI_PHASE_1EDGE;
		break;
	case 3:
		_spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
		_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
		break;
	}

	uint32_t spiSettings = _spi->Instance->CR1 & 0xFFFC;
	spiSettings = spiSettings | (_spi->Init.CLKPolarity<<1) | _spi->Init.CLKPhase;
	WRITE_REG(_spi->Instance->CR1, spiSettings);
}
