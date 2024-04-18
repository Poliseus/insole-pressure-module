# Software bridge to SPI interface

SPI bridge for STM32 MCUs

## Motivation

To avoid write same basic SPI function as is read, write, ... is every MCU project with SPI interface.
If you create OOP written driver for specific SPI sensor or device, you can use this class as brige to SPI.

## Usage

First of all, you have to prepare SPI interface in STMCubeMX software.

```c
SpiManager(SPI_HandleTypeDef *hspi, SPI_Mode_t spi_mode, GPIO_TypeDef *port_cs, uint16_t pin_cs);
```
- hspi - pointer to initialized SPI interface
- spi_mode - you can switch SPI mode during communications. Allowed values are: SPI_MODE_0, SPI_MODE_1,SPI_MODE_2, SPI_MODE_3
- port_cs GPIO port for CSN pin of SPI interface
- pin_cs GPIO pin for CSN pin of SPI interface

### Example of isage in some spicific driver

```c
void Driver::readMultipleReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	PIN_LOW(_spiManager->_csn_port, _spiManager->_csn_pin);
	_spiManager->SPI_ReadRegMulti(reg, pBuf, nRF24_CMD_NOP, count);
	PIN_HIGH(_spiManager->_csn_port, _spiManager->_csn_pin);
}
```

## Public Programming interface

- ``void SetSpiMode(uint8_t mode);``	Set SPI mode. Can be used anytime.
- ``uint8_t SPI_ReadWriteSingle(uint8_t data);``    Write and read 1B from SPI
- ``uint8_t SPI_ReadReg(uint8_t reg);`` Send to SPI 1B (reg) and read the answer from SPI
- ``void SPI_WritedReg(uint8_t reg, uint8_t value);`` Write 1B to SPI
- ``void SPI_ReadRegMulti(uint8_t reg, uint8_t* dataIn, uint8_t dummy, uint32_t count);`` Write 1B to SPI (reg) and read count bytes of response
- ``void SPI_WriteRegMulti(uint8_t reg, uint8_t* toWrite, uint8_t count);`` Write 1B (reg) to SPI and next write count bytes (toWrite)
- ``void SPI_ReadMulti(uint8_t* dataIn, uint8_t dummy, uint32_t count);`` Read count bytes from SPI
- ``void SPI_WriteMulti(uint8_t* dataOut, uint32_t count);``  Write count bytes to SPI
- ``void SPI_SendMulti(uint8_t* dataOut, uint8_t* dataIn, uint32_t count);`` Write count bytes to SPI, in dataOut array is response from SPI
- ``void setIrqCallbacks(uint8_t (*function)(void), void (*clear)(void));`` Set callbacks that can be used with sensor/module IRQ feature

