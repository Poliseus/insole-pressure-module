/**
 * @author  Juraj Dudak
 * @version v1.0
 * @ide     STM32CubeIDE
 * @license MIT
 * @brief   SPI manager for NRF24L01
 *	
 */

#ifndef _NRF_MANAGER_H_
#define _NRF_MANAGER_H_

#if defined(STM32F401xC) || defined(STM32F401xE)
#include "stm32f4xx_hal.h"
#endif

#if defined (STM32L432xx)
#include "stm32l4xx_hal.h"
#endif

#include "nrf24l01_defines.h"
#include <string.h>

//Hardware dependent helper functions
#define PIN_LOW(PORT,PIN)   {PORT->BRR = (uint32_t)PIN;}
#define PIN_HIGH(PORT,PIN)  {PORT->BSRR = (uint32_t)PIN;}

#define Delay_ms(ms) 		HAL_Delay(ms)

typedef enum {
	SPI_MODE_0 = 0,
	SPI_MODE_1,
	SPI_MODE_2,
	SPI_MODE_3,
}SPI_Mode_t;

#ifdef __cplusplus
 extern "C" {
#endif


class SpiManager {
private:
	SPI_HandleTypeDef *_spi;
	SPI_Mode_t _spi_mode;

	uint8_t txbuffer[33];
	uint8_t rxbuffer[33];
	volatile uint8_t irq_flag = 0;

public:
    GPIO_TypeDef *_csn_port;
    uint16_t _csn_pin;

    SpiManager(SPI_HandleTypeDef *spi, SPI_Mode_t spi_mode, GPIO_TypeDef *port_cs, uint16_t pin_cs);
	void SetSpiMode(uint8_t mode);
	SPI_HandleTypeDef* getSpi();

	void (*irqCallbackClear)();
	/**
	 * Return status from IRQ HW line. Have to be implemented in main application.
	 */
	uint8_t (*irqCallback)();

	uint8_t SPI_ReadWriteSingle(uint8_t data);
	uint8_t SPI_ReadReg(uint8_t reg);
	void SPI_WritedReg(uint8_t reg, uint8_t value);
	void SPI_WritedRegNoCSN(uint8_t reg, uint8_t value);

	void SPI_ReadRegMulti(uint8_t reg, uint8_t* dataIn, uint8_t dummy, uint32_t count);
	void SPI_WriteRegMulti(uint8_t reg, uint8_t* toWrite, uint8_t count);

	void SPI_ReadMulti(uint8_t* dataIn, uint8_t dummy, uint32_t count);
	void SPI_WriteMulti(uint8_t* dataOut, uint32_t count);
	void SPI_SendMulti(uint8_t* dataOut, uint8_t* dataIn, uint32_t count);

	void setIrqCallbacks(uint8_t (*function)(void), void (*clear)(void));

	uint8_t hasIrqCallback(void);
	uint8_t hasIrqClearCallback(void);

};



#ifdef __cplusplus
 }
#endif


#endif /* _NRF_MANAGER_H_ */
