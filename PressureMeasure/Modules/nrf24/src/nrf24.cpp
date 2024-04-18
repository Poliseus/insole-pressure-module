// Functions to manage the nRF24L01+ transceiver

#include "nrf24.h"


Nrf24L01::Nrf24L01(NrfSettings_t *settings, SpiManager *spi_manager,
				GPIO_TypeDef *port_ce, uint16_t pin_ce){
	_spiManager = spi_manager;
	_ce_port = port_ce;
	_ce_pin = pin_ce;
	_settings = settings;

	PIN_LOW(_ce_port, _ce_pin);

	Delay_ms(100);	// power-on delay

	uint8_t shot = 0;
	_init_state = 0;
	do {
		_init_state = this->doCheck();
		if( _init_state == 1) {
			break;
		}
		shot++;
		Delay_ms(3);
	}while(shot<5);

	// init fail
	if (_init_state == 0) {
		return;
	}

	this->init();

	this->disableAA(settings->disableShockBurstChannels);
	this->setRFChannel(settings->channel);
	this->setDataRate(settings->datarate);
	this->setCRCScheme(settings->crcScheme);
	this->setAddrWidth(settings->addrWidth);
	this->setAddr(nRF24_PIPETX, settings->address_tx);		// set TX addr
	this->setAddr(settings->pipe_A, settings->address_rx_A);

//	if(settings->operationalMode == nRF24_MODE_RX) {
		this->setRXPipe(settings->pipe_A, nRF24_AA_OFF, settings->payoladLength);
//	}

	if (settings->pipe_B != nRF24_PIPE_None) {
		this->setAddr(settings->pipe_B, settings->address_rx_B);
		this->setRXPipe(settings->pipe_B, nRF24_AA_OFF, settings->payoladLength);
	}

	this->setTXPower(settings->txPower);

	this->setOperationalMode(settings->operationalMode);
	this->clearIRQFlags();

	this->flushRX();
	this->flushTX();

	this->disable();
	this->setPowerMode(nRF24_PWR_UP);

}

/**
 * Change operational mode:
 * For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
 * There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
 * the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
 */
void Nrf24L01::changeMode(NrfOperationalMode mode) {

	this->disable();
	this->setPowerMode(nRF24_PWR_DOWN);

//	if(mode == nRF24_MODE_RX) {		// TODO toto dat prec.
//		this->setRXPipe(_settings->pipe_A, nRF24_AA_OFF, _settings->payoladLength);
//		if (_settings->pipe_B != nRF24_PIPE_None) {
//			this->setAddr(_settings->pipe_B, _settings->address_rx_B);
//			this->setRXPipe(_settings->pipe_B, nRF24_AA_OFF, _settings->payoladLength);
//		}
//	}

	this->clearIRQFlags();
    this->setOperationalMode(mode);
//    this->enable();

    this->setPowerMode(nRF24_PWR_UP);
}

uint8_t Nrf24L01::init() {
	// Write to registers their initial values
	writeReg(nRF24_REG_CONFIG, 0x08);
	writeReg(nRF24_REG_EN_AA, 0x3F);
	writeReg(nRF24_REG_EN_RXADDR, 0x03);
	writeReg(nRF24_REG_SETUP_AW, 0x03);
	writeReg(nRF24_REG_SETUP_RETR, 0x03);
	writeReg(nRF24_REG_RF_CH, 0x02);
	writeReg(nRF24_REG_RF_SETUP, 0x0E);
	writeReg(nRF24_REG_STATUS, 0x00);
	writeReg(nRF24_REG_RX_PW_P0, 0x00);
	writeReg(nRF24_REG_RX_PW_P1, 0x00);
	writeReg(nRF24_REG_RX_PW_P2, 0x00);
	writeReg(nRF24_REG_RX_PW_P3, 0x00);
	writeReg(nRF24_REG_RX_PW_P4, 0x00);
	writeReg(nRF24_REG_RX_PW_P5, 0x00);
	writeReg(nRF24_REG_DYNPD, 0x00);
	writeReg(nRF24_REG_FEATURE, 0x00);

	// Clear the FIFO's
	this->flushRX();
	this->flushTX();

	// Clear any pending interrupt flags
	this->clearIRQFlags();

	return 0;
}

// Read a register
// input:
//   reg - number of register to read
// return: value of register
uint8_t Nrf24L01::readReg(uint8_t reg) {
	return _spiManager->SPI_ReadReg(reg & nRF24_MASK_REG_MAP);
}

// Write a new value to register
// input:
//   reg - number of register to write
//   value - value to write
void Nrf24L01::writeReg(uint8_t reg, uint8_t value) {
	PIN_LOW(_spiManager->_csn_port, _spiManager->_csn_pin);
	if (reg < nRF24_CMD_W_REGISTER) {
		// This is a register access
		_spiManager->SPI_WritedRegNoCSN((nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP)), value);

	} else {
		// This is a single byte command or future command/register
		_spiManager->SPI_ReadWriteSingle(reg);
		if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) && \
				(reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP)) {
			// Send register value
			_spiManager->SPI_ReadWriteSingle(value);
		}
	}
	PIN_HIGH(_spiManager->_csn_port, _spiManager->_csn_pin);
}


//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t Nrf24L01::check(void) {
	return _init_state;
}
// Check if the nRF24L01 present
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t Nrf24L01::doCheck(void) {
	uint8_t rxbuf[5];
	uint8_t i;
	uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;

	// Write test TX address and read TX_ADDR register
	_spiManager->SPI_WriteRegMulti(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);
	_spiManager->SPI_ReadRegMulti(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, nRF24_CMD_NOP, 5);

	// Compare buffers, return error on first mismatch
	for (i = 0; i < 5; i++) {
		if (rxbuf[i] != *ptr++) return 0;
	}

	return 1;
}

// Control transceiver power mode
// input:
//   mode - new state of power mode, one of nRF24_PWR_xx values
void Nrf24L01::setPowerMode(NrfPowerMode mode) {
	uint8_t reg;
	reg = readReg(nRF24_REG_CONFIG);
	if (mode == nRF24_PWR_UP) {
		// Set the PWR_UP bit of CONFIG register to wake the transceiver
		// It goes into Stanby-I mode with consumption about 26uA
		reg |= nRF24_CONFIG_PWR_UP;
	} else {
		// Clear the PWR_UP bit of CONFIG register to put the transceiver
		// into power down mode with consumption about 900nA
		reg &= ~nRF24_CONFIG_PWR_UP;
	}
	writeReg(nRF24_REG_CONFIG, reg);
	Delay_ms(2);
}

// Set transceiver operational mode
// input:
//   mode - operational mode, one of nRF24_MODE_xx values
void Nrf24L01::setOperationalMode(NrfOperationalMode mode) {
	uint8_t reg;
	// Configure PRIM_RX bit of the CONFIG register
	reg  = readReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PRIM_RX;
	reg |= (mode & nRF24_CONFIG_PRIM_RX);
	writeReg(nRF24_REG_CONFIG, reg);
}

// Set transceiver DynamicPayloadLength feature for all the pipes
// input:
//   mode - status, one of nRF24_DPL_ON/nRF24_DPL_OFF values
void Nrf24L01::setDynamicPayloadLength(NrfDPLStatus mode) {
	uint8_t reg;
	reg  = readReg(nRF24_REG_FEATURE);
	if(mode) {
		writeReg(nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_DPL);
		writeReg(nRF24_REG_DYNPD, 0x1F);
	} else {
		writeReg(nRF24_REG_FEATURE, reg &~ nRF24_FEATURE_EN_DPL);
		writeReg(nRF24_REG_DYNPD, 0x0);
	}
}

// Enables Payload With Ack. NB Refer to the datasheet for proper retransmit timing.
// input:
//   mode - status, 1 or 0
void Nrf24L01::setPayloadWithAck(uint8_t mode) {
	uint8_t reg;
	reg  = readReg(nRF24_REG_FEATURE);
	if(mode) {
		writeReg(nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_ACK_PAY);
	} else {
		writeReg(nRF24_REG_FEATURE, reg &~ nRF24_FEATURE_EN_ACK_PAY);
	}
}

// Configure transceiver CRC scheme
// input:
//   scheme - CRC scheme, one of nRF24_CRC_xx values
// note: transceiver will forcibly turn on the CRC in case if auto acknowledgment
//       enabled for at least one RX pipe
void Nrf24L01::setCRCScheme(NrfCrcScheme scheme) {
	uint8_t reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg  = readReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_MASK_CRC;
	reg |= (scheme & nRF24_MASK_CRC);
	writeReg(nRF24_REG_CONFIG, reg);
}

// Set frequency channel
// input:
//   channel - radio frequency channel, value from 0 to 127
// note: frequency will be (2400 + channel)MHz
// note: PLOS_CNT[7:4] bits of the OBSERVER_TX register will be reset
void Nrf24L01::setRFChannel(uint8_t channel) {
	writeReg(nRF24_REG_RF_CH, channel);
}

// Set automatic retransmission parameters
// input:
//   ard - auto retransmit delay, one of nRF24_ARD_xx values
//   arc - count of auto retransmits, value form 0 to 15
// note: zero arc value means that the automatic retransmission disabled
void Nrf24L01::nRF24_SetAutoRetr(uint8_t ard, uint8_t arc) {
	// Set auto retransmit settings (SETUP_RETR register)
	writeReg(nRF24_REG_SETUP_RETR, (uint8_t)((ard << 4) | (arc & nRF24_MASK_RETR_ARC)));
}

// Set of address widths
// input:
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
void Nrf24L01::setAddrWidth(uint8_t addr_width) {
	writeReg(nRF24_REG_SETUP_AW, addr_width - 2);
}

// Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes
void Nrf24L01::setAddr(uint8_t pipe, const uint8_t *addr) {
	uint8_t addr_width;

	// RX_ADDR_Px register
	switch (pipe) {
		case nRF24_PIPETX:
		case nRF24_PIPE0:
		case nRF24_PIPE1:
			// Get address width
			addr_width = readReg(nRF24_REG_SETUP_AW) + 1;
			// Write address in reverse order (LSByte first)
			addr += addr_width;
			PIN_LOW(_spiManager->_csn_port, _spiManager->_csn_pin);
			_spiManager->SPI_ReadWriteSingle(nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
			do {
				_spiManager->SPI_ReadWriteSingle(*addr--);
			} while (addr_width--);
			PIN_HIGH(_spiManager->_csn_port, _spiManager->_csn_pin);
			break;
		case nRF24_PIPE2:
		case nRF24_PIPE3:
		case nRF24_PIPE4:
		case nRF24_PIPE5:
			// Write address LSBbyte (only first byte from the addr buffer)
			writeReg(nRF24_ADDR_REGS[pipe], *addr);
			break;
		default:
			// Incorrect pipe number -> do nothing
			break;
	}
}

// Configure RF output power in TX mode
// input:
//   tx_pwr - RF output power, one of nRF24_TXPWR_xx values
void Nrf24L01::setTXPower(NrfTxPower tx_pwr) {
	uint8_t reg;
	// Configure RF_PWR[2:1] bits of the RF_SETUP register
	reg  = readReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_RF_PWR;
	reg |= tx_pwr;
	writeReg(nRF24_REG_RF_SETUP, reg);
}

// Configure transceiver data rate
// input:
//   data_rate - data rate, one of nRF24_DR_xx values
void Nrf24L01::setDataRate(NrfDataRate data_rate) {
	uint8_t reg;
	// Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
	reg  = readReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_DATARATE;
	reg |= data_rate;
	writeReg(nRF24_REG_RF_SETUP, reg);
}

// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void Nrf24L01::setRXPipe(NrfPipe pipe, NrfAcknowledgementState aa_state, uint8_t payload_len) {
	uint8_t reg;
	// Enable the specified pipe (EN_RXADDR register)
	reg = (readReg(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
	writeReg(nRF24_REG_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register)
	writeReg(nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW);

	// Set auto acknowledgment for a specified pipe (EN_AA register)
	reg = readReg(nRF24_REG_EN_AA);
	if (aa_state == nRF24_AA_ON) {
		reg |=  (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	writeReg(nRF24_REG_EN_AA, reg);
}

// Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
void Nrf24L01::closePipe(uint8_t pipe) {
	uint8_t reg;
	reg  = readReg(nRF24_REG_EN_RXADDR);
	reg &= ~(1 << pipe);
	reg &= nRF24_MASK_EN_RX;
	writeReg(nRF24_REG_EN_RXADDR, reg);
}

// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
void Nrf24L01::enableAA(uint8_t pipe) {
	uint8_t reg;
	// Set bit in EN_AA register
	reg  = readReg(nRF24_REG_EN_AA);
	reg |= (1 << pipe);
	writeReg(nRF24_REG_EN_AA, reg);
}

// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX pipes
// input:
//   pipe - number of the RX pipe, value from 0 to 5, any other value will disable AA for all RX pipes
void Nrf24L01::disableAA(uint8_t pipe) {
	uint8_t reg;
	if (pipe > 5) {
		// Disable Auto-ACK for ALL pipes
		writeReg(nRF24_REG_EN_AA, 0x00);
	} else {
		// Clear bit in the EN_AA register
		reg  = readReg(nRF24_REG_EN_AA);
		reg &= ~(1 << pipe);
		writeReg(nRF24_REG_EN_AA, reg);
	}
}

// Get value of the STATUS register
// return: value of STATUS register
uint8_t Nrf24L01::getStatus(void) {
	return readReg(nRF24_REG_STATUS);
}

// Get pending IRQ flags
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
uint8_t Nrf24L01::getIRQFlags(void) {
	return (readReg(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
}

// Get status of the RX FIFO
// return: one of the nRF24_STATUS_RXFIFO_xx values
uint8_t Nrf24L01::getStatus_RXFIFO(void) {
	return (readReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
}

// Get status of the TX FIFO
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
uint8_t Nrf24L01::getStatus_TXFIFO(void) {
	return ((readReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
}

// Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
uint8_t Nrf24L01::getRXSource(void) {
	return ((readReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
}

// Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
uint8_t Nrf24L01::getRetransmitCounters(void) {
	return (readReg(nRF24_REG_OBSERVE_TX));
}

// Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
void Nrf24L01::resetPLOS(void) {
	uint8_t reg;
	// The PLOS counter is reset after write to RF_CH register
	reg = readReg(nRF24_REG_RF_CH);
	writeReg(nRF24_REG_RF_CH, reg);
}

// Flush the TX FIFO
void Nrf24L01::flushTX(void) {
	writeReg(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

// Flush the RX FIFO
void Nrf24L01::flushRX(void) {
	writeReg(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

// Clear any pending IRQ flags
void Nrf24L01::clearIRQFlags(void) {
	uint8_t reg;
	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
	reg  = readReg(nRF24_REG_STATUS);
	reg |= nRF24_MASK_STATUS_IRQ;
	writeReg(nRF24_REG_STATUS, reg);
}

// Write TX payload
// input:
//   pBuf - pointer to the buffer with payload data
//   length - payload length in bytes
void Nrf24L01::writePayload(uint8_t *pBuf, uint8_t length) {
	_spiManager->SPI_WriteRegMulti(nRF24_CMD_W_TX_PAYLOAD, pBuf, length);

}

uint8_t Nrf24L01::getRxDplPayloadWidth() {
	return _spiManager->SPI_ReadReg(nRF24_CMD_R_RX_PL_WID);
}

nRF24_RXResult Nrf24L01::readPayloadGeneric(uint8_t *pBuf, uint8_t *length, uint8_t dynamicPayloadLength) {
	uint8_t pipe;
	// Extract a payload pipe number from the STATUS register
	pipe = (readReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1;

	// RX FIFO empty?
	if (pipe < 6) {
		// Get payload length
		if(dynamicPayloadLength) {
			*length = getRxDplPayloadWidth();
			if(*length>32) { //broken packet
				*length = 0;
				this->flushRX();
			}
		} else {
			*length = readReg(nRF24_RX_PW_PIPE[pipe]);
		}

		// Read a payload from the RX FIFO
		if (*length) {
			_spiManager->SPI_ReadRegMulti(nRF24_CMD_R_RX_PAYLOAD, pBuf, nRF24_CMD_NOP, *length);
		}

		return ((nRF24_RXResult)pipe);
	}

	// The RX FIFO is empty
	*length = 0;

	return nRF24_RX_EMPTY;
}

// Read top level payload available in the RX FIFO
// input:
//   pBuf - pointer to the buffer to store a payload data
//   length - pointer to variable to store a payload length
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
nRF24_RXResult Nrf24L01::receivePayload(uint8_t *pBuf, uint8_t *length) {
	return readPayloadGeneric(pBuf, length, 0);
}

nRF24_RXResult Nrf24L01::receivePayloadDpl(uint8_t *pBuf, uint8_t *length) {
	return readPayloadGeneric(pBuf, length, 1);
}

uint8_t Nrf24L01::getFeatures() {
    return readReg(nRF24_REG_FEATURE);
}

void Nrf24L01::activateFeatures() {
	_spiManager->SPI_WritedReg(nRF24_CMD_ACTIVATE, 0x73);
}

void Nrf24L01::transmitPayloadAck(nRF24_RXResult pipe, char *payload, uint8_t length) {
	PIN_LOW(_spiManager->_csn_port, _spiManager->_csn_pin);			// TODO prehodit do spiManagera
	_spiManager->SPI_ReadWriteSingle(nRF24_CMD_W_ACK_PAYLOAD | pipe);
	while (length--) {
		_spiManager->SPI_ReadWriteSingle((uint8_t) *payload++);
	}
	PIN_HIGH(_spiManager->_csn_port, _spiManager->_csn_pin);

}

nRF24_TXResult Nrf24L01::transmitPayload(uint8_t *pBuf, uint8_t length) {
    volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
    uint8_t status;

    // Deassert the CE pin (in case if it still high)
    PIN_LOW(_ce_port, _ce_pin);

    // Transfer a data from the specified buffer to the TX FIFO
    writePayload(pBuf, length);

    // Start a transmission by asserting CE pin (must be held at least 10us)
    PIN_HIGH(_ce_port, _ce_pin);

    do {
    	if(_spiManager->hasIrqCallback()) {
    		status = _spiManager->irqCallback();
    	} else {
    	    // Poll the transceiver status register until one of the following flags will be set:
    	    //   TX_DS  - means the packet has been transmitted
    	    //   MAX_RT - means the maximum number of TX retransmits happened
    		status = getStatus(); // SW pooling
    	}
        if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
        	// It will not work, when hasIrqClearCallback is not defined
        	if(_spiManager->hasIrqClearCallback()) {
        		_spiManager->irqCallbackClear();
        	}
            break;
        }

    } while (wait--);

    // Deassert the CE pin (Standby-II --> Standby-I)
   PIN_LOW(_ce_port, _ce_pin);

    if (!wait) {
        // Timeout
        return nRF24_TX_TIMEOUT;
    }

    // Clear pending IRQ flags
    clearIRQFlags();

    if (status & nRF24_FLAG_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
        return nRF24_TX_MAXRT;
    }

    if (status & nRF24_FLAG_TX_DS) {
        // Successful transmission
        return nRF24_TX_SUCCESS;
    }

    // Some banana happens, a payload remains in the TX FIFO, flush it
    flushTX();

    return nRF24_TX_ERROR;
}

void Nrf24L01::enable(void)
{
	PIN_HIGH(_ce_port, _ce_pin);
}


void Nrf24L01::disable(void)
{
	PIN_LOW(_ce_port, _ce_pin);
}
