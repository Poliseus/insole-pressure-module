#ifndef __NRF24_H
#define __NRF24_H

#include "nrf24l01_defines.h"
#include "spiManager.h"

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x0000FFFF

// Fake address to test transceiver presence (5 bytes long)
#define nRF24_TEST_ADDR            "nRF24"

// Retransmit delay
enum {
	nRF24_ARD_NONE   = (uint8_t)0x00, // Dummy value for case when retransmission is not used
	nRF24_ARD_250us  = (uint8_t)0x00,
	nRF24_ARD_500us  = (uint8_t)0x01,
	nRF24_ARD_750us  = (uint8_t)0x02,
	nRF24_ARD_1000us = (uint8_t)0x03,
	nRF24_ARD_1250us = (uint8_t)0x04,
	nRF24_ARD_1500us = (uint8_t)0x05,
	nRF24_ARD_1750us = (uint8_t)0x06,
	nRF24_ARD_2000us = (uint8_t)0x07,
	nRF24_ARD_2250us = (uint8_t)0x08,
	nRF24_ARD_2500us = (uint8_t)0x09,
	nRF24_ARD_2750us = (uint8_t)0x0A,
	nRF24_ARD_3000us = (uint8_t)0x0B,
	nRF24_ARD_3250us = (uint8_t)0x0C,
	nRF24_ARD_3500us = (uint8_t)0x0D,
	nRF24_ARD_3750us = (uint8_t)0x0E,
	nRF24_ARD_4000us = (uint8_t)0x0F
};

// Data rate
typedef enum {
	nRF24_DR_250kbps = (uint8_t)0x20, // 250kbps data rate
	nRF24_DR_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
	nRF24_DR_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
}NrfDataRate;

// RF output power in TX mode
typedef enum {
	nRF24_TXPWR_18dBm = (uint8_t)0x00, // -18dBm
	nRF24_TXPWR_12dBm = (uint8_t)0x02, // -12dBm
	nRF24_TXPWR_6dBm  = (uint8_t)0x04, //  -6dBm
	nRF24_TXPWR_0dBm  = (uint8_t)0x06  //   0dBm
}NrfTxPower;

// CRC encoding scheme
typedef enum {
	nRF24_CRC_off   = (uint8_t)0x00, // CRC disabled
	nRF24_CRC_1byte = (uint8_t)0x08, // 1-byte CRC
	nRF24_CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
}NrfCrcScheme;

// nRF24L01 power control
typedef enum {
	nRF24_PWR_UP   = (uint8_t)0x02, // Power up
	nRF24_PWR_DOWN = (uint8_t)0x00  // Power down
}NrfPowerMode;

// Transceiver mode
typedef enum {
	nRF24_MODE_RX = (uint8_t)0x01, // PRX
	nRF24_MODE_TX = (uint8_t)0x00  // PTX
}NrfOperationalMode;

typedef enum {
	nRF24_DPL_ON = (uint8_t)0x01, // PRX
	nRF24_DPL_OFF = (uint8_t)0x00  // PTX
}NrfDPLStatus ;

// Enumeration of RX pipe addresses and TX address
typedef enum {
	nRF24_PIPE0  = (uint8_t)0x00, // pipe0
	nRF24_PIPE1  = (uint8_t)0x01, // pipe1
	nRF24_PIPE2  = (uint8_t)0x02, // pipe2
	nRF24_PIPE3  = (uint8_t)0x03, // pipe3
	nRF24_PIPE4  = (uint8_t)0x04, // pipe4
	nRF24_PIPE5  = (uint8_t)0x05, // pipe5
	nRF24_PIPETX = (uint8_t)0x06,  // TX address (not a pipe in fact)
	nRF24_PIPE_None = 0xFF

}NrfPipe;

// State of auto acknowledgment for specified pipe
typedef enum {
	nRF24_AA_OFF = (uint8_t)0x00,
	nRF24_AA_ON  = (uint8_t)0x01
}NrfAcknowledgementState;

// Status of the RX FIFO
enum {
	nRF24_STATUS_RXFIFO_DATA  = (uint8_t)0x00, // The RX FIFO contains data and available locations
	nRF24_STATUS_RXFIFO_EMPTY = (uint8_t)0x01, // The RX FIFO is empty
	nRF24_STATUS_RXFIFO_FULL  = (uint8_t)0x02, // The RX FIFO is full
	nRF24_STATUS_RXFIFO_ERROR = (uint8_t)0x03  // Impossible state: RX FIFO cannot be empty and full at the same time
};

// Status of the TX FIFO
enum {
	nRF24_STATUS_TXFIFO_DATA  = (uint8_t)0x00, // The TX FIFO contains data and available locations
	nRF24_STATUS_TXFIFO_EMPTY = (uint8_t)0x01, // The TX FIFO is empty
	nRF24_STATUS_TXFIFO_FULL  = (uint8_t)0x02, // The TX FIFO is full
	nRF24_STATUS_TXFIFO_ERROR = (uint8_t)0x03  // Impossible state: TX FIFO cannot be empty and full at the same time
};

// Result of RX FIFO reading
typedef enum {
	nRF24_RX_PIPE0  = (uint8_t)0x00, // Packet received from the PIPE#0
	nRF24_RX_PIPE1  = (uint8_t)0x01, // Packet received from the PIPE#1
	nRF24_RX_PIPE2  = (uint8_t)0x02, // Packet received from the PIPE#2
	nRF24_RX_PIPE3  = (uint8_t)0x03, // Packet received from the PIPE#3
	nRF24_RX_PIPE4  = (uint8_t)0x04, // Packet received from the PIPE#4
	nRF24_RX_PIPE5  = (uint8_t)0x05, // Packet received from the PIPE#5
	nRF24_RX_EMPTY  = (uint8_t)0xff  // The RX FIFO is empty
} nRF24_RXResult;


// Addresses of the RX_PW_P# registers
static const uint8_t nRF24_RX_PW_PIPE[6] = {
		nRF24_REG_RX_PW_P0,
		nRF24_REG_RX_PW_P1,
		nRF24_REG_RX_PW_P2,
		nRF24_REG_RX_PW_P3,
		nRF24_REG_RX_PW_P4,
		nRF24_REG_RX_PW_P5
};

// Addresses of the address registers
static const uint8_t nRF24_ADDR_REGS[7] = {
		nRF24_REG_RX_ADDR_P0,
		nRF24_REG_RX_ADDR_P1,
		nRF24_REG_RX_ADDR_P2,
		nRF24_REG_RX_ADDR_P3,
		nRF24_REG_RX_ADDR_P4,
		nRF24_REG_RX_ADDR_P5,
		nRF24_REG_TX_ADDR
};

// Result of packet transmission
typedef enum {
    nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
    nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
    nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
    nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;


typedef struct {
	uint8_t disableShockBurstChannels;
	uint8_t channel;
	uint8_t payoladLength;
	NrfDataRate datarate;
	NrfCrcScheme crcScheme;
	uint8_t addrWidth;
	uint8_t *address_tx;
	uint8_t *address_rx_A;
	uint8_t *address_rx_B;
	NrfTxPower txPower;
	NrfOperationalMode operationalMode;
	NrfPipe pipe_A;
	NrfPipe pipe_B;
}NrfSettings_t;


#ifdef __cplusplus
extern "C" {
#endif


class Nrf24L01 {

protected:
	NRF24L01_Conig_t nrf_config;
	SpiManager *_spiManager;

    GPIO_TypeDef *_ce_port;
    uint16_t _ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */

    uint8_t _init_state;

public:

    Nrf24L01(NrfSettings_t *settings, SpiManager *nrf_manager, GPIO_TypeDef *port_ce, uint16_t pin_ce);
    uint8_t check(void);
    void setPowerMode(NrfPowerMode mode);
    void setOperationalMode(NrfOperationalMode mode);
    void setDynamicPayloadLength(NrfDPLStatus mode);
    void setPayloadWithAck(uint8_t mode);
    void setCRCScheme(NrfCrcScheme scheme);
    void setRFChannel(uint8_t channel);
    void nRF24_SetAutoRetr(uint8_t ard, uint8_t arc);
    void setAddrWidth(uint8_t addr_width);
    void setAddr(uint8_t pipe, const uint8_t *addr);
    void setTXPower(NrfTxPower tx_pwr);
    void setDataRate(NrfDataRate data_rate);
    void setRXPipe(NrfPipe pipe, NrfAcknowledgementState aa_state, uint8_t payload_len);
    void closePipe(uint8_t pipe);
    void enableAA(uint8_t pipe);
    void disableAA(uint8_t pipe);
    uint8_t getStatus(void);
    uint8_t getIRQFlags(void);
    uint8_t getStatus_RXFIFO(void);
    uint8_t getStatus_TXFIFO(void);
    uint8_t getRXSource(void);
    uint8_t getRetransmitCounters(void);
    uint8_t getFeatures(void);
    void resetPLOS(void);
    void flushTX(void);
    void flushRX(void);
    void clearIRQFlags(void);
    void activateFeatures(void);
    void writePayload(uint8_t *pBuf, uint8_t length);
    void transmitPayloadAck(nRF24_RXResult pipe, char *payload, uint8_t length);
    uint8_t retRxDplPayloadWidth();
    uint8_t init();
    void changeMode(NrfOperationalMode mode);

    nRF24_RXResult receivePayload(uint8_t *pBuf, uint8_t *length);
    nRF24_RXResult receivePayloadDpl(uint8_t *pBuf, uint8_t *length);
    nRF24_TXResult transmitPayload(uint8_t *pBuf, uint8_t length);

    void enable(void);
    void disable(void);

private:
    uint8_t _buffer[8];
    NrfSettings_t *_settings;

    uint8_t readReg(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t value);

    nRF24_RXResult readPayloadGeneric(uint8_t *pBuf, uint8_t *length, uint8_t dpl);
    uint8_t getRxDplPayloadWidth();
    uint8_t doCheck(void);

};


#ifdef __cplusplus
}
#endif


#endif // __NRF24_H
