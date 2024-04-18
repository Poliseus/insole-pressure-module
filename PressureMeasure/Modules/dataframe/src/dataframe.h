/*
 * @file dataframe.h
 * @brief DataFrame interface
 *
 * @date: 31. 12. 2022
 * @author: juraj
 */

#ifndef __DATAFRAME_H__
#define __DATAFRAME_H__

#include "stdint.h"

/** Character for preamble - 1st byte, if is needed */
#define HEADER_CHAR1 0xCC
/** Character for preamble - 2nd byte, if is needed */
#define HEADER_CHAR2 0xDD

/**
 * Definition of DatFrame type.
 * Can be simple frame, or frame with some preamble.
 */
typedef enum {
  /** Dataframe with plain content. There is no header of preamble in frame*/
  TYPE_PLAIN,
  /** Dataframe contain first byte as a preamble of packet */
  TYPE_HEADER_1B,
  /** Dataframe contain first 2 bytes as a preamble of packet */
  TYPE_HEADER_2B,
} DataframeType_t;

/**
 * Can globally set the CRC computation for dataframe.
 */
typedef enum {
  /** DataFrame does not contain any CRC */
  CRC_OFF,
  /** DataFrame contain CRC byte as last byte of packet*/
  CRC_ON,
} DataframeCrc_t;

/**
 * Internal representation of error state.
 */
typedef enum {
  /** There is no error in dataframe create. */
  ERROR_NONE,
  /** The dataframe has small capacity */
  ERROR_OVERFLOW,
} DataframeError_t;

/**
 * @brief Class for representing data frame.
 */
class DataFrame {

private:
  uint8_t *_frame;
  uint8_t _length;
  uint8_t _capacity;
  DataframeType_t _type;
  DataframeCrc_t _crcUse;
  DataframeError_t _error;

public:
  /**
   * Constructor.
   * @param frame pointer to existing byte array
   * @param size size of buffer `frame`
   */
  DataFrame(uint8_t *frame, uint8_t size, DataframeType_t type, DataframeCrc_t);
  /**
   * @brief Initialize packet
   * - set the header of packet
   * - reserver 1st byte to length of frame
   */
  void Init(void);

  /**
   * @brief Add one byte to data frame.
   * @return true, if it is success
   */
  bool AddUint8(uint8_t d);

  /**
   * @brief add 2 Bytes length variable to dataframe
   * @param d variable added to data frame
   * @return true, if it is success
   */
  bool AddUint16(uint16_t d);

  /**
   * @brief add 4 Bytes long variable to dataframe
   * @param d variable added to data frame
   * @return true, if it is success
   */
  bool AddUint32(uint32_t);

  /**
   * @brief add one byte signed variable to data frame
   * @param d variable added to data frame
   * @return true, if it is success
   */
  bool AddInt8(int8_t d);

  /**
   * @brief add 2 bytes length signed variable to data frame
   * @param d variable added to data frame
   * @return true, if it is success
   */
  bool AddInt16(int16_t);

  /**
   * @brief add 4 bytes length signed variable to data frame
   * @param d variable added to data frame
   * @return true, if it is success
   */
  bool AddInt32(int32_t);

  /**
   * @brief add float value to data frame.
   * Float value is encoded to 4 bytes according IEEE 754
   * @param d variable added to data frame
   * @return true, if it is success
   */
  bool AddFloat(float);

  /**
   * @brief Commit the creation of the packet.
   * - add to second position (frame[1]) length of packet
   * - compute CRC of packet
   * @return real length of packet (including CRC byte)
   */
  uint8_t Commit(void);

  /**
   * @brief Return pointer to data frame.
   * @return resulting packet
   */
  uint8_t *GetFrame(void);

  /**
   * @brief Return error status from data processing
   * @return 0 - no error, 1 - internal buffer overflow. The buffer have to be
   * increased in user code.
   */
  DataframeError_t getError(void);
};

#ifdef __cplusplus
extern "C" {
#endif

uint8_t crc8(uint8_t crc, uint8_t Size, uint8_t *Buffer);

#ifdef __cplusplus
}
#endif

#endif /* DATAFRAME_H_ */
