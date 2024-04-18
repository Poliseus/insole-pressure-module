/*
 * @file dataframe.cpp
 * @brief Simple Data Frame
 *
 * @date: 31. 12. 2022
 * @author: juraj
 */

/**
 * @mainpage DataFrame library
 *
Create formatted frame from basic types:
 - uint8_t, int8_t => 1Byte
 - uint16_t, int16_t => 2Bytes
 - uint32_t, int32_t => 4Bytes
 - float => 4Bytes

 Data are stared in Little Endian format.

Structure of data frame:
  - starting byte (0xCC)
  - length of frame: count of bytes in payload. The first 2 bytes is not
included to overal length. Maximum length: 254 (0xFE)
  - payload
  - CRC8

 The library contans universal function:
 - `uint8_t crc8(uint8_t crc, uint8_t Size, uint8_t *Buffer);`
 - crc - inital value for CRC computig. For new frame, use 0x0
 - Size - size of data Buffer

Detailed description of class: DataFrame

   \code{.c}
    void main(){
        uint8_t data[24];

        DataFrame packet(data_packet, sizeof(data_packet), TYPE_HEADER_1B,
CRC_ON);

        packet.AddUint8(0xAB);
        packet.AddUint16(65874);
        packet.AddInt8(-45874);
        packet.AddUint32(0x12345678);
        packet.AddInt32(-0xA2345678);

        packet.AddFloat(0.265);
        packet.AddFloat(-3.1415);

        // resulted length is 23B (20B of data, 1B preamble, 1B lenfth byte,1B
CRC) int data_length = packet.Commit(); if (packet.getError() == ERROR_NONE) {
            // frame[1] contain packet length. The real value is 20
            uint8_t* frame = packet.GetFrame();
        }
    }
  \endcode

 */

#include "dataframe.h"

/**
 * @brief Helper function to compute CRC.
 * @param crc - initial CRC, use 0 if dont know about it.
 * @param Size - size of input buffer
 * @param Buffer - input array with bytes
 */
uint8_t crc8(uint8_t crc, uint8_t Size, uint8_t *Buffer) {
  // https://community.st.com/s/question/0D50X0000CDmAkpSQF/calculate-crc8
  static const unsigned char CrcTable[] = {
      // 0x97 Polynomial Table, 8-bit,
      0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73,
      0xB8, 0x2F, 0x01, 0x96, 0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71,
      0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C, 0xE3, 0x74, 0x5A, 0xCD,
      0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
      0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A,
      0xE1, 0x76, 0x58, 0xCF, 0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A,
      0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7, 0xEB, 0x7C, 0x52, 0xC5,
      0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
      0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1,
      0x0A, 0x9D, 0xB3, 0x24, 0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3,
      0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E, 0xA2, 0x35, 0x1B, 0x8C,
      0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
      0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B,
      0xA0, 0x37, 0x19, 0x8E, 0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A,
      0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7, 0xFB, 0x6C, 0x42, 0xD5,
      0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
      0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80,
      0x4B, 0xDC, 0xF2, 0x65, 0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82,
      0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF, 0x10, 0x87, 0xA9, 0x3E,
      0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
      0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9,
      0x12, 0x85, 0xAB, 0x3C};

  while (Size--) {
    crc = crc ^ *Buffer++;      // Apply Byte
    crc = CrcTable[crc & 0xFF]; // One round of 8-bits
  }

  if (crc == 0) {
    crc = 1;
  }

  return (crc);
}

DataFrame::DataFrame(uint8_t *frame, uint8_t size, DataframeType_t type,
                     DataframeCrc_t useCrc) {
  _frame = frame;
  _capacity = size;
  _type = type;
  _crcUse = useCrc;
  this->Init();
}

void DataFrame::Init(void) {
  _error = ERROR_NONE;

  if (_type == TYPE_HEADER_1B) {
    _frame[0] = HEADER_CHAR1;
    _frame[1] = 0; // length of packet
    _length = 2;
  }

  if (_type == TYPE_HEADER_2B) {
    _frame[0] = HEADER_CHAR1;
    _frame[1] = HEADER_CHAR2;
    _frame[2] = 0; // length of packet
    _length = 3;
  }

  if (_type == TYPE_PLAIN) {
    _frame[1] = 0; // length of packet
    _length = 1;
  }
}

bool DataFrame::AddUint8(uint8_t d) {
  if ((_length + 1) >= _capacity) {
    _error = ERROR_OVERFLOW;
    return false;
  }

  _frame[_length] = d;
  _length++;
  return true;
}

bool DataFrame::AddUint16(uint16_t d) {
  if ((_length + 2) >= _capacity) {
    _error = ERROR_OVERFLOW;
    return false;
  }

  _frame[_length++] = d & 0xFF;
  _frame[_length++] = (d >> 8) & 0xFF;

  return true;
}

bool DataFrame::AddUint32(uint32_t d) {
  if ((_length + 4) >= _capacity) {
    _error = ERROR_OVERFLOW;
    return false;
  }

  _frame[_length++] = d & 0xFF;
  _frame[_length++] = (d >> 8) & 0xFF;
  _frame[_length++] = (d >> 16) & 0xFF;
  _frame[_length++] = (d >> 24) & 0xFF;

  return true;
}

bool DataFrame::AddInt8(int8_t d) { return this->AddUint8((uint8_t)d); }

bool DataFrame::AddInt16(int16_t d) { return this->AddUint16((uint16_t)d); }

bool DataFrame::AddInt32(int32_t d) { return this->AddUint32((uint32_t)d); }

bool DataFrame::AddFloat(float f) {
  if ((_length + 4) >= _capacity) {
    _error = ERROR_OVERFLOW;
    return false;
  }

  uint8_t *ptr;
  ptr = (unsigned char *)&f;

  _frame[_length++] = *(ptr);
  _frame[_length++] = *(ptr + 1);
  _frame[_length++] = *(ptr + 2);
  _frame[_length++] = *(ptr + 3);

  return true;
}

uint8_t DataFrame::Commit(void) {
  if (_type == TYPE_HEADER_1B) {
    // compute _length
    if (_crcUse == CRC_ON) {
      _length++;
      _frame[_length - 1] = crc8(0, _length - 1, _frame);
    }
    _frame[1] = _length - 2;
    // length of payload: from byte 2 to CRC byte (the last)
  }

  if (_type == TYPE_HEADER_2B) {
    // compute _length
    if (_crcUse == CRC_ON) {
      _length++;
      _frame[_length - 1] = crc8(0, _length - 1, _frame);
      // length of payload: from byte 2 to CRC byte (the last)
    }
    _frame[2] = _length - 3;
  }

  if (_type == TYPE_PLAIN) {
    _frame[0] = _length;
  }
  return _length;
}

DataframeError_t DataFrame::getError(void) { return _error; }

uint8_t *DataFrame::GetFrame(void) { return _frame; }
