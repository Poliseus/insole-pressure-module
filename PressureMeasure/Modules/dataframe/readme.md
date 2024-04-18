# Simple wrapper for DataFrame

Create formatted frame from basic types:
 - uint8_t, int8_t => 1Byte
 - uint16_t, int16_t => 2Bytes
 - uint32_t, int32_t => 4Bytes
 - float => 4Bytes

Support modes:
 - Plain (TYPE_PLAIN): construct simple dataframe, 
 - 1B header (TYPE_HEADER_1B): construct dataframe with header byte (0xCC) as first byte and CRC checksum as last byte.
 -  2B header (TYPE_HEADER_2B): construct dataframe with first 2 bytes as header (0xCCDD) and CRC checksum as last byte.
 
 Data are stored in Little Endian format.

Structure of data frame (**TYPE_HEADER_1B**):
  - data[0] - starting byte (0xCC)
  - data[1] - length of frame: count of bytes in payload. The first 2 bytes is not included to overal length. Maximum length: 254 (0xFE)
  - payload
  - CRC8 (if it is needed)
  
  Structure of data frame (**TYPE_HEADER_2B**):
  - data[0] - starting byte (0xCC)
  - data[1] - starting byte (0xDD)
  - data[2] - length of frame: count of bytes in payload. The first 3 bytes is not included to overal length. Maximum length: 253 (0xFD)
  - payload
  - CRC8 (if it is needed)
  
Structure of data frame (**TYPE_PLAIN**):
  - data[0] - length of frame: count of bytes in payload. The first byte is not included to overal length. Maximum length: 254 (0xFE)
  - payload
 
 
 The library contans universal function:
 - `uint8_t crc8(uint8_t crc, uint8_t Size, uint8_t *Buffer);`
  - crc - inital value for CRC computig. For new frame, use 0x0
  - Size - size of data Buffer
   

## Basic usage

```c
    void main(){
    uint8_t data[24];

    DataFrame packet(data_packet, sizeof(data_packet), TYPE_HEADER_1B, CRC_ON);

    packet.AddUint8(0xAB);
    packet.AddUint16(65874);
    packet.AddInt8(-45874);
    packet.AddUint32(0x12345678);
    packet.AddInt32(-0xA2345678);
    
    packet.AddFloat(0.265);
    packet.AddFloat(-3.1415);

   // resulted length is 23B (20B of data, 1B breamble, 1B lenfth byte, 1B CRC)
    int data_length = packet.Commit();  
    if (packet.getError() == ERROR_NONE) {
        // frame[1] contain packet length. The real value is 20
        uint8_t* frame = packet.GetFrame();
    }
}

```

## CRC8 Computation

- Generator polynome: 0x97 ->  0x197 = x^8 + x^7 + x^4 + x^2 + x^1 +1 
- CRC is implemented as fast computation with prepared CrcTable. Algorithm complexity is O(n)
