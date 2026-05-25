#pragma once

#include <Arduino.h>
#include <cstring>

namespace ser
{
  inline uint16_t bytesToUint16(const uint8_t* bytes)
  {
    return (static_cast<uint16_t>(bytes[0]) << 8) |
           (static_cast<uint16_t>(bytes[1]));
  }

  inline void uint16ToBytes(uint16_t value, uint8_t* bytes)
  {
    bytes[0] = (value >> 8) & 0xFF;
    bytes[1] = value & 0xFF;
  }

  inline int16_t bytesToInt16(const uint8_t* bytes)
  {
    return static_cast<int16_t>(bytesToUint16(bytes));
  }

  inline void int16ToBytes(int16_t value, uint8_t* bytes)
  {
    uint16ToBytes(static_cast<uint16_t>(value), bytes);
  }

  inline uint32_t bytesToUint32(const uint8_t* bytes)
  {
    return (static_cast<uint32_t>(bytes[0]) << 24) |
           (static_cast<uint32_t>(bytes[1]) << 16) |
           (static_cast<uint32_t>(bytes[2]) <<  8) |
           (static_cast<uint32_t>(bytes[3])      );
  }

  inline void uint32ToBytes(uint32_t value, uint8_t* bytes)
  {
    bytes[0] = (value >> 24) & 0xFF;
    bytes[1] = (value >> 16) & 0xFF;
    bytes[2] = (value >> 8) & 0xFF;
    bytes[3] = value & 0xFF;
  }

  inline int32_t bytesToInt32(const uint8_t* bytes)
  {
    return static_cast<int32_t>(bytesToUint32(bytes));
  }

  inline void int32ToBytes(int32_t value, uint8_t* bytes)
  {
    uint32ToBytes(static_cast<uint32_t>(value), bytes);
  }

  inline float bytesToFloat(const uint8_t* bytes)
  {
    uint32_t temp = bytesToUint32(bytes);

    float value;
    std::memcpy(&value, &temp, sizeof(float));

    return value;
  }

  inline void floatToBytes(float value, uint8_t* bytes)
  {
    uint32_t temp;
    std::memcpy(&temp, &value, sizeof(float));

    uint32ToBytes(temp, bytes);
  }
}