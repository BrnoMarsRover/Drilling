#ifndef CubeMarsDriverV2_UART
#define CubeMarsDriverV2_UART

#include <Arduino.h>
#include <vector>

struct motorDataRaw
{
  uint8_t valid = 0;
  int16_t MOSTmp;
  int16_t motorTmp;
  int32_t current;
  int32_t speed;
};

void setERPM(int32_t erpm);

void setRPM(float rpm);

void sendPayload(std::vector<uint8_t> payload);

void uartInit();

void requestAllData();

void requestTmpCurrRPM();

std::vector<uint8_t> readPayload();

struct motorDataRaw readTmpCurrRPM();

#endif