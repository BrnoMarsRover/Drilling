#include <Arduino.h>
#include <HardwareSerial.h>
#include "CubeMarsV2.h"

#define RQMASK_MOSTMP 0x01
#define RQMASK_MOTORTMP 0x02
#define RQMASK_OUTCURRENT 0x04
#define RQMASK_RPM 0x80


// ---------------- PUBLIC ----------------

CubeMarsV2::CubeMarsV2(HardwareSerial& serialPort, HardwareSerial& debugSerialPort, uint8_t aRxPin, uint8_t aTxPin)
  : cubeMarsSerial(serialPort), debugSerial(debugSerialPort), rxPin(aRxPin), txPin(aTxPin)
{
}

void CubeMarsV2::begin()
{
  cubeMarsSerial.begin(921600, SERIAL_8N1, rxPin, txPin);
  commNextMillis = millis();
}

void CubeMarsV2::update()
{
  handleRX();

  if(millis() > commNextMillis)
  {
    commNextMillis += commDeltaMillis;
    if(requestedERPM == 0)
    {
      transmitDuty(0.0);
    }
    else
    {
      transmitERPM();
    }

    requestTmpCurrRPM();
  }
}

void CubeMarsV2::setERPM(int32_t erpm)
{
  requestedERPM = erpm;
  transmitERPM();
}

void CubeMarsV2::setRPM(float rpm)
{
  setERPM((int32_t)(rpm * poleCount * gearboxRatio));
}

void CubeMarsV2::requestAllData()
{
  uint8_t payload[1] = {4};
  transmitPayload(payload, 1);
}

void CubeMarsV2::requestTmpCurrRPM()
{
  uint8_t payload[5] = {50, 0,0,0,0};
  int32_t mask = RQMASK_MOSTMP | RQMASK_MOTORTMP | RQMASK_OUTCURRENT | RQMASK_RPM;
  int32ToBytes(mask, payload + 1);
  transmitPayload(payload, 5);
}

float CubeMarsV2::getMOSTmp()  { return MOSTmp; }
float CubeMarsV2::getMotorTmp() { return motorTmp; }
float CubeMarsV2::getCurrent() { return current; }
float CubeMarsV2::getRPM()     { return RPM; }

void CubeMarsV2::printMotorInfoToDebug()
{
  debugSerial.print("MOS tmp: ");
  debugSerial.println(MOSTmp);
  debugSerial.print("Motor tmp: ");
  debugSerial.println(motorTmp);
  debugSerial.print("Current: ");
  debugSerial.println(current);
  debugSerial.print("Speed: ");
  debugSerial.println(RPM);
}

// ---------------- PRIVATE ----------------

void CubeMarsV2::transmitERPM()
{
  uint8_t payload[5] = {8, 0,0,0,0};
  int32ToBytes(requestedERPM, payload + 1);
  transmitPayload(payload, 5);
}


void CubeMarsV2::transmitDuty(float duty)
{
  uint8_t payload[5] = {0x05, 0, 0, 0, 0};
  int32_t val = (int32_t)(duty * 100000.0f);
  int32ToBytes(val, payload + 1);
  transmitPayload(payload, 5);
}

void CubeMarsV2::transmitPayload(uint8_t* payload, uint8_t payloadLength)
{
  uint8_t msgLength = payloadLength + 5; //extended by start, length, 2 bytes for checksum, end
  uint8_t msg[msgLength];

  msg[0] = 2;             // STX
  msg[1] = payloadLength;

  for (size_t i = 0; i < payloadLength; i++)
    msg[2 + i] = payload[i];

  uint16_t cksum = crc16(payload, payloadLength);
  msg[payloadLength + 2] = (cksum >> 8) & 0xFF;   // CRC high byte
  msg[payloadLength + 3] = cksum & 0xFF;          // CRC low byte
  msg[payloadLength + 4] = 3;                     // ETX

  cubeMarsSerial.write(msg, msgLength);
}

uint16_t CubeMarsV2::crc16(uint8_t* buffer, uint8_t bufferLength)
{
  uint16_t cksum = 0;
  for (size_t i = 0; i < bufferLength; i++)
    cksum = crc16_tab[((cksum >> 8) ^ buffer[i]) & 0xFF] ^ (cksum << 8);
  return cksum;
}

int32_t CubeMarsV2::bytesToInt32(uint8_t* buffer)
{
  return ((int32_t)buffer[0] << 24) | ((int32_t)buffer[1] << 16) | ((int32_t)buffer[2] << 8) | buffer[3];
}

int16_t CubeMarsV2::bytesToInt16(uint8_t* buffer)
{
  return ((int16_t)buffer[0] << 8) | buffer[1];
}

void CubeMarsV2::int32ToBytes(int32_t input, uint8_t* output)
{
  output[0] = (input >> 24) & 0xFF;
  output[1] = (input >> 16) & 0xFF;
  output[2] = (input >> 8) & 0xFF;
  output[3] = input & 0xFF;
}


void CubeMarsV2::handleRX()
{
  // Reset parser if it has been stuck mid-packet for too long
  if (parserState != WAIT_START && (millis() - rxLastByteMillis) > rxTimeoutMillis)
  {
    parserState = WAIT_START;
  }

  while (cubeMarsSerial.available())
  {
    rxLastByteMillis = millis();
    uint8_t b = cubeMarsSerial.read();

    switch (parserState)
    {
      case WAIT_START:
        if (b == 2) parserState = READ_LENGTH;
        break;

      case READ_LENGTH:
        if (b == 0 || b > MAX_PAYLOAD - 1)  // -1 to leave room for checksum byte
        {
          parserState = WAIT_START;
          break;
        }
        rxLength = b;
        rxIndex = 0;
        parserState = READ_PAYLOAD;
        break;

      case READ_PAYLOAD:
        rxBuffer[rxIndex] = b;
        rxIndex++;
        if (rxIndex >= rxLength) parserState = READ_CRC1;
        break;

      case READ_CRC1:
        rxBuffer[rxLength] = b;
        parserState = READ_CRC2;
        break;

      case READ_CRC2:
        rxBuffer[rxLength + 1] = b;
        parserState = WAIT_END;
        break;

      case WAIT_END:
        if (b == 3) // ETX
        {
          uint16_t receivedCRC = (uint16_t)((rxBuffer[rxLength] << 8) | rxBuffer[rxLength + 1]);
          uint16_t computedCRC = crc16(rxBuffer, rxLength);
          if (receivedCRC == computedCRC)
          {
            readTmpCurrRPM(); // call interpreter
          }
        }
        parserState = WAIT_START;
        break;
    }
  }
}

void CubeMarsV2::readTmpCurrRPM()
{
  if (rxLength != 17 || rxBuffer[0] != 50) return;

  uint8_t begin = 5;

  MOSTmp = 0.1 * bytesToInt16(rxBuffer + begin);
  motorTmp = 0.1 * bytesToInt16(rxBuffer + begin + 2);
  current = 0.01 * bytesToInt32(rxBuffer + begin + 4);
  RPM = (1.0 / (poleCount * gearboxRatio)) * bytesToInt32(rxBuffer + begin + 8);
}