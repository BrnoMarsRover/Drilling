#include "roverComm.h"

#define STX 0x02
#define ETX 0x03

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

RoverComm::RoverComm(HardwareSerial& serial) : _serial(serial)
{
}

void RoverComm::handle()
{
    // Reset parser if it has been stuck mid-packet for too long
    if (_parserState != WAIT_START && (millis() - _rxLastByteMillis) > _rxTimeoutMillis)
    {
        _parserState = WAIT_START;
    }

    while (_serial.available())
    {
        _rxLastByteMillis = millis();
        uint8_t b = _serial.read();

        switch (_parserState)
        {
            case WAIT_START:
                if (b == STX) _parserState = READ_LENGTH;
                break;

            case READ_LENGTH:
                // Reject zero-length or oversized packets
                if (b == 0 || b > sizeof(_rxBuffer) - 1)
                {
                    _parserState = WAIT_START;
                    break;
                }
                _rxLength = b;
                _rxIndex  = 0;
                _parserState = READ_PAYLOAD;
                break;

            case READ_PAYLOAD:
                _rxBuffer[_rxIndex++] = b;
                if (_rxIndex >= _rxLength) _parserState = READ_CKSUM;
                break;

            case READ_CKSUM:
                _rxBuffer[_rxLength] = b; // store checksum just after payload
                _parserState = WAIT_END;
                break;

            case WAIT_END:
                if (b == ETX)
                {
                    // Verify checksum: sum of payload + checksum byte must wrap to 0
                    uint8_t sum = 0;
                    for (uint8_t i = 0; i < _rxLength + 1; i++)
                    {
                        sum += _rxBuffer[i];
                    }
                    if (sum == 0)
                    {
                        RoverMessage msg(_rxBuffer, _rxLength);
                        _pushMessage(msg);
                    }
                }
                _parserState = WAIT_START;
                break;
        }
    }
}

bool RoverComm::messageAvailable() const
{
    return _queueCount > 0;
}

RoverMessage RoverComm::popMessage()
{
    RoverMessage msg = _queue[_queueTail];
    _queueTail = (_queueTail + 1) % PROTOCOL_QUEUE_DEPTH;
    _queueCount--;
    return msg;
}

// ------------------------------------------------------------------ //
//  Transmit helpers                                                   //
// ------------------------------------------------------------------ //

void RoverComm::sendAck(RoverCommand cmd)
{
    uint8_t payload[1] = { (uint8_t)cmd };
    _sendRaw(payload, 1);
}

void RoverComm::sendNack()
{
    uint8_t payload[1] = { 0x00 };
    _sendRaw(payload, 1);
}

void RoverComm::sendUint16(RoverCommand cmd, uint16_t value)
{
    uint8_t payload[3];
    payload[0] = (uint8_t)cmd;
    ser::uint16ToBytes(value, payload + 1);
    _sendRaw(payload, 3);
}

void RoverComm::sendFloat(RoverCommand cmd, float value)
{
    uint8_t payload[5];
    payload[0] = (uint8_t)cmd;
    ser::floatToBytes(value, payload + 1);
    _sendRaw(payload, 5);
}

void RoverComm::sendState(float carriageDepthMM, float carriageSpeedMMps, float stepperCurrent, float rpm, float tempC, uint16_t trayAngle, DrillState swState)
{
    uint8_t payload[11];
    payload[0] = (uint8_t)CMD_STATE;

    ser::int16ToBytes((int16_t)carriageDepthMM, payload + 1);

    payload[3] = (int8_t)(carriageSpeedMMps*10);

    payload[4] = (uint8_t)stepperCurrent;

    ser::int16ToBytes((int16_t)rpm, payload + 5);

    payload[7] = (uint8_t)tempC;

    ser::uint16ToBytes(trayAngle, payload + 8);

    payload[10] = (uint8_t)swState;

    _sendRaw(payload, 11);
}

void RoverComm::sendDeviceStatus(bool vertStepper, bool vertEncoder, bool vertCurrentSensor, bool spiralMotor, bool heightSensor, bool deepSampleStepper, bool deepSampleEncoder, bool deepSampleADC, bool surfaceSampleADC)
{
    uint16_t status =
        ((uint16_t)vertStepper          << 0) |
        ((uint16_t)vertEncoder           << 1) |
        ((uint16_t)vertCurrentSensor     << 2) |
        ((uint16_t)spiralMotor            << 3) |
        ((uint16_t)heightSensor           << 4) |
        ((uint16_t)deepSampleStepper      << 5) |
        ((uint16_t)deepSampleEncoder      << 6) |
        ((uint16_t)deepSampleADC          << 7) |
        ((uint16_t)surfaceSampleADC       << 8);

    uint8_t payload[3];
    payload[0] = (uint8_t)CMD_GET_DEVICE_STATUS;
    ser::uint16ToBytes(status, payload + 1);
    _sendRaw(payload, 3);
}

void RoverComm::sendWeight(RoverCommand cmd, WeightResult result)
{
    uint8_t payload[9];
    payload[0] = (uint8_t)cmd;

    // Copy float as big-endian bytes
    ser::floatToBytes(result.grams, payload + 1);
    ser::uint32ToBytes(result.raw, payload + 5);
    _sendRaw(payload, 9);
}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //


void RoverComm::_pushMessage(RoverMessage& msg)
{
    if (_queueCount >= PROTOCOL_QUEUE_DEPTH)
    {
        // Queue full — drop the oldest message to make room
        _queueTail = (_queueTail + 1) % PROTOCOL_QUEUE_DEPTH;
        _queueCount--;
    }

    _queue[_queueHead] = msg;
    _queueHead = (_queueHead + 1) % PROTOCOL_QUEUE_DEPTH;
    _queueCount++;
}

uint8_t RoverComm::_checksum(uint8_t* data, uint8_t length) const
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++) sum += data[i];
    return (0x100 - sum) & 0xFF;
}

void RoverComm::_sendRaw(uint8_t* payload, uint8_t length)
{
    uint8_t cs = _checksum(payload, length);
    _serial.write(STX);
    _serial.write(length);
    _serial.write(payload, length);
    _serial.write(cs);
    _serial.write(ETX);
}
