#pragma once
#include <Arduino.h>

#include "../shared/ADS122C04_LIB.h"
#include "../shared/serializer.h"

// ------------------------------------------------------------------ //
//  Message queue depth — increase if commands are arriving faster    //
//  than main() can process them                                       //
// ------------------------------------------------------------------ //
#define PROTOCOL_QUEUE_DEPTH 4

// ------------------------------------------------------------------ //
//  Command codes                                                      //
// ------------------------------------------------------------------ //
enum RoverCommand : uint8_t
{
    CMD_RESTART           = 0x01,
    CMD_STATE             = 0x02,
    CMD_DRILL_AUTO        = 0x03,
    CMD_STOP_AUTO         = 0x04,
    CMD_CALIBRATE_CARRIAGE_DEPTH  = 0x05,
    CMD_START_DEVICE_CHECK = 0x06,
    CMD_GET_DEVICE_STATUS = 0x07,
    CMD_DRILL_SPEED       = 0x20,
    CMD_VERTICAL_SPEED    = 0x21,
    CMD_STORAGE_POSITION  = 0x22,
    CMD_MEASURE_HEIGHT_ABOVE_GROUND    = 0x23,
    CMD_GET_HEIGHT_ABOVE_GROUND        = 0x24,
    CMD_WEIGH_DEEP        = 0x40,
    CMD_WEIGH_SURFACE     = 0x41,
    CMD_GET_WEIGHT_DEEP   = 0x42,
    CMD_GET_WEIGHT_SURFACE= 0x43,
    CMD_CALIBRATE_0_DEEP  = 0x44,
    CMD_CALIBRATE_X_DEEP  = 0x45,
    CMD_CALIBRATE_0_SURFACE = 0x46,
    CMD_CALIBRATE_X_SURFACE = 0x47,
    CMD_ROCK_OPEN         = 0x50,
    CMD_ROCK_CLOSE        = 0x51,
    CMD_SAND_OPEN         = 0x52,
    CMD_SAND_CLOSE        = 0x53,
};

// ------------------------------------------------------------------ //
//  Software state codes (used when sending STATE response)           //
// ------------------------------------------------------------------ //
enum DrillState : uint8_t
{
    STATE_INITIALIZING        = 0x00,
    STATE_ERROR               = 0x01,
    STATE_READY               = 0x02,
    STATE_AUTO_DRILLING_DOWN  = 0xF0,
    STATE_AUTO_CANT_REACH     = 0xF1,
    STATE_AUTO_MOVING_UP      = 0xF2,
    STATE_AUTO_STORING        = 0xF3,
};

// ------------------------------------------------------------------ //
//  Received message — populated by the parser, pulled by main()      //
// ------------------------------------------------------------------ //
class RoverMessage
{
public:
    RoverMessage(uint8_t* buffer, uint8_t length)
    {
        code = (RoverCommand)buffer[0];
        argLength = length - 1;           // argument bytes after the code
        for (uint8_t i = 0; i < argLength; i++)
        {
            argArray[i] = buffer[i + 1];
        }
    }
    RoverMessage() : code((RoverCommand)0), argLength(0) {}

    RoverCommand getCommandCode() const {return code;}
    uint8_t getUint8Arg() const {return argArray[0];}
    int8_t getInt8Arg() const  {return argArray[0];}
    uint16_t getUint16Arg() const    {return ((uint16_t)argArray[0] << 8) | (uint16_t)argArray[1];}
    int16_t getInt16Arg() const    {return ((int16_t)argArray[0] << 8) | (int16_t)argArray[1];}
private:
    enum RoverCommand code;
    uint8_t argArray[4];   // raw argument bytes, big-endian
    uint8_t argLength; // number of argument bytes (0 if none)
};

// ------------------------------------------------------------------ //
//  Protocol class                                                    //
// ------------------------------------------------------------------ //
class RoverComm
{
public:
    // Constructor. The serial needs to be initialized in setup, independent of the class.
    RoverComm(HardwareSerial& serial);

    // Call every loop() — runs the parser and feeds the rx queue
    void handle();

    // Returns true if at least one complete message is waiting
    bool messageAvailable() const;

    // Remove and return the oldest message from the queue.
    // Check messageAvailable() before calling.
    RoverMessage popMessage();

    // ---- Transmit helpers ---------------------------------------- //

    // Send a response with no data (e.g. simple ACK of a command)
    void sendAck(RoverCommand cmd);

    // Send a NACK (drill cannot perform the requested command)
    void sendNack();

    void sendUint16(RoverCommand cmd, uint16_t value);

    // Send a float value after the command code (e.g. weight response)
    void sendFloat(RoverCommand cmd, float value);

    // Send the full STATE response
    void sendState(float carriageDepthMM, float carriageSpeedMMps, float stepperCurrent, float rpm, float tempC, uint16_t trayAngle, DrillState swState);

    void sendDeviceStatus(bool vertStepper, bool vertEncoder, bool vertCurrentSensor, bool spiralMotor, bool heightSensor, bool deepSampleStepper, bool deepSampleEncoder, bool deepSampleADC, bool surfaceSampleADC);

    void sendWeight(RoverCommand cmd, WeightResult result);
private:
    HardwareSerial& _serial;

    // ---- Parser state -------------------------------------------- //
    enum ParseState { WAIT_START, READ_LENGTH, READ_PAYLOAD, READ_CKSUM, WAIT_END };
    ParseState _parserState = WAIT_START;

    uint8_t  _rxBuffer[32];
    uint8_t  _rxLength  = 0;
    uint8_t  _rxIndex   = 0;
    uint32_t _rxLastByteMillis = 0;
    uint32_t _rxTimeoutMillis  = 100;

    // ---- Receive queue ------------------------------------------- //
    RoverMessage _queue[PROTOCOL_QUEUE_DEPTH];
    uint8_t _queueHead = 0;  // next slot to write
    uint8_t _queueTail = 0;  // next slot to read
    uint8_t _queueCount = 0;

    void _pushMessage(RoverMessage& msg);

    // ---- Internal helpers ---------------------------------------- //
    uint8_t  _checksum(uint8_t* data, uint8_t length) const;
    void     _sendRaw(uint8_t* payload, uint8_t length);
};
