#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "src/RoverComm/RoverComm.h"
#include "src/DeepSampler/DeepSampler.h"
#include "src/SurfaceSampleHolder/SurfaceSampleHolder.h"


//-------I2C
constexpr uint8_t sdaPin = 21;
constexpr uint8_t sclPin = 22;
constexpr uint32_t i2cFrequency = 100000;
TwoWire I2CBus(0);

HardwareSerial& roverSerial = Serial0;

void setupI2CBus()
{
  // Print starting message for I2C recovery and setup
  roverSerial.println(F("Starting I2C Bus Recovery and Setup..."));

  // Check if I2C bus is free
  if (isI2CBusFree())
  {
    roverSerial.println(F("I2C bus is already free."));
  } else
  {
    // Bus is stuck, attempt recovery
    roverSerial.println(F("I2C bus is stuck, attempting recovery..."));
    if (i2cRecoverBus())
    {
      roverSerial.println(F("I2C bus recovery successful."));
    } else
    {
      roverSerial.println(F("I2C bus recovery failed!"));
    }
  }

  // Initialize the I2C bus
  I2CBus.begin(sdaPin, sclPin, i2cFrequency);
  I2CBus.setTimeOut(50);

  roverSerial.println(F("I2C Bus Setup Complete."));
}

bool isI2CBusFree()
{
  // Check if the SDA and SCL lines are both high (idle state for I2C)
  return (digitalRead(sdaPin) == HIGH && digitalRead(sclPin) == HIGH);
}

bool i2cRecoverBus()
{
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);

  if (digitalRead(sdaPin) == HIGH && digitalRead(sclPin) == HIGH)
  {
    return true; // Bus is already free
  }

  roverSerial.println(F("Attempting I2C bus recovery..."));

  // Try to generate clock pulses on SCL to free the bus
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  pinMode(sdaPin, INPUT_PULLUP);

  for (int i = 0; i < 20; i++)
  {  // Try up to 20 clock pulses
    digitalWrite(sclPin, LOW);
    delayMicroseconds(10);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);

    if (digitalRead(sdaPin) == HIGH)
    {
      roverSerial.println(F("SDA line released. Bus is free."));
      return true; // SDA line released, bus is free
    }
  }

  // Send STOP condition to release the bus
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(10);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(10);

  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);

  bool success = (digitalRead(sdaPin) == HIGH);
  roverSerial.println(success ? F("I2C recovery successful.") : F("I2C recovery failed."));

  return success;
}

//---------PERIPHERAL CLASSES
DeepSampler deepSampler(I2CBus, roverSerial);
SurfaceSampleHolder surfaceSampleHolder(I2CBus, roverSerial);

RoverComm roverComm(roverSerial);

void respondToMsg(const RoverMessage& msg)
{
  switch(msg.getCommandCode())
  {
    case CMD_RESTART:
    {
      roverComm.sendAck(CMD_RESTART);
      ESP.restart();
      break;
    }

    case CMD_STATE:
    {
      roverComm.sendState(
        deepSampler.getCarriageDepthMM(),
        deepSampler.getCarriageSpeedMMps(),
        deepSampler.getVerticalStepperCurrentA(),
        deepSampler.getSpiralMotorData(),
        deepSampler.storageGetCurrentAngle(),  //deep sample storage angle
        deepSampler.getAutoState(),
        deepSampler.getDrillControllerState(),
        deepSampler.getDeepSampleHolderState()
      );
      break;
    }

    case CMD_CALIBRATE_CARRIAGE_DEPTH:
    {
      if(deepSampler.setCarriageSpeedMMps(-10.0))
        roverComm.sendAck(CMD_CALIBRATE_CARRIAGE_DEPTH);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_START_DEVICE_CHECK:
    {
      roverComm.sendAck(CMD_START_DEVICE_CHECK);
      break;
    }

    case CMD_GET_DEVICE_STATUS:
    {
      roverComm.sendDeviceStatus(
        deepSampler.verticalStepperConnected(), //vertStepper
        deepSampler.verticalEncoderConnected(), //vertEncoder
        deepSampler.currentSensorIsConnected(),
        deepSampler.spiralMotorConnected(), //spiralMotor
        deepSampler.heightSensorConnected(), //heightSensor
        deepSampler.deepSampleStepperConnected(),  // deepSampleStepper
        deepSampler.deepSampleEncoderConnected(),  // deepSampleEncoder
        deepSampler.getAdcConnected(), //deepSampleADC
        surfaceSampleHolder.getAdcConnected() //surfaceSampleADC
      );
      break;
    }

    case CMD_DRILL_SPEED:
    {
      if(deepSampler.setSpiralRPM(msg.getInt16Arg()))
        roverComm.sendAck(CMD_DRILL_SPEED);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_VERTICAL_SPEED:
    {
      if(deepSampler.setCarriageSpeedMMps(((float)msg.getInt8Arg())*0.1))
        roverComm.sendAck(CMD_VERTICAL_SPEED);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_STORAGE_POSITION:
    {
      if(deepSampler.storageMoveToSlot((StepperPositioner::StoragePosition)msg.getUint8Arg()))
        roverComm.sendAck(CMD_STORAGE_POSITION);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_MEASURE_HEIGHT_ABOVE_GROUND:
    {
      if(deepSampler.startDistFromSurfaceMeasure())
        roverComm.sendAck(CMD_MEASURE_HEIGHT_ABOVE_GROUND);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_GET_HEIGHT_ABOVE_GROUND:
    {
      roverComm.sendUint16(CMD_GET_HEIGHT_ABOVE_GROUND, deepSampler.getDistFromSurfaceMM());
      break;
    }
    
    case CMD_WEIGH_DEEP:
    {
      if(deepSampler.requestMeasure())
        roverComm.sendAck(CMD_WEIGH_DEEP);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_WEIGH_SURFACE:
    {
      if(surfaceSampleHolder.requestMeasure())
        roverComm.sendAck(CMD_WEIGH_SURFACE);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_GET_WEIGHT_DEEP:
    {
      if(deepSampler.getResultReady()){
        roverComm.sendWeight(CMD_GET_WEIGHT_DEEP, deepSampler.getLastWeight());
      }
      else
        roverComm.sendNack();
      break;
    }

    case CMD_GET_WEIGHT_SURFACE:
      if(surfaceSampleHolder.getResultReady()){
        roverComm.sendWeight(CMD_GET_WEIGHT_SURFACE, surfaceSampleHolder.getLastWeight());
      }
      else
        roverComm.sendNack();
      break;

    case CMD_CALIBRATE_0_DEEP:
    {
      if(deepSampler.setCalibration0())
        roverComm.sendAck(CMD_CALIBRATE_0_DEEP);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_CALIBRATE_X_DEEP:
    {
      if(deepSampler.setCalibrationX(msg.getFloatArg())) // getFloat()
        roverComm.sendAck(CMD_CALIBRATE_X_DEEP);
      else
        roverComm.sendNack();
      break;
    }
    
    case CMD_CALIBRATE_0_SURFACE:
    {
      if(surfaceSampleHolder.setCalibration0())
        roverComm.sendAck(CMD_CALIBRATE_0_SURFACE);
      else
        roverComm.sendNack();
      break;
    }
    
    case CMD_CALIBRATE_X_SURFACE:
    {
      if(surfaceSampleHolder.setCalibrationX(msg.getFloatArg())) // getFloat()
        roverComm.sendAck(CMD_CALIBRATE_X_SURFACE);
      else
        roverComm.sendNack();
      break;
    }
    
    case CMD_ROCK_OPEN:
    {
      if(surfaceSampleHolder.openRockBox())
        roverComm.sendAck(CMD_ROCK_OPEN);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_ROCK_CLOSE:
    {
      if(surfaceSampleHolder.closeRockBox())
        roverComm.sendAck(CMD_ROCK_CLOSE);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_SAND_OPEN:
    {
      if(surfaceSampleHolder.openSandBox())
        roverComm.sendAck(CMD_SAND_OPEN);
      else
        roverComm.sendNack();
      break;
    }
    
    case CMD_SAND_CLOSE:
    {
      if(surfaceSampleHolder.closeSandBox())
        roverComm.sendAck(CMD_SAND_CLOSE);
      else
        roverComm.sendNack();
      break;
    }

    case SET_HOLD_MODE:
    {
      if(deepSampler.storageSetHoldMode(true))
      {
        roverComm.sendAck(SET_HOLD_MODE);
      }
      else
      {
        roverComm.sendNack();
      }
      break;
    }

    case CLEAR_HOLD_MODE:
    {
      if(deepSampler.storageSetHoldMode(false))
      {
        roverComm.sendAck(CLEAR_HOLD_MODE);
      }
      else
      {
        roverComm.sendNack();
      }
      break;
    }

    case CMD_STOP_AUTO:
    {
      if(deepSampler.setManualControl())
      {
        roverComm.sendAck(CMD_STOP_AUTO);
      }
      else
      {
        roverComm.sendNack();
      }
      break;
    }

    case CMD_DRILL_AUTO:
    {
      if(deepSampler.autoDrillStoreWeigh(10*((float)msg.getUint8Arg()) ) )
        roverComm.sendAck(CMD_DRILL_AUTO);
      else
        roverComm.sendNack();
      break;
    }
    
    case CMD_STORE_AUTO:
    {
      if(deepSampler.autoStoreWeigh())
        roverComm.sendAck(CMD_STORE_AUTO);
      else
        roverComm.sendNack();

      break;
    }

    default:
    {
      roverComm.sendNack();
      break;
    }
  }
}

// 3,3 V MOSFET switch - RESET pin
constexpr uint8_t resetPin = 2;
void startPeripheralReset()
{
  digitalWrite(resetPin, HIGH);
}
void endPeripheralReset()
{
  digitalWrite(resetPin, LOW);
}
void activeDelayReset()
{
  startPeripheralReset();
  delay(100);
  endPeripheralReset();
  delay(100);
}


void setup()
{
  pinMode(resetPin, OUTPUT);
  activeDelayReset();
  roverSerial.begin(38400);

  setupI2CBus();

  deepSampler.begin();
  surfaceSampleHolder.begin();
}

void loop()
{
  deepSampler.update();

  roverComm.handle();
  while(roverComm.messageAvailable())
  {
    respondToMsg(roverComm.popMessage());
  }
}



