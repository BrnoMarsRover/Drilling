#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "DrillController/DrillController.h"
#include "DeepSampleHolder/DeepSampleHolder.h"

class DeepSampler
{
public:
  DeepSampler(TwoWire& wire, HardwareSerial& debugSerial);
  bool begin();
  void update();

  // Integrated carriage/spiral motor control

  // Low level carriage/vertical drive control
  bool setCarriageSpeedMMps(float MMps);
  float getCarriageHeightMM();

  // Low level spiral motor control
  bool setSpiralRPM(float rpm);
  float getSpiralRPM();
  float getSpiralMotorTmp();

  //Storage control
  void setTareDeep();
  void setCalibration0Deep();
  void setCalibration100Deep();

  bool getResultReadyDeep();
  float getLastWeightDeep();
  float getLastTempDeep();

  void requestMeasureDeep();
  void requestTempDeep();

  // Connection checks
  bool verticalEncoderConnected();
  bool verticalStepperConnected();
  bool spiralMotorConnected();
  bool heightSensorConneted();

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;
  DrillController _drillController;
  DeepSampleHolder _deepSampleHolder;


};