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
  bool setCalibration0Deep();
  bool setCalibrationXDeep(float weightX = 100);

  bool getResultReadyDeep();
  WeightResult getLastWeight();
  float getLastTempDeep();

  bool requestMeasureDeep();
  bool requestTempDeep();

  // Connection checks
  bool verticalEncoderConnected();
  bool verticalStepperConnected();
  bool spiralMotorConnected();
  bool heightSensorConneted();
  bool weightSensorConnected(); // to be finished

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;
  DrillController _drillController;
  DeepSampleHolder _deepSampleHolder;


};