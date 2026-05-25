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

  // High level control/autonomy

  // Low level carriage/vertical drive control
  bool setCarriageSpeedMMps(float MMps);
  float getCarriageHeightMM();

  // Low level spiral motor control
  bool setSpiralRPM(float rpm);
  float getSpiralRPM();
  float getSpiralMotorTmp();

  //Storage control
  bool setCalibration0();
  bool setCalibrationX(float weightX = 100);

  bool getResultReady();
  WeightResult getLastWeight();
  float getLastTemp();

  bool requestMeasure();
  bool requestTemp();

  // Connection checks
  bool verticalEncoderConnected();
  bool verticalStepperConnected();
  bool spiralMotorConnected();
  bool heightSensorConneted();
  bool getAdcConnecred(); // to be finished

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;
  DrillController _drillController;
  DeepSampleHolder _deepSampleHolder;


};