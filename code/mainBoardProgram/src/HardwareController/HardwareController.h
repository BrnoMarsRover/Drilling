#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "DrillController/DrillController.h"
#include "DeepSample/DeepSample.h"

class HardwareController
{
public:
  HardwareController(TwoWire& wire, HardwareSerial& debugSerial);  //will be deleted later. Wire and debugSerial will be initialized in begin.
  bool begin();
  void update();

  bool setCarriageSpeedMMps(float MMps);
  bool setSpiralRPM(float rpm);
  void setTareDeep();
  void setCalibration0Deep();
  void setCalibration100Deep();

  float getCarriageHeightMM();
  float getSpiralRPM();
  float getSpiralMotorTmp();
  bool getResultReadyDeep();
  float getLastWeightDeep();
  float getLastTempDeep();

  void requestMeasureDeep();
  void requestTempDeep();

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;
  DrillController _drillController;
  DeepSample _deepSample;
  // SurfaceSample _surfaceSample; // to be done later
  // Sample holder or something like that will belong here later.
};