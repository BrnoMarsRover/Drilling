#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../../shared/ADS122C04_LIB.h" //ADS122C04_LIB/
#include "StepperPositioner.h"

class DeepSampleHolder
{
public:
  DeepSampleHolder(TwoWire& wire);
  bool begin();

  bool requestMeasure();
  bool requestTemp();

  bool getResultReady();
  WeightResult getLastWeight();
  float getLastTemp();

  bool setTare();
  bool setCalibration0();
  bool setCalibration100();
  void reset();

private:
  TwoWire& _wire;

  ADS122C04 _adcDeep;
  //StepperPositioner _stepperPositioner;
};