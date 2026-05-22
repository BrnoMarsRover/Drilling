#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../../shared/ADS122C04_LIB.h" //ADS122C04_LIB/

class DeepSample
{
public:
  DeepSample(TwoWire& wire);
  bool begin();

  void requestMeasure();
  void requestTemp();

  bool getResultReady();
  float getLastWeight();
  float getLastTemp();

  void setTare();
  void setCalibration0();
  void setCalibration100();
  void reset();

private:
  TwoWire& _wire;

  ADS122C04 _adcDeep;
  //ADS122C04 _adcSurface;
};