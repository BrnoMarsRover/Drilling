#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "../shared/ADS122C04_LIB.h"

class SurfaceSampleHolder
{
public:
  SurfaceSampleHolder(TwoWire& wire, HardwareSerial& debugSerial);

  //asi tady tak nějak?
  void openRockBox();
  void closeRockBox();
  void weighRock();
  float getRockWeight();

  void openSandBox();
  void closeSandBox();
  void weighSand();
  float getSandWeight();

  // ADC
  bool begin();

  bool requestMeasure();
  bool requestTemp();

  bool getResultReady();
  WeightResult getLastWeight();
  float getLastTemp();
  bool getAdcConnected();

  bool setTare();
  bool setCalibration0();
  bool setCalibrationX(float);
  void reset();

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;
  ADS122C04 _adcSurface;
  //servo
};