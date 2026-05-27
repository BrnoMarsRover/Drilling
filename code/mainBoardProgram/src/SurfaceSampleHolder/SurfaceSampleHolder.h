#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "../shared/ADS122C04_LIB.h"
#include "servo_mg.h"


class SurfaceSampleHolder
{
public:
  SurfaceSampleHolder(TwoWire& wire, HardwareSerial& debugSerial);

  //asi tady tak nějak?
  bool openRockBox();
  bool closeRockBox();
  //void weighRock();
  //float getRockWeight();

  bool openSandBox();
  bool closeSandBox();
  //void weighSand();
  //float getSandWeight();

  void update();
  uint8_t getPosRock(); // useless?
  uint8_t getPosSand(); // useless?
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

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;
  ADS122C04 _adcSurface;
  SERVO_MG _servoRock;
  SERVO_MG _servoSand;
};