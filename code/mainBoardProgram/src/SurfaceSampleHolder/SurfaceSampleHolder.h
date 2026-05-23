#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

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

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;
  //adc
  //servo
};