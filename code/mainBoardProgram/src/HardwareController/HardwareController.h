#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "ADS122C04_LIB.h"
#include "DrillController/DrillController.h"

class HardwareController
{
public:
  HardwareController(TwoWire& wire, HardwareSerial& debugSerial);  //will be deleted later. Wire and debugSerial will be initialized in begin.
  bool begin();
  void update();

  bool setCarriageSpeedMMps(float MMps);
  bool setSpiralRPM(float rpm);
  
  float getCarriageHeightMM();
  float getSpiralRPM();
  float getSpiralMotorTmp();

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;
  DrillController _drillController;
  //Sample holder or something like that will belong here later.
};