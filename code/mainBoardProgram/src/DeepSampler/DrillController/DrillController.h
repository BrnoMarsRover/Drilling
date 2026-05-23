#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "VL53L1X_Sensor.h"
#include "CubeMarsV2.h"
#include "LinearAxis/LinearAxis.h"

class DrillController
{
public:
  DrillController(TwoWire& wire, HardwareSerial& debugSerial);
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

  LinearAxis _linearAxis;
  CubeMarsV2 _motorDriver;
  VL53L1X_Sensor _distanceSensor;
};