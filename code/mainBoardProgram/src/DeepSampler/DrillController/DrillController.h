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

  // Linear Axis
  bool setCarriageSpeedMMps(float MMps);
  float getCarriageHeightMM();

  // Spiral motor
  bool setSpiralRPM(float rpm);
  float getSpiralRPM();
  float getSpiralMotorTmp();

  // Connection checks
  bool encoderConnected();
  bool stepperConnected();
  bool spiralMotorConnected();
  bool heightSensorConnected();

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;

  LinearAxis _linearAxis;
  CubeMarsV2 _motorDriver;
  VL53L1X_Sensor _heightSensor;
};