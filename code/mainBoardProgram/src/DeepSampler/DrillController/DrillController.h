#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "VL53L1X_Sensor.h"
#include "CubeMarsV2.h"
#include "LinearAxis/LinearAxis.h"

enum ControlMode
{
  MANUAL,
  AUTOMATIC
};

enum AutoState
{
  WAITING_FOR_HEIGHT,
  MOVING_DOWN,
  DRILLING,
  MOVING_UP,
  DONE,
  ERROR
};

class DrillController
{
public:
  DrillController(TwoWire& wire, HardwareSerial& debugSerial);
  bool begin();
  void update();

  // Linear Axis
  bool setCarriageSpeedMMps(float MMps);
  float getCarriageSpeedMMps() const;
  float getCarriageDepthMM();

  // Spiral motor
  bool setSpiralRPM(float rpm);
  float getSpiralRPM();
  float getSpiralMotorTmp();

  // Integrated drill control
  bool startDistFromSurfaceMeasure();
  float getDistFromSurfaceMM();
  bool setManualControl();
  bool autoDrillToDepth(float rateOfPenetrationMMpRev, float targetRPM, float targetDepthMM);
  //multiplier 1 -> spiral is "screwed" into the material
  //higher multiplier -> motor needs more power, but can get into harder materials
  //use multiplier about 10x

  ControlMode getControlMode();
  AutoState getAutoState();

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

  ControlMode _controlMode = MANUAL;
  AutoState _autoState = DONE;

  float _rateOfPenetrationMMpRev = 0;
  float _targetSpiralRPS = 0;
  float _targetDepthMM = 0;

  static constexpr float linAxisZeroToSensorMM = 77.5;
  static constexpr float carriageTopToSpiralTipMM = 72.0;
  float spiralDepthBelowGroundMM()
  {
    return _linearAxis.getDepthMM() + carriageTopToSpiralTipMM - linAxisZeroToSensorMM - _heightSensor.getDistanceMM();
  }

  static constexpr float spiralLead = 80.0; //millimeters per revolution of the spiral. Currently Unused.
};