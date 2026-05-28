#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <FastAccelStepper.h>

#include "VL53L1X_Sensor.h"
#include "CubeMarsV2.h"
#include "LinearAxis/LinearAxis.h"

class DrillController
{
public:
  enum class AutoState
  {
    MANUAL,
    WAITING_FOR_HEIGHT,
    MOVING_DOWN,
    DRILLING,
    MOVING_UP,
    DONE,
    ERROR
  };

  DrillController(TwoWire& wire, HardwareSerial& debugSerial, FastAccelStepperEngine& stepperEngine);
  bool begin();
  void update();

  // Linear Axis
  bool setCarriageSpeedMMps(float MMps);
  float getCarriageSpeedMMps() const;
  float getCarriageDepthMM() const;
  float getVerticalStepperCurrentA() const;
  bool currentSensorIsConnected() const;

  float spiralDepthBelowSensor()
  {
    return _linearAxis.getDepthMM() + carriageTopToSpiralTipMM - linAxisZeroToSensorMM;
  }

  float spiralDepthBelowGroundMM()
  {
    return _linearAxis.getDepthMM() + carriageTopToSpiralTipMM - linAxisZeroToSensorMM - _heightSensor.getDistanceMM();
  }

  // Spiral motor
  bool setSpiralRPM(float rpm);
  float getSpiralRPM();
  float getSpiralMotorTmp();

  // Integrated drill control
  AutoState getAutoState();
  bool startDistFromSurfaceMeasure();
  float getDistFromSurfaceMM();
  bool setManualControl();
  bool autoDrillToDepth(float rateOfPenetrationMMpRev, float targetRPM, float targetDepthMM);


  // Connection checks
  bool encoderConnected();
  bool stepperConnected();
  bool spiralMotorConnected();
  bool heightSensorConnected();

private:
  FastAccelStepperEngine& _stepperEngine;

  TwoWire& _wire;
  HardwareSerial& _debugSerial;

  LinearAxis _linearAxis;
  CubeMarsV2 _motorDriver;
  VL53L1X_Sensor _heightSensor;

  AutoState _autoState = AutoState::MANUAL;

  float _rateOfPenetrationMMpRev = 0;
  float _targetSpiralRPS = 0;
  float _targetDepthMM = 0;

  static constexpr float linAxisZeroToSensorMM = 775.0;
  static constexpr float carriageTopToSpiralTipMM = 720.0;

  static constexpr float spiralLead = 80.0; //millimeters per revolution of the spiral. Currently Unused.
};