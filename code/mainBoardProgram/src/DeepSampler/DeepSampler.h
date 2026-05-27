#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <FastAccelStepper.h>

#include "DrillController/DrillController.h"
#include "DeepSampleHolder/DeepSampleHolder.h"

class DeepSampler
{
public:
  enum class AutoState
  {
    MANUAL,
    WAITING_FOR_STORAGE_CLEAR,
    DRILLING,
    MOVING_STORAGE,
    MOVING_CARRIAGE_TO_STORE,
    STORING,
    WEIGHING,
    MOVING_UP,
    DONE,
    ERROR
  };

  DeepSampler(TwoWire& wire, HardwareSerial& debugSerial);
  bool begin();
  void update();

  // High level control/autonomy
  AutoState getAutoState();
  bool setManualControl();
  bool startDistFromSurfaceMeasure();
  float getDistFromSurfaceMM();
  bool autoSampleAndWeigh(float targetDepthMM);

  // Low level carriage/vertical drive control
  bool setCarriageSpeedMMps(float MMps);
  float getCarriageSpeedMMps() const;
  float getCarriageDepthMM();

  // Low level spiral motor control
  bool setSpiralRPM(float rpm);
  float getSpiralRPM();
  float getSpiralMotorTmp();

  //Storage control
  bool setCalibration0();
  bool setCalibrationX(float);

  bool storageMoveToSlot(uint8_t slot);
  bool storageUnlock();
  bool storageSetHoldMode(bool hold);
  uint16_t storageGetCurrentAngle() const;

  bool getResultReady();
  WeightResult getLastWeight();
  float getLastTemp();

  bool requestMeasure();
  bool requestTemp();

  // Connection checks
  bool verticalEncoderConnected();
  bool verticalStepperConnected();
  bool spiralMotorConnected();
  bool heightSensorConnected();
  bool getAdcConnected(); // to be finished

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;

  FastAccelStepperEngine _stepperEngine;
  
  DrillController _drillController;
  DeepSampleHolder _deepSampleHolder;

  AutoState _autoState = AutoState::MANUAL;
  float _targetDepthMM;
  uint8_t storeSlot = 2;

  uint32_t _storingStartTimeMS = 0;
  static constexpr uint32_t _storingDurationMS = 5000;
};