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
    RAISING_BEFORE_STORE,
    WAITING_FOR_STORAGE_CLEAR,
    DRILLING,
    MOVE_CARRIAGE_TO_STORE,
    MOVING_CARRIAGE_TO_STORE,
    MOVING_STORAGE,
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
  bool autoStoreSample();

  // Low level carriage/vertical drive control
  bool setCarriageSpeedMMps(float MMps);
  float getCarriageSpeedMMps() const;
  float getCarriageDepthMM() const;
  float getVerticalStepperCurrentA() const;
  bool currentSensorIsConnected() const;

  // Low level spiral motor control
  bool setSpiralRPM(float rpm);
  CubeMarsData getSpiralMotorData();

  //Storage control
  bool setCalibration0();
  bool setCalibrationX(float);

  bool storageMoveToSlot(StepperPositioner::StoragePosition position);
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

  bool deepSampleEncoderConnected();
  bool deepSampleStepperConnected();


private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;

  FastAccelStepperEngine _stepperEngine;
  
  DrillController _drillController;
  DeepSampleHolder _deepSampleHolder;

  AutoState _autoState = AutoState::MANUAL;
  float _targetDepthMM;

  uint32_t _storingStartTimeMS = 0;
  static constexpr uint32_t _storingDurationMS = 5000;
  uint8_t  _divideSlotIndex = 0;

  StepperPositioner::StoragePosition _divideSlots[3] = {
    StepperPositioner::StoragePosition::First,
    StepperPositioner::StoragePosition::Second,
    StepperPositioner::StoragePosition::Third
  };
};