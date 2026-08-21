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
  enum class AutoState : uint8_t
  {
    MANUAL = 0x00,
    WAITING_FOR_STORAGE_CLEAR = 0x01,
    DRILLING = 0x02,
    MOVE_CARRIAGE_TO_STORE = 0x03,
    MOVING_CARRIAGE_TO_STORE = 0x04,
    MOVING_STORAGE = 0x05,
    STORING = 0x06,
    WEIGHING = 0x07,
    MOVING_UP = 0x08,
    DONE = 0xFE,
    ERROR = 0xFF
  };

  DeepSampler(TwoWire& wire, HardwareSerial& debugSerial);
  bool begin();
  void update();

  // High level control/autonomy
  AutoState getAutoState();
  DrillController::AutoState getDrillControllerState();
  DeepSampleHolder::AutoState getDeepSampleHolderState();
  bool setManualControl();
  bool startDistFromSurfaceMeasure();
  float getDistFromSurfaceMM();
  bool autoDrillStoreWeigh(float targetDepthMM);
  bool autoStoreWeigh();

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
  void enterError();
  float _targetDepthMM;

  uint32_t _storingStartTimeMS = 0;
  const float _storingRPM = -40.0; 
  static constexpr uint32_t _storingDurationMS = 2000;
  static constexpr float storingPositionTop = -25.0;
  static constexpr float storingPositionBottom = -20.0;

  uint8_t  _divideSlotIndex = 0;
  StepperPositioner::StoragePosition _divideSlots[3] = {
    StepperPositioner::StoragePosition::First,
    StepperPositioner::StoragePosition::Second,
    StepperPositioner::StoragePosition::Third
  };
};