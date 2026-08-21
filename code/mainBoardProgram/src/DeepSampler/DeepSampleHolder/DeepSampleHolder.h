#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>

#include "../../shared/ADS122C04_LIB.h" //ADS122C04_LIB/
#include "StepperPositioner.h"





class DeepSampleHolder
{
public:
  enum class AutoState : uint8_t
  {
    MANUAL = 0x00,
    STORAGE_MOVING = 0x01,
    WAITING_FOR_SETTLE = 0x02,
    WEIGHING = 0x03,
    DONE = 0xFE,
    ERROR = 0xFF
  };

  DeepSampleHolder(TwoWire& wire, HardwareSerial& debugSerial, FastAccelStepperEngine& stepperEngine);
  bool begin();

  void update();

  bool startAutoWeighing();
  AutoState getAutoState();
  bool setManualControl();

  bool requestMeasure();
  bool requestTemp();

  bool getResultReady();
  WeightResult getLastWeight();
  float getLastTemp();
  bool getAdcConnected();

  bool setTare();
  bool setCalibration0();
  bool setCalibrationX(float);

  bool storageMoveToAngle(int angleDeg);
  bool storageMoveToSlot(StepperPositioner::StoragePosition);
  bool storageUnlock();
  bool storageSetHoldMode(bool hold);
  bool storageIsHoldMode() const { return _stepperPositioner.isHoldMode(); }
  bool storageIsBlocked() const { return _stepperPositioner.hasFatalError(); }
  int16_t storageGetCurrentAngle()  const;
  StepperPositioner::StoragePosition storageGetCurrentSlot() const;
  bool storageIsMoving() const {return _stepperPositioner.isMoving(); }
  bool storageStop();

  bool storageEncoderConnected() const;
  bool storageStepperConnected();

private:
  TwoWire& _wire;
  HardwareSerial& _debugSerial;

  ADS122C04 _adcDeep;
  uint32_t _measureStateEntryTime = 0;
  uint32_t _settleDuration = 4000;
  FastAccelStepperEngine& _stepperEngine;

  // Piny pro StepperPositioner
  static constexpr uint8_t STEP_PIN  = 26;
  static constexpr uint8_t DIR_PIN   = 25;
  static constexpr uint8_t EN_PIN    = 27;
  static constexpr uint8_t RX_PIN    = 32;
  static constexpr uint8_t TX_PIN    = 33;

  StepperPositioner _stepperPositioner;

  AutoState _autoState = AutoState::MANUAL;

  void enterError();
};