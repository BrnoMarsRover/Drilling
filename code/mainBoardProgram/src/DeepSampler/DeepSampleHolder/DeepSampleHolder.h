#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>

#include "../../shared/ADS122C04_LIB.h" //ADS122C04_LIB/
#include "StepperPositioner.h"





class DeepSampleHolder
{
public:
  enum class AutoState
  {
    MANUAL,
    STORAGE_MOVING,
    WEIGHING,
    DONE,
    ERROR
  };

  DeepSampleHolder(TwoWire& wire, FastAccelStepperEngine& stepperEngine);
  bool begin();

  void update();

  bool startAutoWeighing();
  AutoState getAutoState();

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
  bool storageMoveToSlot(uint8_t slot);
  bool storageUnlock();
  bool storageSetHoldMode(bool hold);
  bool storageIsHoldMode() const { return _stepperPositioner.isHoldMode(); }
  bool storageIsBlocked() const { return _stepperPositioner.hasFatalError(); }
  int16_t storageGetCurrentAngle()  const;
  uint8_t storageGetCurrentSlot() const;
  bool storageIsMoving() const {return _stepperPositioner.isMoving(); }

private:
  TwoWire& _wire;

  ADS122C04 _adcDeep;
  FastAccelStepperEngine& _stepperEngine;

  // Piny pro StepperPositioner
  static constexpr uint8_t STEP_PIN  = 26;
  static constexpr uint8_t DIR_PIN   = 25;
  static constexpr uint8_t EN_PIN    = 27;
  static constexpr uint8_t RX_PIN    = 32;
  static constexpr uint8_t TX_PIN    = 33;

  StepperPositioner _stepperPositioner;

  AutoState _autoState = AutoState::MANUAL;
};