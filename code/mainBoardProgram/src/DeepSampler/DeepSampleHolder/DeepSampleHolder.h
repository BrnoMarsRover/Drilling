#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../../shared/ADS122C04_LIB.h" //ADS122C04_LIB/
#include "StepperPositioner.h"

class DeepSampleHolder
{
public:
  DeepSampleHolder(TwoWire& wire);
  bool begin();

  bool requestMeasure();
  bool requestTemp();

  bool getResultReady();
  WeightResult getLastWeight();
  float getLastTemp();
  bool getAdcConnected();

  bool setTare();
  bool setCalibration0();
  bool setCalibrationX(float);
  void reset();

  void storageMoveToAngle(int angleDeg);
  void storageMoveToSlot(uint8_t slot);
  void storageUnlock();
  void storageSetHoldMode(bool hold);
  bool storageIsHoldMode() const { return _stepperPositioner.isHoldMode(); }
  int  storageGetCurrentAngle()  const;
  bool storageIsMoving() const {return _stepperPositioner.isMoving(); }

private:
  TwoWire& _wire;

  ADS122C04 _adcDeep;
  StepperPositioner _stepperPositioner;

  // Piny pro StepperPositioner
  static constexpr uint8_t STEP_PIN  = 26;
  static constexpr uint8_t DIR_PIN   = 25;
  static constexpr uint8_t EN_PIN    = 27;
  static constexpr uint8_t RX_PIN    = 16;
  static constexpr uint8_t TX_PIN    = 17;

};