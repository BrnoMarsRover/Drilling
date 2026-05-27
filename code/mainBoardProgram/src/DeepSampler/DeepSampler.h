#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <FastAccelStepper.h>

#include "DrillController/DrillController.h"
#include "DeepSampleHolder/DeepSampleHolder.h"

enum DeepSamplerAutoState
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

class DeepSampler
{
public:
  DeepSampler(TwoWire& wire, HardwareSerial& debugSerial);
  bool begin();
  void update();

  // High level control/autonomy
  bool startDistFromSurfaceMeasure();
  float getDistFromSurfaceMM();
  bool drillSetManualControl();
  bool autoDrillToDepth(float rateOfPenetrationMMpRev, float targetRPM, float targetDepthMM);

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
  bool setCalibrationX(float weightX = 100);

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

  DeepSamplerAutoState _autoState = MANUAL;
};