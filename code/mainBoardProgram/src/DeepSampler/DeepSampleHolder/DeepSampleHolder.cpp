#include "DeepSampleHolder.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DeepSampleHolder::DeepSampleHolder(TwoWire& wire, FastAccelStepperEngine& stepperEngine) :
  _wire(wire),
  _stepperEngine(stepperEngine),
  _adcDeep(
    wire,
    0x44  // address for deep sample weight
  ),
  _stepperPositioner(STEP_PIN, DIR_PIN, EN_PIN, RX_PIN, TX_PIN, wire, stepperEngine)

{
}

bool DeepSampleHolder::begin()
{
  bool beginOK = true;

  _adcDeep.begin();
  _adcDeep.task_start();
  _stepperPositioner.begin(1000, 16);  // rmsCurrent, microsteps

  return beginOK;
}

void DeepSampleHolder::update(){
  _stepperPositioner.update();
}

void DeepSampleHolder::storageMoveToAngle(int angleDeg) {
  _stepperPositioner.moveToAngle(angleDeg);
}

void DeepSampleHolder::storageMoveToSlot(uint8_t slot) {
  _stepperPositioner.moveToSlot(slot);
}

void DeepSampleHolder::storageUnlock() {
  _stepperPositioner.unlock();
}

void DeepSampleHolder::storageSetHoldMode(bool hold) {
  _stepperPositioner.setHoldMode(hold);
}

int DeepSampleHolder::storageGetCurrentAngle() const {
  return _stepperPositioner.getCurrentAngle();
}

bool DeepSampleHolder::requestMeasure()
{
  _adcDeep.request_measure();
  return 1;
}

bool DeepSampleHolder::getResultReady()
{
  return _adcDeep.get_result_ready();
}

WeightResult DeepSampleHolder::getLastWeight()
{
  return _adcDeep.get_last_weight();
}

bool DeepSampleHolder::requestTemp()
{
  _adcDeep.request_tmp();
  return 1;
}

float DeepSampleHolder::getLastTemp()
{
  return _adcDeep.get_last_temp();
}

bool DeepSampleHolder::getAdcConnected()
{
  return _adcDeep.get_adc_connected();
}

bool DeepSampleHolder::setTare()
{
  _adcDeep.set_tare();
  return true;
}

bool DeepSampleHolder::setCalibration0()
{
  _adcDeep.set_calibration_0();
  return true;
}

bool DeepSampleHolder::setCalibrationX(float weightX)
{
  _adcDeep.set_calibration_100(weightX);
  return true;
}

void DeepSampleHolder::reset()
{
  _adcDeep.reset();
}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //