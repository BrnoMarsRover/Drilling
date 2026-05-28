#include "DeepSampleHolder.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DeepSampleHolder::DeepSampleHolder(TwoWire& wire, HardwareSerial& debugSerial, FastAccelStepperEngine& stepperEngine) :
  _wire(wire),
  _debugSerial(debugSerial),
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
  _adcDeep.begin();
  _adcDeep.task_start();
  _stepperPositioner.begin(1000, 16);  // rmsCurrent, microsteps

  return true;
}

void DeepSampleHolder::update(){
  _stepperPositioner.update();

  switch(_autoState){
    case AutoState::MANUAL:
    {

    }
    case AutoState::STORAGE_MOVING:
    {
      if(!storageIsMoving()) {
        if (storageGetCurrentSlot() == StepperPositioner::StoragePosition::Second) {
          if(requestMeasure()) {
            _autoState = AutoState::WEIGHING;
          }
          else{
            _autoState = AutoState::ERROR;
          }
        }
        else {
          _autoState = AutoState::ERROR;
        }
      }
      break;
    }
    case AutoState::WEIGHING:
    {
      if (getResultReady()) {
        _autoState = AutoState::DONE;
      }
      break;
    }
    case AutoState::DONE:
    {
    }
    case AutoState::ERROR:
    {

    }
  }
}

DeepSampleHolder::AutoState DeepSampleHolder::getAutoState(){
  if(_autoState == AutoState::DONE){
    _autoState = AutoState::MANUAL;
    return AutoState::DONE;
  }
  else{
    return _autoState;
  }
}


bool DeepSampleHolder::startAutoWeighing() {
  if(storageMoveToSlot(StepperPositioner::StoragePosition::Second)){
    _autoState = AutoState::STORAGE_MOVING;
    return true;
  }
  else {
    return false;
  }
}

bool DeepSampleHolder::storageMoveToAngle(int angleDeg) {
  if(storageIsBlocked()){
    return false;
  }
  else {
    _stepperPositioner.moveToAngle(angleDeg);
    return true;
  }
 
}

bool DeepSampleHolder::storageMoveToSlot(StepperPositioner::StoragePosition position) {
  if(storageIsBlocked()){
    return false;
  }
  else {
    _stepperPositioner.moveToSlot(position);
    return true;
  }
}

bool DeepSampleHolder::storageUnlock() {
  _stepperPositioner.unlock();
}

bool DeepSampleHolder::storageSetHoldMode(bool hold) {
  _stepperPositioner.setHoldMode(hold);
  return true;
}

int16_t DeepSampleHolder::storageGetCurrentAngle() const {
  return _stepperPositioner.getCurrentAngle();
}

StepperPositioner::StoragePosition DeepSampleHolder::storageGetCurrentSlot() const {
  return _stepperPositioner.getCurrentSlot();
}

bool DeepSampleHolder::requestMeasure()
{
  _adcDeep.request_measure();
  return true;
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

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //