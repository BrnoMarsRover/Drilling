 #include "DeepSampler.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DeepSampler::DeepSampler(TwoWire& wire, HardwareSerial& debugSerial):
  _wire(wire),
  _debugSerial(debugSerial),
  _stepperEngine(),
  _drillController(wire, debugSerial, _stepperEngine),
  _deepSampleHolder(wire, _stepperEngine)
{
  
}

bool DeepSampler::begin()
{
  _stepperEngine.init();

  if(_drillController.begin() && _deepSampleHolder.begin())
    return true;
  else
    return false;  
}

void DeepSampler::update()
{
  _drillController.update();
  _deepSampleHolder.update();

  switch(_autoState)
  {
    case AutoState::MANUAL:
    {
      break;
    }

    case AutoState::WAITING_FOR_STORAGE_CLEAR:
    {
      break;
    }
    
    case AutoState::DRILLING:
    {
      break;
    }
    
    case AutoState::MOVING_STORAGE:
    {
      break;
    }
    
    case AutoState::MOVING_CARRIAGE_TO_STORE:
    {
      break;
    }
    
    case AutoState::STORING:
    {
      break;
    }
    
    case AutoState::WEIGHING:
    {
      break;
    }
    
    case AutoState::MOVING_UP:
    {
      break;
    }
    
    case AutoState::DONE:
    {
      break;
    }
    
    case AutoState::ERROR:
    {
      break;
    }
  }
}

// Integrated carriage/spiral motor control
DeepSampler::AutoState DeepSampler::getAutoState() {return _autoState; }
bool DeepSampler::setManualControl()
{
  _drillController.setManualControl();
  _autoState = AutoState::MANUAL;
  return true;
}
bool DeepSampler::startDistFromSurfaceMeasure() {return _drillController.startDistFromSurfaceMeasure(); }
float DeepSampler::getDistFromSurfaceMM() {return _drillController.getDistFromSurfaceMM(); }
bool DeepSampler::drillSetManualControl() {return _drillController.setManualControl(); }
bool DeepSampler::autoDrillToDepth(float rateOfPenetrationMMpRev, float targetRPM, float targetDepthMM) {return _drillController.autoDrillToDepth(rateOfPenetrationMMpRev, targetRPM, targetDepthMM); }

// Low level carriage/vertical drive control
bool DeepSampler::setCarriageSpeedMMps(float MMps)
{
  return _drillController.setCarriageSpeedMMps(MMps);
}

float DeepSampler::getCarriageSpeedMMps() const { return _drillController.getCarriageSpeedMMps(); }
float DeepSampler::getCarriageDepthMM() { return _drillController.getCarriageDepthMM(); }

// Low level spiral motor control
bool DeepSampler::setSpiralRPM(float rpm)
{
  return _drillController.setSpiralRPM(rpm);
}

float DeepSampler::getSpiralRPM() { return _drillController.getSpiralRPM(); }
float DeepSampler::getSpiralMotorTmp() { return _drillController.getSpiralMotorTmp(); }

//Storage control
bool DeepSampler::setCalibration0() { return _deepSampleHolder.setCalibration0(); }
bool DeepSampler::setCalibrationX(float weightX) { return _deepSampleHolder.setCalibrationX(weightX); }

bool DeepSampler::getResultReady() { return _deepSampleHolder.getResultReady(); }
WeightResult DeepSampler::getLastWeight() { return _deepSampleHolder.getLastWeight(); }
float DeepSampler::getLastTemp() { return _deepSampleHolder.getLastTemp(); }

bool DeepSampler::requestMeasure() { return _deepSampleHolder.requestMeasure(); }
bool DeepSampler::requestTemp() { return _deepSampleHolder.requestTemp(); }

bool DeepSampler::storageMoveToSlot(uint8_t slot)
{
  _deepSampleHolder.storageMoveToSlot(slot);
  return true;
}

bool DeepSampler::storageUnlock()
{
  _deepSampleHolder.storageUnlock();
  return true;
}

bool DeepSampler::storageSetHoldMode(bool hold)
{
  _deepSampleHolder.storageSetHoldMode(hold);
  return true;
}

uint16_t DeepSampler::storageGetCurrentAngle()  const {return _deepSampleHolder.storageGetCurrentAngle(); }

// Connection checks
bool DeepSampler::verticalEncoderConnected() {return _drillController.encoderConnected();}
bool DeepSampler::verticalStepperConnected() {return _drillController.stepperConnected();}
bool DeepSampler::spiralMotorConnected() {return _drillController.spiralMotorConnected();}
bool DeepSampler::heightSensorConnected() {return _drillController.heightSensorConnected();}
bool DeepSampler::getAdcConnected() {return _deepSampleHolder.getAdcConnected();}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //