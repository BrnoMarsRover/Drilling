 #include "DeepSampler.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DeepSampler::DeepSampler(TwoWire& wire, HardwareSerial& debugSerial):
  _wire(wire),
  _debugSerial(debugSerial),
  _stepperEngine(),
  _drillController(wire, debugSerial, _stepperEngine),
  _deepSampleHolder(wire, debugSerial, _stepperEngine)
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
      if(!_deepSampleHolder.storageIsMoving())
      {
        if (_deepSampleHolder.storageGetCurrentSlot() == StepperPositioner::StoragePosition::Home)
        {
          if(_drillController.autoDrillToDepth(2, 60, _targetDepthMM ) )
          {
            _autoState = AutoState::DRILLING;
          }
          else
          {
            _autoState = AutoState::ERROR;
          }
        }
        else
        {
          _autoState = AutoState::ERROR;
        }
      }
      break;
    }
    
    case AutoState::DRILLING:
    {
      if(_drillController.getAutoState() == DrillController::AutoState::DONE)
      {
        if(_drillController.getCarriageDepthMM() < 50.0)
        {
          if(_deepSampleHolder.storageMoveToSlot(storeSlot))
          {
            _autoState = AutoState::MOVING_STORAGE; 
          }
          else
          {
            _autoState = AutoState::ERROR;
          }
        }
        else
        {
          _autoState = AutoState::ERROR;
        }
      }
      if(_drillController.getAutoState() == DrillController::AutoState::ERROR)
      {
        _autoState = AutoState::ERROR;
      }
      break;
    }
    
    case AutoState::MOVING_STORAGE:
    {
      if(!_deepSampleHolder.storageIsMoving())
      {
        if(_deepSampleHolder.storageGetCurrentSlot() == storeSlot)
        {
          if(_drillController.getCarriageDepthMM() < 50.0)
          {
            if(_drillController.setCarriageSpeedMMps(8))
            {
              _autoState = AutoState::MOVING_CARRIAGE_TO_STORE;
            }
            else
            {
              _autoState = AutoState::ERROR;
            }
          }
          else
          {
            _autoState = AutoState::ERROR;
          }
        }
        else
        {
          _autoState = AutoState::ERROR;
        }
      }

      break;
    }
    
    case AutoState::MOVING_CARRIAGE_TO_STORE:
    {
      if(_drillController.spiralDepthBelowSensor() > -5.0)
      {
        if(_drillController.setCarriageSpeedMMps(0))
        {
          if(_drillController.setSpiralRPM(-30))
          {
            _storingStartTimeMS = millis();
            _autoState = AutoState::STORING;
          }
          else
          {
            _autoState = AutoState::ERROR;
          }
        }
        else
        {
          _autoState = AutoState::ERROR;
        }
      }
      break;
    }
    
    case AutoState::STORING:
    {
      if(millis() > _storingStartTimeMS + _storingDurationMS)
      {
        if(_drillController.setSpiralRPM(0) && _deepSampleHolder.startAutoWeighing())
        {
          _autoState = AutoState::WEIGHING;
        }
        else
        {
          _autoState = AutoState::ERROR;
        }
      }
      break;
    }
    
    case AutoState::WEIGHING:
    {
      if(_deepSampleHolder.getAutoState() == DeepSampleHolder::AutoState::DONE)
      {
        if(_drillController.setCarriageSpeedMMps(-10.0))
        {
          _autoState = AutoState::MOVING_UP;
        }
        else
        {
          _autoState = AutoState::ERROR;
        }
      }
      if(_deepSampleHolder.getAutoState() == DeepSampleHolder::AutoState::ERROR)
      {
        _autoState = AutoState::ERROR;
      }
      break;
    }
    
    case AutoState::MOVING_UP:
    {
      if(_drillController.getCarriageDepthMM() == 0.0)
      {
        _autoState = AutoState::DONE;
      }
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

  _autoState = AutoState::MANUAL;
  return _drillController.setManualControl();
}
bool DeepSampler::startDistFromSurfaceMeasure() {return _drillController.startDistFromSurfaceMeasure(); }
float DeepSampler::getDistFromSurfaceMM() {return _drillController.getDistFromSurfaceMM(); }


bool DeepSampler::autoSampleAndWeigh(float targetDepthMM)
{
  if(_autoState == AutoState::MANUAL)
  {
    if(_deepSampleHolder.storageMoveToSlot(StepperPositioner::StoragePosition::Home))
    {
      _targetDepthMM = targetDepthMM;
      _autoState = AutoState::WAITING_FOR_STORAGE_CLEAR;
      return true;
    }
  }
}

// Low level carriage/vertical drive control
bool DeepSampler::setCarriageSpeedMMps(float MMps)
{
  return _drillController.setCarriageSpeedMMps(MMps);
}

float DeepSampler::getCarriageSpeedMMps() const { return _drillController.getCarriageSpeedMMps(); }
float DeepSampler::getCarriageDepthMM() const { return _drillController.getCarriageDepthMM(); }
float DeepSampler::getVerticalStepperCurrentA() const { return _drillController.getVerticalStepperCurrentA(); }
bool DeepSampler::currentSensorIsConnected() const {return _drillController.currentSensorIsConnected(); }

// Low level spiral motor control
bool DeepSampler::setSpiralRPM(float rpm) { return _drillController.setSpiralRPM(rpm); }

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

bool DeepSampler::storageMoveToSlot(StepperPositioner::StoragePosition position)
{
  _deepSampleHolder.storageMoveToSlot(position);
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