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
      _debugSerial.println("auto waitingForClear");
      if(!_deepSampleHolder.storageIsMoving())
      {
        if(_deepSampleHolder.storageGetCurrentSlot() == 1)
        {
          if(_drillController.autoDrillToDepth(2, 60, _targetDepthMM ) )
          {
            _autoState = AutoState::DRILLING;
          }
          else
          {
            _debugSerial.println("auto failedToDrill");
            _autoState = AutoState::ERROR;
          }
        }
        else
        {
          _debugSerial.println("auto notCleared");
          _debugSerial.println(_deepSampleHolder.storageGetCurrentSlot());
          _autoState = AutoState::ERROR;
        }
      }
      break;
    }
    
    case AutoState::DRILLING:
    {
      _debugSerial.println("auto drilling");
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
      _debugSerial.println("auto movingStorage");
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
      _debugSerial.println("auto movingCarriageToStore");
      if(_drillController.getCarriageDepthMM() > 50.0)
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
      _debugSerial.println("auto storing");
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
      _debugSerial.println("auto weighing");
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
      _debugSerial.println("auto movingUp");
      if(_drillController.getCarriageDepthMM() == 0.0)
      {
        _autoState = AutoState::DONE;
      }
      break;
    }
    
    case AutoState::DONE:
    {
      _debugSerial.println("auto Done");
      break;
    }
    
    case AutoState::ERROR:
    {
      _debugSerial.println("auto error");
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
    if(_deepSampleHolder.storageMoveToSlot(1))
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