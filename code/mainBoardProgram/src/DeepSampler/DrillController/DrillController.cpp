#include "DrillController.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DrillController::DrillController(TwoWire& wire, HardwareSerial& debugSerial,  FastAccelStepperEngine& stepperEngine):
  _wire(wire),
  _debugSerial(debugSerial),
   _stepperEngine(stepperEngine),
  // ===== LINEAR AXIS =====
  _linearAxis(
    13, // STEP
    12, // DIR
    14, // EN
    5,  // CS (TMC5160)
    18, // SCK
    19, // MISO
    23, // MOSI
    35, // horní koncák
    34,  // dolní koncák
    wire,  //i2c bus class
    stepperEngine,
    0x42 // adresa AS5600
  ),
  _motorDriver(Serial2, debugSerial, 16, 17),
  _heightSensor(wire)
{
}

bool DrillController::begin()
{
  bool beginOK = true;
  if (!_linearAxis.begin(600, 16)) {
    _debugSerial.println("Linear axis FAILED");
    beginOK = false;
  }
  if (!_heightSensor.begin()) {
    _debugSerial.println("TOF FAILED");
    beginOK = false;
  }
  if(!_motorDriver.begin())
  {
    _debugSerial.println("MOTOR FAILED");
    beginOK = false;
  }

  return beginOK;
}

void DrillController::update()
{
  _linearAxis.update();
  _motorDriver.update();
  _heightSensor.update();


  switch(_autoState)
  {
    case AutoState::MANUAL:
    {
      break;
    }

    case AutoState::WAITING_FOR_HEIGHT:
    {
      _debugSerial.println("drillAuto waitingForheight");
      if(_heightSensor.dataReady())
      {
        if(spiralDepthBelowGroundMM() > -20.0)
        {
          if(_motorDriver.setRPM(_targetSpiralRPS*60))
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
          _autoState = AutoState::MOVING_DOWN;
          _linearAxis.setSpeedMMps(10);
        }
      }

      break;
    }

    case AutoState::MOVING_DOWN:
    {
      if(spiralDepthBelowGroundMM() > -20.0)
      {
        if(_motorDriver.setRPM(_targetSpiralRPS*60))
        {
          _autoState = AutoState::DRILLING;
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
      if(spiralDepthBelowGroundMM() > _targetDepthMM)
      {
        if(_linearAxis.setSpeedMMps(-10.0) && _motorDriver.setRPM(0.0))
        {
          _autoState = AutoState::MOVING_UP;
        }
        else
        {
          _autoState = AutoState::ERROR;
        }
      }
      else
      {
        if(!(_linearAxis.setSpeedMMps((_motorDriver.getRPM()/60)*_rateOfPenetrationMMpRev)))
        {
          _autoState = AutoState::ERROR;
        }
      }
      break;
    }

    case AutoState::MOVING_UP:
    {
      if(_linearAxis.getDepthMM() == 0.0)
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

bool DrillController::setCarriageSpeedMMps(float MMps)
{
  if (_autoState == AutoState::MANUAL)
  {
    _linearAxis.setSpeedMMps(MMps);
    return true;
  }
  else
    return false;
}

float DrillController::getCarriageSpeedMMps() const { return _linearAxis.getSpeedMMps(); }

float DrillController::getCarriageDepthMM() const { return _linearAxis.getDepthMM(); }

float DrillController::getVerticalStepperCurrentA() const { return _linearAxis.getStepperCurrentA(); }

bool DrillController::currentSensorIsConnected() const {return _linearAxis.currentSensorIsConnected(); }

bool DrillController::setSpiralRPM(float rpm)
{
  if (_autoState == AutoState::MANUAL)
  {
    return _motorDriver.setRPM(rpm);
  }
  else
    return false;
}

float DrillController::getSpiralRPM()
{
  return _motorDriver.getRPM();
}

float DrillController::getSpiralMotorTmp()
{
  if(_motorDriver.getMOSTmp() > _motorDriver.getMotorTmp())
  {
    return _motorDriver.getMOSTmp();
  }
  else
  {
    return _motorDriver.getMotorTmp();
  }
}

// Integrated drill control
DrillController::AutoState DrillController::getAutoState()
{
  if(_autoState == AutoState::DONE)
  {
    _autoState = AutoState::MANUAL;
    return AutoState::DONE;
  }
  else
  {
    return _autoState;
  }
}

bool DrillController::startDistFromSurfaceMeasure()
{
  return _heightSensor.startMeasure();
}

float DrillController::getDistFromSurfaceMM()
{
  return _heightSensor.getDistanceMM();
}

bool DrillController::setManualControl()
{
  _autoState = AutoState::MANUAL;
  _motorDriver.setRPM(0);
  _linearAxis.setSpeedMMps(0);
  return true;
}

bool DrillController::autoDrillToDepth(float rateOfPenetrationMMpRev, float targetRPM, float targetDepthMM)
{
  if (_autoState == AutoState::MANUAL)
  {
    _rateOfPenetrationMMpRev = rateOfPenetrationMMpRev;
    _targetSpiralRPS = targetRPM/60.0;
    _targetDepthMM = targetDepthMM + 15.0; //additional 15 mm to account for potential spilling

    _heightSensor.startMeasure();
    _autoState = AutoState::WAITING_FOR_HEIGHT;
    return true;
  }
  else
    return false;
}

// Connection checks
bool DrillController::encoderConnected() {return false;}
bool DrillController::stepperConnected() {return false;}
bool DrillController::spiralMotorConnected() {return _motorDriver.isConnected(); }
bool DrillController::heightSensorConnected() {return _heightSensor.isConnected(); }

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //