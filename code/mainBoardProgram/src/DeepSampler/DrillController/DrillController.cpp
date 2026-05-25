#include "DrillController.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DrillController::DrillController(TwoWire& wire, HardwareSerial& debugSerial):
  _wire(wire),
  _debugSerial(debugSerial),
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

  if(_controlMode == AUTOMATIC)
  {
    switch(_autoState)
    {
      case WAITING_FOR_HEIGHT:
      {
        if(_heightSensor.dataReady())
        {
          if(spiralDepthBelowGroundMM() > -20.0)
          {
            _motorDriver.setRPM(_targetSpiralRPS*60);
            _autoState = DRILLING;
          }
          else
          {
            _autoState = MOVING_DOWN;
            _linearAxis.setSpeedMMps(10);
          }
        }

        break;
      }

      case MOVING_DOWN:
      {
        if(spiralDepthBelowGroundMM() > -20.0)
          _motorDriver.setRPM(_targetSpiralRPS*60);
          _autoState = DRILLING;
        break;
      }

      case DRILLING:
      {
        if(spiralDepthBelowGroundMM() > _targetDepthMM)
        {
          _linearAxis.setSpeedMMps(-10);
          _motorDriver.setRPM(0);
          _autoState = MOVING_UP;
        }
        _linearAxis.setSpeedMMps((_motorDriver.getRPM()/60)*_rateOfPenetrationMMpRev);
        break;
      }

      case MOVING_UP:
      {
        if(_linearAxis.getDepthMM() == 0.0)
        {
          _autoState = DONE;
        }
        break;
      }
      
      case DONE:
      {
        break;
      }
      
      case ERROR:
      {
        break;
      }
    }
  }
}

bool DrillController::setCarriageSpeedMMps(float MMps)
{
  if (_controlMode == MANUAL)
  {
    _linearAxis.setSpeedMMps(MMps);
    return true;
  }
  else
    return false;
}

float DrillController::getCarriageSpeedMMps() const { return _linearAxis.getSpeedMMps(); }

float DrillController::getCarriageDepthMM()
{
  return _linearAxis.getDepthMM();
}

bool DrillController::setSpiralRPM(float rpm)
{
  if (_controlMode == MANUAL)
  {
    _motorDriver.setRPM(rpm);
    return true;
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
  setSpiralRPM(0);
  setCarriageSpeedMMps(0);
  _controlMode = MANUAL;
  return true;
}

bool DrillController::autoDrillToDepth(float rateOfPenetrationMMpRev, float targetRPM, float targetDepthMM)
{
  if (_controlMode == MANUAL)
  {
    _controlMode = AUTOMATIC;

    _rateOfPenetrationMMpRev = rateOfPenetrationMMpRev;
    _targetSpiralRPS = targetRPM/60.0;
    _targetDepthMM = targetDepthMM + 15;

    _heightSensor.startMeasure();
    _autoState = WAITING_FOR_HEIGHT;
    return true;
  }
  else
    return false;
}

ControlMode DrillController::getControlMode() {return _controlMode;}
AutoState DrillController::getAutoState() {return _autoState;}

// Connection checks
bool DrillController::encoderConnected() {return false;}
bool DrillController::stepperConnected() {return false;}
bool DrillController::spiralMotorConnected() {return _motorDriver.isConnected(); }
bool DrillController::heightSensorConnected() {return _heightSensor.isConnected(); }

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //