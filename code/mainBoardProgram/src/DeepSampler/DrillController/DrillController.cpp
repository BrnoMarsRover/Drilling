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
    15, // horní koncák
    0,  // dolní koncák
    wire,  //i2c bus class
    0x42 // adresa AS5600
  ),
  _motorDriver(Serial2, debugSerial, 16, 17),
  _distanceSensor(wire)
{
}

bool DrillController::begin()
{
  bool beginOK = true;
  if (!_linearAxis.begin(600, 16)) {
    _debugSerial.println("Linear axis FAILED");
    beginOK = false;
  }
  if (!_distanceSensor.begin()) {
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
}

bool DrillController::setCarriageSpeedMMps(float MMps)
{
  _linearAxis.setSpeedMMps(MMps);
  return true;
}

bool DrillController::setSpiralRPM(float rpm)
{
  _motorDriver.setRPM(rpm);
  return true;
}

float DrillController::getCarriageHeightMM()
{
  return _linearAxis.getHeightMM();
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

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //