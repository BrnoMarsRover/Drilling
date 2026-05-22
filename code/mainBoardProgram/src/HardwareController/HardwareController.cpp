 #include "HardwareController.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

HardwareController::HardwareController(TwoWire& wire, HardwareSerial& debugSerial):
  _wire(wire),
  _debugSerial(debugSerial),
  _drillController(wire, debugSerial)
{
  
}

bool HardwareController::begin()
{
  if(_drillController.begin())
    return true;
  else
    return false;
  //later, _wire will be initialized here.
  
}

void HardwareController::update()
{
  _drillController.update();
}

bool HardwareController::setCarriageSpeedMMps(float MMps)
{
  return _drillController.setCarriageSpeedMMps(MMps);
}

bool HardwareController::setSpiralRPM(float rpm)
{
  return _drillController.setSpiralRPM(rpm);
}
float HardwareController::getCarriageHeightMM() { return _drillController.getCarriageHeightMM(); }
float HardwareController::getSpiralRPM() { return _drillController.getSpiralRPM(); }
float HardwareController::getSpiralMotorTmp() { return _drillController.getSpiralMotorTmp(); }

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //