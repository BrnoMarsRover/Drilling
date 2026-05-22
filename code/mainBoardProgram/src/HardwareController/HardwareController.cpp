 #include "HardwareController.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

HardwareController::HardwareController(TwoWire& wire, HardwareSerial& debugSerial):
  _wire(wire),
  _debugSerial(debugSerial),
  _drillController(wire, debugSerial),
  _deepSample(wire)
{
  
}

bool HardwareController::begin()
{
  if(_drillController.begin() && _deepSample.begin())
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
void HardwareController::setTareDeep() {_deepSample.setTare(); }
void HardwareController::setCalibration0Deep() {_deepSample.setCalibration0(); }
void HardwareController::setCalibration100Deep() {_deepSample.setCalibration100(); }

float HardwareController::getCarriageHeightMM() { return _drillController.getCarriageHeightMM(); }
float HardwareController::getSpiralRPM() { return _drillController.getSpiralRPM(); }
float HardwareController::getSpiralMotorTmp() { return _drillController.getSpiralMotorTmp(); }
bool HardwareController::getResultReadyDeep() { return _deepSample.getResultReady(); }
float HardwareController::getLastWeightDeep() { return _deepSample.getLastWeight(); }
float HardwareController::getLastTempDeep() { return _deepSample.getLastTemp(); }

void HardwareController::requestMeasureDeep() { return _deepSample.requestMeasure(); }
void HardwareController::requestTempDeep() { return _deepSample.requestTemp(); }

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //