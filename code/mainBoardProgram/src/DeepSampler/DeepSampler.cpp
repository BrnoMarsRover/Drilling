 #include "DeepSampler.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DeepSampler::DeepSampler(TwoWire& wire, HardwareSerial& debugSerial):
  _wire(wire),
  _debugSerial(debugSerial),
  _drillController(wire, debugSerial),
  _deepSampleHolder(wire)
{
  
}

bool DeepSampler::begin()
{
  if(_drillController.begin() && _deepSampleHolder.begin())
    return true;
  else
    return false;  
}

void DeepSampler::update()
{
  _drillController.update();
}

// Integrated carriage/spiral motor control

// Low level carriage/vertical drive control
bool DeepSampler::setCarriageSpeedMMps(float MMps)
{
  return _drillController.setCarriageSpeedMMps(MMps);
}

float DeepSampler::getCarriageHeightMM() { return _drillController.getCarriageHeightMM(); }

// Low level spiral motor control
bool DeepSampler::setSpiralRPM(float rpm)
{
  return _drillController.setSpiralRPM(rpm);
}

float DeepSampler::getSpiralRPM() { return _drillController.getSpiralRPM(); }
float DeepSampler::getSpiralMotorTmp() { return _drillController.getSpiralMotorTmp(); }

//Storage control
bool DeepSampler::setTareDeep() {_deepSampleHolder.setTare(); }
bool DeepSampler::setCalibration0Deep() {_deepSampleHolder.setCalibration0(); }
bool DeepSampler::setCalibration100Deep() {_deepSampleHolder.setCalibration100(); }

bool DeepSampler::getResultReadyDeep() { return _deepSampleHolder.getResultReady(); }
float DeepSampler::getLastWeightDeep() { return _deepSampleHolder.getLastWeight(); }
float DeepSampler::getLastTempDeep() { return _deepSampleHolder.getLastTemp(); }

bool DeepSampler::requestMeasureDeep() { return _deepSampleHolder.requestMeasure(); }
bool DeepSampler::requestTempDeep() { return _deepSampleHolder.requestTemp(); }

// Connection checks
bool DeepSampler::verticalEncoderConnected() {return _drillController.encoderConnected();}
bool DeepSampler::verticalStepperConnected() {return _drillController.stepperConnected();}
bool DeepSampler::spiralMotorConnected() {return _drillController.spiralMotorConnected();}
bool DeepSampler::heightSensorConneted() {return _drillController.heightSensorConnected();}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //