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
bool DeepSampler::setCalibration0() {_deepSampleHolder.setCalibration0(); }
bool DeepSampler::setCalibrationX(float weightX) {_deepSampleHolder.setCalibrationX(weightX); }

bool DeepSampler::getResultReady() { return _deepSampleHolder.getResultReady(); }
WeightResult DeepSampler::getLastWeight() { return _deepSampleHolder.getLastWeight(); }
float DeepSampler::getLastTemp() { return _deepSampleHolder.getLastTemp(); }

bool DeepSampler::requestMeasure() { return _deepSampleHolder.requestMeasure(); }
bool DeepSampler::requestTemp() { return _deepSampleHolder.requestTemp(); }

// Connection checks
bool DeepSampler::verticalEncoderConnected() {return _drillController.encoderConnected();}
bool DeepSampler::verticalStepperConnected() {return _drillController.stepperConnected();}
bool DeepSampler::spiralMotorConnected() {return _drillController.spiralMotorConnected();}
bool DeepSampler::heightSensorConneted() {return _drillController.heightSensorConnected();}
bool DeepSampler::getAdcConnecred() {return _deepSampleHolder.getAdcConnected();}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //