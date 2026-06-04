#include "SurfaceSampleHolder.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

SurfaceSampleHolder::SurfaceSampleHolder(TwoWire& wire, HardwareSerial& debugSerial):
  _wire(wire),
  _debugSerial(debugSerial),
  _adcSurface(
    wire,
    0x45  // address for surface sample weight
  ),
  _servoRock(
    4,  // pin 
    180, // closed angle
    10 // open angle
    ),
  _servoSand(
    15,
    10, 
    180
    )
{
}
void SurfaceSampleHolder::update()
{
  _servoRock.update();
  _servoSand.update();
}

uint8_t SurfaceSampleHolder::getPosRock()
{
  return _servoRock.getPos();
}

uint8_t SurfaceSampleHolder::getPosSand()
{
  return _servoSand.getPos();
}

bool SurfaceSampleHolder::openRockBox()
{
  return _servoRock.openBox();
}

bool SurfaceSampleHolder::openSandBox()
{
  return _servoSand.openBox();
}

bool SurfaceSampleHolder::closeRockBox()
{
  return _servoRock.closeBox();
}

bool SurfaceSampleHolder::closeSandBox()
{
  return _servoSand.closeBox();
}

bool SurfaceSampleHolder::begin()
{
  _adcSurface.begin();
  _adcSurface.task_start();

  _servoRock.begin();
  _servoSand.begin();

  return true;
}

bool SurfaceSampleHolder::requestMeasure()
{
  _adcSurface.request_measure();
  return 1;
}

bool SurfaceSampleHolder::getResultReady()
{
  return _adcSurface.get_result_ready();
}

WeightResult SurfaceSampleHolder::getLastWeight()
{
  return _adcSurface.get_last_weight();
}

bool SurfaceSampleHolder::requestTemp()
{
  _adcSurface.request_tmp();
  return 1;
}

float SurfaceSampleHolder::getLastTemp()
{
  return _adcSurface.get_last_temp();
}

bool SurfaceSampleHolder::getAdcConnected()
{
  return _adcSurface.get_adc_connected();
}

bool SurfaceSampleHolder::setTare()
{
  _adcSurface.set_tare();
  return true;
}

bool SurfaceSampleHolder::setCalibration0()
{
  _adcSurface.set_calibration_0();
  return true;
}

bool SurfaceSampleHolder::setCalibrationX(float weightX)
{
  _adcSurface.set_calibration_100(weightX);
  return true;
}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //