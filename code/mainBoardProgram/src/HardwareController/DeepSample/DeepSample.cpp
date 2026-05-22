#include "DeepSample.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DeepSample::DeepSample(TwoWire& wire) :
  _wire(wire),
  _adcDeep(
    wire,
    0x44  // address for deep sample weight
  )/*, // will be added later
  _adcSurface(
    wire,
    0x45  // address for surface sample weight
  )*/
{
}

bool DeepSample::begin()
{
  bool beginOK = true;

  _adcDeep.begin();
  _adcDeep.task_start();

  return beginOK;
}

void DeepSample::requestMeasure()
{
  _adcDeep.request_measure();
}

bool DeepSample::getResultReady()
{
  return _adcDeep.get_result_ready();
}

float DeepSample::getLastWeight()
{
  return _adcDeep.get_last_weight();
}

void DeepSample::requestTemp()
{
  _adcDeep.request_tmp();
}

float DeepSample::getLastTemp()
{
  return _adcDeep.get_last_temp();
}

void DeepSample::setTare()
{
  _adcDeep.set_tare();
}

void DeepSample::setCalibration0()
{
  _adcDeep.set_calibration_0();
}

void DeepSample::setCalibration100()
{
  _adcDeep.set_calibration_100();
}

void DeepSample::reset()
{
  _adcDeep.reset();
}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //