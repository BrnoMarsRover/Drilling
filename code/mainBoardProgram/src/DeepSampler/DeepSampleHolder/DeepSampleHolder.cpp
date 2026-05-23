#include "DeepSampleHolder.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

DeepSampleHolder::DeepSampleHolder(TwoWire& wire) :
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

bool DeepSampleHolder::begin()
{
  bool beginOK = true;

  _adcDeep.begin();
  _adcDeep.task_start();

  return beginOK;
}

void DeepSampleHolder::requestMeasure()
{
  _adcDeep.request_measure();
}

bool DeepSampleHolder::getResultReady()
{
  return _adcDeep.get_result_ready();
}

float DeepSampleHolder::getLastWeight()
{
  return _adcDeep.get_last_weight();
}

void DeepSampleHolder::requestTemp()
{
  _adcDeep.request_tmp();
}

float DeepSampleHolder::getLastTemp()
{
  return _adcDeep.get_last_temp();
}

void DeepSampleHolder::setTare()
{
  _adcDeep.set_tare();
}

void DeepSampleHolder::setCalibration0()
{
  _adcDeep.set_calibration_0();
}

void DeepSampleHolder::setCalibration100()
{
  _adcDeep.set_calibration_100();
}

void DeepSampleHolder::reset()
{
  _adcDeep.reset();
}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //