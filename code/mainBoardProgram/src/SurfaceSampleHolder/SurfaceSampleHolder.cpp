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
  )
  //servo?
{
}

bool SurfaceSampleHolder::begin()
{
  bool beginOK = true;

  _adcSurface.begin();
  _adcSurface.task_start();

  return beginOK;
}

bool SurfaceSampleHolder::requestMeasure()
{
  _adcSurface.request_measure();
  return 1;
}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //