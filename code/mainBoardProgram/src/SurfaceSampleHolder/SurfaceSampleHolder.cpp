#include "SurfaceSampleHolder.h"

// ------------------------------------------------------------------ //
//  Public                                                            //
// ------------------------------------------------------------------ //

SurfaceSampleHolder::SurfaceSampleHolder(TwoWire& wire, HardwareSerial& debugSerial):
  _wire(wire),
  _debugSerial(debugSerial)
  //adc?
  //servo?
{
}

// ------------------------------------------------------------------ //
//  Private                                                           //
// ------------------------------------------------------------------ //