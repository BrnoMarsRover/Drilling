#ifndef PIController_H
#define PIController_H

#include "Arduino.h"

class PIController
{
public:
  float setpoint;

  PIController(float Kp, float Ki, float outputLimit, float sumClamp);

  float compute(float input, float Ts);

  float getErrorSum();

  void reset();

private:
  float errorSum;

  float Kp;
  float Ki;
  float outputLimit;
  float sumClamp;
};


#endif