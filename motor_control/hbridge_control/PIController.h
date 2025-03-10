#ifndef PIController_H
#define PIController_H

#include "Arduino.h"

class PIController
{
public:
  float setpoint;
  float Kp;
  float Ki;
  float outputLimit;
  float sumClamp;

  PIController(float Kp, float Ki, float outputLimit, float sumClamp);

  float compute(float input, float Ts);

  float getErrorSum();

  void reset();

private:
  float errorSum;
};


#endif