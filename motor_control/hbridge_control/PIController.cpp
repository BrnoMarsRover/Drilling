#include "PIController.h"
#include "Arduino.h"

PIController::PIController(float aKp, float aKi, float aOutputLimit, float aSumClamp)
{
  errorSum = 0;
  setpoint = 0;
  Kp = aKp;
  Ki = aKi;
  outputLimit = aOutputLimit;
  sumClamp = aSumClamp;
}

float PIController::compute(float aInput, float aTs)
{
  float error = setpoint - aInput;
  errorSum = constrain(errorSum + (aTs * error), -sumClamp, sumClamp);
  return constrain(Kp * error + Ki * errorSum, -outputLimit, outputLimit);
}

float PIController::getErrorSum()
{
  return errorSum;
}

void PIController::reset()
{
  errorSum = 0;
  setpoint = 0;
  return;
}