#include <Arduino.h>

#include "LimitSwitch.h"

//PRIVATE

//PUBLIC
LimitSwitch::LimitSwitch(uint8_t pinNumber) : switchPin(pinNumber)
{
  pinMode(switchPin, INPUT);
}

bool LimitSwitch::isPressed()
{
  return digitalRead(switchPin);
}