#pragma once

class LimitSwitch
{
private:
  uint8_t switchPin;
public:
  LimitSwitch(uint8_t pinNumber);

  bool isPressed();
};