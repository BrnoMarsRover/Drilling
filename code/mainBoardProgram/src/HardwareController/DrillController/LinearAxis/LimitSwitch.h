#pragma once
#include <Arduino.h>

class LimitSwitch {
public:
  LimitSwitch(uint8_t pinNumber);

  void update();
  bool isPressed();
  bool wasPressed();
  bool wasReleased();

private:
  uint8_t switchPin;
  bool currentState;
  bool lastState;
  bool pressedEvent;
  bool releasedEvent;
};