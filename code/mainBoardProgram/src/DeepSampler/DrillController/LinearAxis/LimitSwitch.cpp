#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(uint8_t pinNumber) {
  switchPin = pinNumber;
  pinMode(switchPin, INPUT);

  currentState = (digitalRead(switchPin) == LOW);
  lastState = currentState;
  pressedEvent = false;
  releasedEvent = false;
}

void LimitSwitch::update() {
  currentState = (digitalRead(switchPin) == LOW);

  pressedEvent = false;
  releasedEvent = false;

  if (currentState != lastState) {
    if (currentState) {
      pressedEvent = true;
    } else {
      releasedEvent = true;
    }
    lastState = currentState;
  }
}

bool LimitSwitch::isPressed() {
  return currentState;
}

bool LimitSwitch::wasPressed() {
  return pressedEvent;
}

bool LimitSwitch::wasReleased() {
  return releasedEvent;
}