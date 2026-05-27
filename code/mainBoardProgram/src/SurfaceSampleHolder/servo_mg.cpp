#include "SERVO_MG.h"

SERVO_MG::SERVO_MG(int pin)
  : _pin(pin),
    _currentPos(CLOSED_ANGLE),
    _targetPos(CLOSED_ANGLE),
    _lastStepTime(0)
{}

bool SERVO_MG::begin() {
  _servo.setPeriodHertz(50);
  _servo.attach(_pin, 1000, 2000);
  _servo.write(_currentPos);
  return true;
}

bool SERVO_MG::openBox() {
  return setPos(OPEN_ANGLE);
}

bool SERVO_MG::closeBox() {
  return setPos(CLOSED_ANGLE);
}

bool SERVO_MG::setPos(uint8_t angle) {
  if (angle > 180) 
    angle = 180;
  if (angle != _currentPos) {
    _targetPos = angle;
  }
  return true;
}

uint8_t SERVO_MG::getPos() {
  return (uint8_t)_currentPos;
}

void SERVO_MG::update() {
  unsigned long now = millis();
  if (now - _lastStepTime < STEP_DELAY_MS) return;
  _lastStepTime = now;

  if (_currentPos < _targetPos) {
      _currentPos++;
  } else if (_currentPos > _targetPos) {
      _currentPos--;
  } else {
      return;
  }

  _servo.write(_currentPos);
}
