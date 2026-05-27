#include "SERVO_MG.h"

SERVO_MG::SERVO_MG(int pin)
    : _pin(pin),
      _currentPos(CLOSED_ANGLE),
      _targetPos(CLOSED_ANGLE),
      _lastStepTime(0)
{}

void SERVO_MG::begin() {
    _servo.setPeriodHertz(50);
    _servo.attach(_pin, 1000, 2000);
    _servo.write(_currentPos);
}

void SERVO_MG::openRockBox() {
    setPos(OPEN_ANGLE);
}

void SERVO_MG::closeRockBox() {
    setPos(CLOSED_ANGLE);
}

void SERVO_MG::setPos(uint16_t angle) {
    if (angle > 180) angle = 180;
    if ((int)angle != _currentPos) {
        _targetPos = (int)angle;
    }
}

uint16_t SERVO_MG::getPos() {
    return (uint16_t)_currentPos;
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
