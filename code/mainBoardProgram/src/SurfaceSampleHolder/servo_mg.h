#pragma once
 
#include <ESP32Servo.h>
 
class SERVO_MG {
public:
    SERVO_MG(int pin);
 
    bool begin();
    bool openBox();
    bool closeBox();
    bool setPos(uint8_t angle);
    uint8_t getPos();
    void update();
 
private:
    uint8_t _pin;
    uint8_t _currentPos;
    uint8_t _targetPos;
    bool  _moving;
    unsigned long _lastStepTime;
 
    static const uint8_t CLOSED_ANGLE  = 0;
    static const uint8_t OPEN_ANGLE    = 180;
    static const uint8_t STEP_DELAY_MS = 2;
 
    Servo _servo;
};