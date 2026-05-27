#pragma once
 
#include <ESP32Servo.h>
 
class SERVO_MG {
public:
    SERVO_MG(int pin);
 
    void begin();
    void openRockBox();
    void closeRockBox();
    void setPos(uint16_t angle);
    uint16_t getPos();
    void update();
 
private:
    int   _pin;
    int   _currentPos;
    int   _targetPos;
    bool  _moving;
    unsigned long _lastStepTime;
 
    static const int CLOSED_ANGLE  = 0;
    static const int OPEN_ANGLE    = 90;
    static const int STEP_DELAY_MS = 5;
 
    Servo _servo;
};