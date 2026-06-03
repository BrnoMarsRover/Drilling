#pragma once
 
#include <ESP32Servo.h>
 
class SERVO_MG {
public:
    SERVO_MG(int pin, uint8_t closed_ang, uint8_t open_ang);
 
    bool begin();
    bool openBox();
    bool closeBox();
    bool setPos(uint8_t angle);
    uint8_t getPos();
    void update();
 
private:
    uint8_t _pin;
    uint8_t _closed_ang;
    uint8_t _open_ang;
    uint8_t _currentPos;
    uint8_t _targetPos;
    bool  _moving;
    unsigned long _lastStepTime;

    static const uint8_t STEP_DELAY_MS = 1;
 
    Servo _servo;
};