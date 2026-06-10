#pragma once
 
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
 
class SERVO_MG {
public:
  SERVO_MG(int pin, uint8_t closed_ang, uint8_t open_ang);
 
  bool begin();
  bool openBox();
  bool closeBox();
  bool setPos(uint8_t angle);
  uint8_t getPos();
  void update();

  TaskHandle_t _taskHandle;
 
private:
  uint8_t _pin;
  uint8_t _closed_ang;
  uint8_t _open_ang;
  uint16_t _currentPos;
  uint16_t _targetPos;
  bool _moving = false;
  bool _job_active = false;

  static const uint32_t TICK_MS = 20;   // FreeRTOS task interval
 
  void _task_start();
  static SemaphoreHandle_t _servoMutex;
  static bool              _mutexCreated;
  static void _servoTask(void* pvParameters);
 
  Servo _servo;
};