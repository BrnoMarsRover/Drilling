#include "SERVO_MG.h"

SemaphoreHandle_t SERVO_MG::_servoMutex = nullptr;
bool SERVO_MG::_mutexCreated = false;

SERVO_MG::SERVO_MG(int pin, uint8_t closed_ang, uint8_t open_ang)
  : _pin(pin),
    _closed_ang(closed_ang),
    _open_ang(open_ang),
    _currentPos(closed_ang),
    _targetPos(closed_ang),
    _moving(false),
    _taskHandle(nullptr)
{}

bool SERVO_MG::begin() {
  if (!_mutexCreated) {
    _servoMutex   = xSemaphoreCreateMutex();
    _mutexCreated = true;
  }

  _servo.setPeriodHertz(50);
  _servo.attach(_pin, 500, 2500);
  _servo.write(_currentPos);
  _task_start();
  return true;
}

bool SERVO_MG::openBox() {
  return setPos(_open_ang);
}

bool SERVO_MG::closeBox() {
  return setPos(_closed_ang);
}

bool SERVO_MG::setPos(uint8_t angle) {
  if (angle > 180) 
    angle = 180;
  if (angle != _currentPos) {
    if (!_servo.attached()) {
        _servo.attach(_pin, 500, 2500);  // re-acquire LEDC channel
      }
    _targetPos = angle;
    _moving = true;
    _job_active = true;
  }
  return true;
}

uint8_t SERVO_MG::getPos() {
  return (uint8_t)_currentPos;
}

void SERVO_MG::update() {
  if (!_moving) return;

  if (_currentPos < _targetPos) {
    _currentPos++;
  } else if (_currentPos > _targetPos) {
    _currentPos--;
  } else {
    _servo.detach();
    _moving = false;
    _job_active = false;
    return;
  }
 
  if (xSemaphoreTake(_servoMutex, pdMS_TO_TICKS(2))) {
    _servo.write(_currentPos);
    xSemaphoreGive(_servoMutex);
  }
}

void SERVO_MG::_task_start() {
  char taskName[16];
  snprintf(taskName, sizeof(taskName), "SERVO_pin:%d", _pin); // dynamic "SERVO_pin:14" OR "SERVO_pin:4"

  xTaskCreatePinnedToCore(
    _servoTask,               // fnc to run
    taskName,
    3072,                   // stack size in bytes
    this,                   // pvParameters
    1,                      // priority low
    &_taskHandle,      // handle stored here not used now
    0                       // loop() runs on core 1
  );
}

void SERVO_MG::_servoTask(void* pvParameters) {
  SERVO_MG* self = static_cast<SERVO_MG*>(pvParameters);

  while (true) {
    if (self->_job_active == true) {
      self->update();
    }
      //UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL); // NULL refers to this task; to erase after testing
      //Serial.print("[_servoTask] remaining words till overflow: "); // to erase after testing
      //Serial.println(hwm); // to erase after testing; i wish for this to be max 1000 words left
    vTaskDelay(pdMS_TO_TICKS(TICK_MS));
  }
}
