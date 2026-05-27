#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "../../shared/as5600.h"

// ---------------------------------------------------------------
//  StepperPositioner
//  Otáčí zásobníkem na zadanou pozici (slot 1..NUM_SLOTS).
//  Enkodér AS5600L ověřuje pohyb; při zaseknutí provede 1 retry,
//  pak zablokuje motor a nastaví fatalError.
// ---------------------------------------------------------------

class StepperPositioner {
public:
    // Piny: STEP, DIR, EN, TMC2209 RX, TMC2209 TX, SDA, SCL
    // uartPort: 1 nebo 2 (HardwareSerial na ESP32)
    StepperPositioner(uint8_t stepPin,
                       uint8_t dirPin,
                       uint8_t enPin,
                       uint8_t rxPin,
                       uint8_t txPin,
                       TwoWire& wire,
                       FastAccelStepperEngine& stepperEngine,
                       uint8_t uartPort = 2);

    // Inicializace – volejte v setup()
    // rmsCurrent: RMS proud v mA (např. 600)
    // microsteps: mikrokroky TMC2209 (8, 16, 32 …)
    bool begin(uint16_t rmsCurrent, uint16_t microsteps);

    // Volat každý loop() – řídí detekci zaseknutí
    void update();

    // Otočit na slot 1..numSlots
    bool moveToSlot(uint8_t slot);

    // Přímé zadání úhlu 0..359°
    bool moveToAngle(int16_t angleDeg);

    // Nastavit nulový bod na aktuální polohu enkodéru
    void setZero();

    // Odblokovát po fatal erroru
    void unlock();

    // Zastavit motor okamžitě
    void stop();

    // Držení polohy po zastavení.
    // hold=true  → motor drží pozici (plný proud), nelze otočit rukou
    // hold=false → motor bez proudu po zastavení, otočit lze volně
    // Pohyb funguje v obou módech stejně.
    void setHoldMode(bool hold);
    bool isHoldMode() const { return _holdMode; }

    // Gettery stavu
    bool    isMoving()      const { return _moving; }
    bool    hasFatalError() const { return _fatalError; }
    bool    isInitialized() const { return _initialized; }
    int16_t     getCurrentAngle()  const;
    uint16_t     getTargetAngle()   const { return _targetAngle; }
    uint8_t getCurrentSlot()   const;   // nejbližší slot k aktuální pozici
    int32_t getZeroOffset() const;
    float getZeroOffsetDegrees() const;
    void setZeroOffset(int32_t offsetCounts);
    void setZeroOffsetDegrees(float offsetDegrees);
    void printStatus(Stream& out) const;


private:
    // interní pohyb (bez resetu retry)
    void _doMoveToAngle(int16_t angleDeg);
    void _applyHoldState();

    void handleStall();
    static uint16_t  normalizeAngle(int16_t a);
    static int16_t  shortestAngleDiff(int16_t from, int16_t to);

    // Piny
    uint8_t _stepPin, _dirPin, _enPin;
    uint8_t _rxPin,   _txPin;
    uint8_t _sdaPin,  _sclPin;
    uint8_t _uartPort;

    // Periferie
    HardwareSerial*      _serialDriver = &Serial1;
    TMC2209Stepper*      _driver       = nullptr;
    FastAccelStepper*    _stepper      = nullptr;
    AS5600L*             _encoder      = nullptr;
    TwoWire&        _wire;

    FastAccelStepperEngine& _stepperEngine;

    // Stav
    bool     _initialized = false;
    bool     _moving      = false;
    bool     _fatalError  = false;
    bool     _holdMode    = true;   // true=drzi, false=volny
    uint16_t _rmsCurrent  = 600;

    uint16_t _targetAngle    = 0;
    uint16_t _lastAngle      = 0;
    uint16_t _retryCount     = 0;
    uint32_t _lastCheckMs    = 0;

    uint16_t _stepsPerRevolution = 3200;

    long _moveStartSteps = 0;
    int  _moveStartAngle = 0;

    // Prahové hodnoty pro detekci zaseknutí
    static constexpr int  STALL_ANGLE_ERR_DEG = 15;   // °  – max. odchylka
    static constexpr int  STALL_CHECK_MS      = 200;  // ms – interval kontroly
    static constexpr int  MAX_RETRIES         = 1;    // počet pokusů uvolnění

    static constexpr uint16_t SLOT_TOLERANCE = 3;

    private:
    static constexpr uint8_t NUM_POSITIONS = 4;

    static constexpr int POSITIONS[NUM_POSITIONS] = {
        0,
        30,
        60,
        150
    };
};