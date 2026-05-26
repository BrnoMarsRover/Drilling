#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#include "../../../shared/as5600.h"
#include "LimitSwitch.h"

class LinearAxis {
public:
    // STEP/DIR/EN pro stepper
    // CS/SCK/MISO/MOSI pro TMC5160 přes SPI
    // limitTopPin / limitBottomPin pro koncáky
    // sdaPin / sclPin / encoderAddress pro AS5600L
    // mmPerRevolution = mechanický převod osy
    LinearAxis(uint8_t stepPin,
               uint8_t dirPin,
               uint8_t enPin,
               uint8_t csPin,
               uint8_t sckPin,
               uint8_t misoPin,
               uint8_t mosiPin,
               uint8_t limitTopPin,
               uint8_t limitBottomPin,
               TwoWire& wire,
               FastAccelStepperEngine& stepperEngine,
               uint8_t encoderAddress = 0x40,
               float mmPerRevolution = 2.0f,
               float rSense = 0.075f
               );

    bool begin(uint16_t rmsCurrent = 600, uint16_t microsteps = 16);
    void update();

    // pohyb
    void moveUp();
    void moveDown();
    void stop();
    void zero();

    // nastavení pohybu
    void setSpeed(uint32_t speedHz);
    void setAcceleration(uint32_t accelHz);
    void changeSpeedRelative(int32_t deltaHz);

    // stav
    bool isMoving() const;
    bool isMovingUp() const;
    bool isMovingDown() const;
    bool isStopped() const;

    bool isTopLimitPressed() const;
    bool isBottomLimitPressed() const;

    int32_t getStepperPosition() const;
    float getDepthMM() const;
    float getDepthCM() const;
    float getDepthM() const;
    void printDepth(Stream& out) const;
    void setDepthPrintEnabled(bool enabled);

    uint32_t getSpeedHz() const;
    uint32_t getAccelerationHz() const;

    bool hasFatalError() const;
    void printStatus(Stream& out) const;

    uint16_t getLoad() const;
    void updateLoadFilter(uint16_t raw);
    float getFilteredLoad() const;
    void printLoad(Stream& out) const;
    void setLoadPrintEnabled(bool enabled);

    long getAngleFromSteps() const;
    long getAngleFromEncoder() const;
    bool compareEncoderAndSteps(long angleSteps, long angleEncoder) const;

    void setSpeedMMps(float mmPerSec);
    float getSpeedMMps() const;
    void printSpeed(Stream& out) const;
    void setSpeedPrintEnabled(bool enabled);

    void setSpeedMps(float mPerSec);
    float getSpeedMps() const;

private:
    enum MotionState : int8_t {
        Stop = 0,
        Up   = 1,
        Down = -1
    };

    void setupDriver();
    void setupStepper();
    void applyMotion();
    void setMotionState(MotionState state);
    void stopAndZeroPosition();

    uint8_t _stepPin;
    uint8_t _dirPin;
    uint8_t _enPin;

    uint8_t _csPin;
    uint8_t _sckPin;
    uint8_t _misoPin;
    uint8_t _mosiPin;

    uint8_t _limitTopPin;
    uint8_t _limitBottomPin;

    uint8_t _sdaPin;
    uint8_t _sclPin;
    uint8_t _encoderAddress;

    float _mmPerRevolution;
    float _rSense;

    uint32_t _speedHz = 500;
    uint32_t _accelHz = 2000;
    MotionState _motionState = Stop;

    bool _initialized = false;
    bool _fatalError = false;
    bool _loadPrintEnabled = false;
    bool _speedPrintEnabled = false;
    bool _heightPrintEnabled = false;

    uint32_t _loadPrintIntervalMs = 0;
    uint32_t _speedPrintIntervalMs = 300;
    uint32_t _heightPrintIntervalMs = 300;
    uint32_t _lastLoadPrintMs = 0;
    uint32_t _lastSpeedPrintMs = 0;
    uint32_t _lastHeightPrintMs = 0;

    long _stepCompareThresholdDeg = 90;
    bool _stepComparePrintEnabled = false;

    uint16_t _stepsPerRevolution = 3200;

    TwoWire& _wire;

    TMC5160Stepper* _driver = nullptr;
    FastAccelStepper* _stepper = nullptr;
    AS5600L* _encoder = nullptr;
    LimitSwitch* _limitTop = nullptr;
    LimitSwitch* _limitBottom = nullptr;

    uint16_t _loadUnfiltered = 0;
    float _loadFiltered = 0;
    float _loadAlpha = 0.1;

    FastAccelStepperEngine& _stepperEngine;
};
