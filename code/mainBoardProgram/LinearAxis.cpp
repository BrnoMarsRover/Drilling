#include "LinearAxis.h"

FastAccelStepperEngine LinearAxis::_engine = FastAccelStepperEngine();

LinearAxis::LinearAxis(uint8_t stepPin,
                       uint8_t dirPin,
                       uint8_t enPin,
                       uint8_t csPin,
                       uint8_t sckPin,
                       uint8_t misoPin,
                       uint8_t mosiPin,
                       uint8_t limitTopPin,
                       uint8_t limitBottomPin,
                       uint8_t sdaPin,
                       uint8_t sclPin,
                       uint8_t encoderAddress,
                       float mmPerRevolution,
                       float rSense)
    : _stepPin(stepPin),
      _dirPin(dirPin),
      _enPin(enPin),
      _csPin(csPin),
      _sckPin(sckPin),
      _misoPin(misoPin),
      _mosiPin(mosiPin),
      _limitTopPin(limitTopPin),
      _limitBottomPin(limitBottomPin),
      _sdaPin(sdaPin),
      _sclPin(sclPin),
      _encoderAddress(encoderAddress),
      _mmPerRevolution(mmPerRevolution),
      _rSense(rSense) {
}

bool LinearAxis::begin(uint16_t rmsCurrent, uint16_t microsteps) {
    _limitTop = new LimitSwitch(_limitTopPin);
    _limitBottom = new LimitSwitch(_limitBottomPin);

    _encoder = new AS5600L(_encoderAddress);
    if (!_encoder->begin(_sdaPin, _sclPin, 400000)) {
        Serial.println(F("CHYBA: AS5600L nenalezen"));
        _fatalError = true;
    } else {
        Serial.println(F("AS5600L OK"));
        _encoder->setZero();
    }

    _driver = new TMC5160Stepper(_csPin, _rSense);

    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    SPI.begin(_sckPin, _misoPin, _mosiPin, _csPin);
    delay(50);

    setupDriver();

    _engine.init();
    setupStepper();

    if (_stepper == nullptr) {
        _fatalError = true;
        _initialized = false;
        return false;
    }

    _driver->rms_current(rmsCurrent);
    _driver->microsteps(microsteps);

    _initialized = true;
    return !_fatalError;
}

void LinearAxis::setupDriver() {
    if (_driver == nullptr) return;

    _driver->begin();
    _driver->toff(5);
    _driver->rms_current(600);
    _driver->microsteps(16);

    // převzato z původního kódu
    _driver->en_pwm_mode(false);
}

void LinearAxis::setupStepper() {
    _stepper = _engine.stepperConnectToPin(_stepPin);

    if (_stepper == nullptr) {
        Serial.println(F("CHYBA: stepperConnectToPin() selhalo"));
        return;
    }

    _stepper->setDirectionPin(_dirPin, true);
    _stepper->setEnablePin(_enPin, true);
    _stepper->setAutoEnable(true);
    _stepper->setDelayToEnable(50);
    _stepper->setDelayToDisable(100);

    _stepper->setAcceleration(_accelHz);
    _stepper->setSpeedInHz(_speedHz);
    _stepper->setCurrentPosition(0);
    _stepper->stopMove();
}

void LinearAxis::update() {
    if (!_initialized) return;

    if (_encoder != nullptr) {
        _encoder->update();
    }

    if (_limitTop != nullptr) {
        _limitTop->update();
    }

    if (_limitBottom != nullptr) {
        _limitBottom->update();
    }

    // horní koncák - zastaví pohyb nahoru a vynuluje pozici
    if (_limitTop != nullptr && _limitTop->wasPressed() && _motionState == Up) {
        stopAndZeroPosition();
        Serial.println(F("Horni koncak sepnut"));
    }

    // dolní koncák - zastaví pohyb dolů
    if (_limitBottom != nullptr && _limitBottom->wasPressed() && _motionState == Down) {
        stop();
        Serial.println(F("Dolni koncak sepnut"));
    }
}

void LinearAxis::moveUp() {
    if (_fatalError || !_initialized) return;

    if (isTopLimitPressed()) {
        Serial.println(F("Horni koncak je sepnuty, nahoru nelze jet."));
        stopAndZeroPosition();
        return;
    }

    setMotionState(Up);
}

void LinearAxis::moveDown() {
    if (_fatalError || !_initialized) return;

    if (isBottomLimitPressed()) {
        Serial.println(F("Dolni koncak je sepnuty, dolu nelze jet."));
        stop();
        return;
    }

    setMotionState(Down);
}

void LinearAxis::stop() {
    _motionState = Stop;

    if (_stepper != nullptr) {
        _stepper->forceStop();
    }

    Serial.println(F("Motor zastaven"));
}

void LinearAxis::zero() {
    if (_stepper != nullptr) {
        _stepper->setCurrentPosition(0);
    }

    if (_encoder != nullptr) {
        _encoder->setZero();
    }

    Serial.println(F("Pozice a encoder nulovany"));
}

void LinearAxis::setSpeed(uint32_t speedHz) {
    if (speedHz < 1) speedHz = 1;
    _speedHz = speedHz;

    if (_stepper != nullptr) {
        _stepper->setSpeedInHz(_speedHz);
        _stepper->applySpeedAcceleration();
    }
}

void LinearAxis::setAcceleration(uint32_t accelHz) {
    if (accelHz < 1) accelHz = 1;
    _accelHz = accelHz;

    if (_stepper != nullptr) {
        _stepper->setAcceleration(_accelHz);
        _stepper->applySpeedAcceleration();
    }
}

void LinearAxis::changeSpeedRelative(int32_t deltaHz) {
    int32_t newSpeed = (int32_t)_speedHz + deltaHz;
    if (newSpeed < 1) newSpeed = 1;
    setSpeed((uint32_t)newSpeed);
}

bool LinearAxis::isMoving() const {
    return _motionState != Stop;
}

bool LinearAxis::isMovingUp() const {
    return _motionState == Up;
}

bool LinearAxis::isMovingDown() const {
    return _motionState == Down;
}

bool LinearAxis::isStopped() const {
    return _motionState == Stop;
}

bool LinearAxis::isTopLimitPressed() const {
    return digitalRead(_limitTopPin) == LOW;
}

bool LinearAxis::isBottomLimitPressed() const {
    return digitalRead(_limitBottomPin) == LOW;
}

int32_t LinearAxis::getStepperPosition() const {
    if (_stepper == nullptr) return 0;
    return _stepper->getCurrentPosition();
}

float LinearAxis::getHeightMM() const {
    if (_encoder == nullptr) return 0.0f;
    return _encoder->getLinearDistanceMM(_mmPerRevolution);
}

float LinearAxis::getHeightCM() const {
    return getHeightMM() / 10.0f;
}

float LinearAxis::getHeightM() const {
    return getHeightMM() / 1000.0f;
}

uint32_t LinearAxis::getSpeedHz() const {
    return _speedHz;
}

uint32_t LinearAxis::getAccelerationHz() const {
    return _accelHz;
}

bool LinearAxis::hasFatalError() const {
    return _fatalError;
}

void LinearAxis::printStatus(Stream& out) const {
    out.println(F("=== LINEAR AXIS ==="));
    out.print(F("Rychlost [Hz]: "));
    out.println(_speedHz);

    out.print(F("Akcelerace [steps/s^2]: "));
    out.println(_accelHz);

    out.print(F("Smer: "));
    if (_motionState == Up) {
        out.println(F("NAHORU"));
    } else if (_motionState == Down) {
        out.println(F("DOLU"));
    } else {
        out.println(F("STOP"));
    }

    out.print(F("Stepper pozice: "));
    out.println(getStepperPosition());

    out.print(F("Vyska [mm]: "));
    out.println(getHeightMM(), 3);

    out.print(F("Horni koncak: "));
    out.println(isTopLimitPressed() ? F("SEPNUT") : F("ROZEPNUT"));

    out.print(F("Dolni koncak: "));
    out.println(isBottomLimitPressed() ? F("SEPNUT") : F("ROZEPNUT"));

    out.print(F("Fatal error: "));
    out.println(_fatalError ? F("ANO") : F("NE"));
}

void LinearAxis::applyMotion() {
    if (_stepper == nullptr) return;

    _stepper->setSpeedInHz(_speedHz);
    _stepper->setAcceleration(_accelHz);

    if (_motionState == Up) {
        _stepper->runForward();
    } else if (_motionState == Down) {
        _stepper->runBackward();
    } else {
        _stepper->stopMove();
    }
}

void LinearAxis::setMotionState(MotionState state) {
    _motionState = state;

    if (_motionState == Up) {
        Serial.println(F("Smer: NAHORU"));
    } else if (_motionState == Down) {
        Serial.println(F("Smer: DOLU"));
    } else {
        Serial.println(F("Motor STOP"));
    }

    applyMotion();
}

void LinearAxis::stopAndZeroPosition() {
    setMotionState(Stop);

    if (_stepper != nullptr) {
        _stepper->forceStop();
        _stepper->setCurrentPosition(0);
    }

    if (_encoder != nullptr) {
        _encoder->setZero();
    }

    Serial.println(F("Koncak sepnut -> motor zastaven, pozice nulovana"));
}
