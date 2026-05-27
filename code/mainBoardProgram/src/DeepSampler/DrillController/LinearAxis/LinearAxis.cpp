#include "LinearAxis.h"

LinearAxis::LinearAxis(uint8_t stepPin,
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
                       uint8_t encoderAddress,
                       float mmPerRevolution,
                       float rSense
                       )
    : _stepPin(stepPin),
      _dirPin(dirPin),
      _enPin(enPin),
      _csPin(csPin),
      _sckPin(sckPin),
      _misoPin(misoPin),
      _mosiPin(mosiPin),
      _limitTopPin(limitTopPin),
      _limitBottomPin(limitBottomPin),
      _encoderAddress(encoderAddress),
      _mmPerRevolution(mmPerRevolution),
      _rSense(rSense),
      _wire(wire),
      _stepperEngine(stepperEngine)
{
}

bool LinearAxis::begin(uint16_t rmsCurrent, uint16_t microsteps) {
    _limitTop = new LimitSwitch(_limitTopPin);
    _limitBottom = new LimitSwitch(_limitBottomPin);

    _encoder = new AS5600L(_wire, _encoderAddress);
    if (!_encoder->begin()) {
        Serial.println(F("[LINEAR] CHYBA: AS5600L nenalezen"));
        _fatalError = true;
    } else {
        Serial.println(F("[LINEAR] AS5600L OK"));
        _encoder->setZero();
    }

    _driver = new TMC5160Stepper(_csPin, _rSense);

    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    SPI.begin(_sckPin, _misoPin, _mosiPin, _csPin);
    delay(50);

    setupDriver();

    setupStepper();

    if (_stepper == nullptr) {
        _fatalError = true;
        _initialized = false;
        return false;
    }

    _driver->rms_current(rmsCurrent);
    _driver->microsteps(microsteps);

    if (microsteps == 0) {
        _stepsPerRevolution = 200;
    } else {
        _stepsPerRevolution = 200U * microsteps;
    }

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
    _stepper = _stepperEngine.stepperConnectToPin(_stepPin);

    if (_stepper == nullptr) {
        Serial.println(F("[LINEAR] CHYBA: stepperConnectToPin() selhalo"));
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
        Serial.println(F("[LINEAR] Horni koncak sepnut"));
    }

    // dolní koncák - zastaví pohyb dolů
    if (_limitBottom != nullptr && _limitBottom->wasPressed() && _motionState == Down) {
        stop();
        Serial.println(F("[LINEAR] Dolni koncak sepnut"));
    }

    _loadUnfiltered = getLoad();
    updateLoadFilter(_loadUnfiltered);

    if (_loadPrintEnabled) {
        uint32_t now = millis();
        if (now - _lastLoadPrintMs >= _loadPrintIntervalMs) {
            _lastLoadPrintMs = now;
            printLoad(Serial);
        }
    }

    if (_heightPrintEnabled) {
        uint32_t now = millis();
        if (now - _lastHeightPrintMs >= _heightPrintIntervalMs) {
            _lastHeightPrintMs = now;
            printDepth(Serial);
        }
    }

    if (_speedPrintEnabled) {
        uint32_t now = millis();
        if (now - _lastSpeedPrintMs >= _speedPrintIntervalMs) {
            _lastSpeedPrintMs = now;
            printSpeed(Serial);
        }
    }



    if (isMoving()) {
        long angleSteps = getAngleFromSteps();
        long angleEncoder = getAngleFromEncoder();
        //Serial.print("anglesteps");Serial.print(angleSteps);
        //Serial.print("angleencoder");Serial.println(angleEncoder);

        if (compareEncoderAndSteps(angleSteps, -angleEncoder)) {
            Serial.println(F("[LINEAR] VAROVANI: mozna ztrata kroku!"));
            stop();
        }
    }
    
}

void LinearAxis::moveUp() {
    if (_fatalError || !_initialized) return;

    if (isTopLimitPressed()) {
        Serial.println(F("[LINEAR] Horni koncak je sepnuty, nahoru nelze jet."));
        //stopAndZeroPosition(); //možny fix proč je potřeba davat dvakrát DOWN
        return;
    }

    setMotionState(Up);
}

void LinearAxis::moveDown() {
    if (_fatalError || !_initialized) return;

    if (isBottomLimitPressed()) {
        Serial.println(F("[LINEAR] Dolni koncak je sepnuty, dolu nelze jet."));
        //stop(); //možny fix proč je potřeba davat dvakrát DOWN
        return;
    }

    setMotionState(Down);
}

void LinearAxis::stop() {
    _motionState = Stop;

    if (_stepper != nullptr) {
        _stepper->forceStop();
        _speedHz = 0;
    }

    Serial.println(F("[LINEAR] Motor zastaven"));
}

void LinearAxis::zero() {
    if (_stepper != nullptr) {
        _stepper->setCurrentPosition(0);
    }

    if (_encoder != nullptr) {
        _encoder->setZero();
    }

    Serial.println(F("[LINEAR] Pozice a encoder nulovany"));
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

bool LinearAxis::setSpeedMMps(float mmPerSec) {
    //if (mmPerSec <= 0.0f) mmPerSec = 0.1f;

    float stepsPerSec = abs(mmPerSec) * ((float)_stepsPerRevolution / _mmPerRevolution);

    setSpeed((uint32_t)stepsPerSec);

    if(mmPerSec < 0)
    {
        moveUp();
    }
    else if (mmPerSec > 0)
    {
        moveDown();
    }
    else if (mmPerSec == 0)
    {
        stop();
    }

    return true;
}

float LinearAxis::getSpeedMMps() const {
    if (_stepsPerRevolution == 0) return 0.0f;

    return ((float)_speedHz * _mmPerRevolution) / (float)_stepsPerRevolution;
}

void LinearAxis::setSpeedMps(float mPerSec) {
    setSpeedMMps(mPerSec * 1000.0f);
}

float LinearAxis::getSpeedMps() const {
    return getSpeedMMps() / 1000.0f;
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

float LinearAxis::getDepthMM() const {
    if (_encoder == nullptr) return 0.0f;
    return _encoder->getLinearDistanceMM(_mmPerRevolution);
}

float LinearAxis::getDepthCM() const {
    return getDepthMM() / 10.0f;
}

float LinearAxis::getDepthM() const {
    return getDepthMM() / 1000.0f;
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

uint16_t LinearAxis::getLoad() const {
    if (_driver == nullptr) return 0;
    return _driver->sg_result();
}

float LinearAxis::getFilteredLoad() const {
    return _loadFiltered;
}

void LinearAxis::updateLoadFilter(uint16_t raw) {
    _loadUnfiltered = raw;
    _loadFiltered = _loadAlpha * _loadUnfiltered + (1 - _loadAlpha) * _loadFiltered;
}

void LinearAxis::printLoad(Stream& out) const {
    /*
    out.print(F("SG raw: "));
    out.print(_loadUnfiltered);
    out.print(F(" | SG filt: "));
    out.println(_loadFiltered);
    */
    Serial.print(getSpeedHz());
    Serial.print(',');
    Serial.print(_loadUnfiltered);
    Serial.print(',');
    Serial.println(_loadFiltered, 2);
}

void LinearAxis::setLoadPrintEnabled(bool enabled) {
    _loadPrintEnabled = enabled;
    _lastLoadPrintMs = millis();
}

void LinearAxis::printSpeed(Stream& out) const {
    out.print(F("Rychlost (mm/s): "));
    out.println(getSpeedMMps());
}

void LinearAxis::setSpeedPrintEnabled(bool enabled) {
    _speedPrintEnabled = enabled;
    _lastSpeedPrintMs = millis();
}

void LinearAxis::printDepth(Stream& out) const {
    out.print(F("Vyska (mm): "));
    out.println(getDepthMM());
}

void LinearAxis::setDepthPrintEnabled(bool enabled) {
    _heightPrintEnabled = enabled;
    _lastHeightPrintMs = millis();
}

long LinearAxis::getAngleFromSteps() const {
    if (_stepper == nullptr || _stepsPerRevolution == 0) return 0;

    long currentPosition = _stepper->getCurrentPosition();
    long angle = (long)(currentPosition * (360.0 / _stepsPerRevolution));
    return angle;
}

long LinearAxis::getAngleFromEncoder() const {
    if (_encoder == nullptr) return 0;

    // celkový úhel od setZero(), ne modulo 360
    return (long)_encoder->getTotalAngleDegrees();
}

bool LinearAxis::compareEncoderAndSteps(long angleSteps, long angleEncoder) const {
    long difference = labs(angleEncoder - angleSteps);

    if (difference > _stepCompareThresholdDeg) {
        return true;
    } else {
        return false;
    }
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
    out.println(getDepthMM(), 3);

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
    _stepper->applySpeedAcceleration();

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
        Serial.println(F("[LINEAR] Smer: NAHORU"));
    } else if (_motionState == Down) {
        Serial.println(F("[LINEAR] Smer: DOLU"));
    } else {
        Serial.println(F("[LINEAR] Motor STOP"));
    }

    applyMotion();
}

void LinearAxis::stopAndZeroPosition() {
    setMotionState(Stop);

    if (_stepper != nullptr) {
        _stepper->forceStop();
        _stepper->setCurrentPosition(0);
       _speedHz = 0;
    }

    if (_encoder != nullptr) {
        _encoder->setZero();
    }

    Serial.println(F("[LINEAR] Koncak sepnut -> motor zastaven, pozice nulovana"));
}
