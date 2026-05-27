#include "StepperPositioner.h"

// ---------------------------------------------------------------
//  Konstruktor
// ---------------------------------------------------------------
StepperPositioner::StepperPositioner(uint8_t stepPin,
                                       uint8_t dirPin,
                                       uint8_t enPin,
                                       uint8_t rxPin,
                                       uint8_t txPin,
                                       TwoWire& wire,
                                       FastAccelStepperEngine& stepperEngine,
                                       uint8_t uartPort)
    : _stepPin(stepPin),
      _dirPin(dirPin),
      _enPin(enPin),
      _rxPin(rxPin),
      _txPin(txPin),
      _wire(wire),
      _stepperEngine(stepperEngine),
      _uartPort(uartPort)
{}

// ---------------------------------------------------------------
//  begin()
// ---------------------------------------------------------------
bool StepperPositioner::begin(uint16_t rmsCurrent, uint16_t microsteps) {
    // --- TMC2209 UART ---
    _serialDriver->begin(115200, SERIAL_8N1, _rxPin, _txPin);

    _driver = new TMC2209Stepper(_serialDriver, 0.11f, 0b00);
    _driver->begin();
    _driver->toff(4);
    _driver->blank_time(24);
    _driver->rms_current(rmsCurrent);
    _driver->microsteps(microsteps);
    _driver->pdn_disable(true);
    _driver->I_scale_analog(false);
    _driver->en_spreadCycle(false);
    _driver->pwm_autoscale(true);

    _rmsCurrent = rmsCurrent;
    _stepsPerRevolution = (microsteps == 0) ? 200 : (200U * microsteps);

    _encoder = new AS5600L(_wire, 0x41);   // 0x36 = výchozí adresa AS5600L

    if (!_encoder->begin()) {
        _fatalError = true;
        Serial.println(F("[STORAGE] CHYBA: AS5600L nenalezen na I2C."));
        return false;
    }


    if (!_encoder->magnetDetected()) {
        Serial.println(F("[STORAGE] VAROVANI: Slaby nebo zadny magnet!"));
    }

    _encoder->setDirection(true);

    // --- FastAccelStepper ---
    _stepper = _stepperEngine.stepperConnectToPin(_stepPin);
    if (_stepper == nullptr) {
        _fatalError = true;
        Serial.println(F("[STORAGE] CHYBA: Nelze inicializovat stepper."));
        return false;
    }

    _stepper->setDirectionPin(_dirPin);
    _stepper->setEnablePin(_enPin);
    _stepper->setAutoEnable(true);
    _stepper->setSpeedInHz(3000);
    _stepper->setAcceleration(1500);


    setZeroOffsetDegrees(290); // konstanta podle natočení magentu vůči senzoru -> zaručí úhel nula na uprostřed pod roverem

    _initialized = true;
    Serial.print(F("[STORAGE] Inicializovano"));
    return true;
}

// ---------------------------------------------------------------
//  update() – volat každý loop()
// ---------------------------------------------------------------
void StepperPositioner::update() {
    // Enkodér musí být updateován každý loop pro správné multi-turn počítání
    if (_encoder != nullptr) {
        _encoder->update();
    }

    if (_fatalError || !_moving || _stepper == nullptr || _encoder == nullptr) return;

    // Motor doběhl
    if (!_stepper->isRunning()) {
        _moving    = false;
        _lastAngle = getCurrentAngle();
        Serial.print(F("[STORAGE] Dojeto. Aktualni uhel: "));
        Serial.print(_lastAngle);
        Serial.print(F("°  (slot "));
        Serial.print(getCurrentSlot());
        Serial.println(F(")"));
        _applyHoldState();
        return;
    }

    // Periodická kontrola polohy vůči enkodéru
    uint32_t now = millis();
    if (now - _lastCheckMs < STALL_CHECK_MS) return;
    _lastCheckMs = now;

    uint16_t realPos = getCurrentAngle();

    // Vypočítaná očekávaná poloha z kroků
    long deltaSteps   = _stepper->getCurrentPosition() - _moveStartSteps;
    uint16_t  expectedPos  = normalizeAngle(
        _moveStartAngle + (int)((deltaSteps * 360L) / _stepsPerRevolution)
    );

    int16_t err = shortestAngleDiff(expectedPos, realPos);
/*
    // --- Průběžný debug výpis ---
    Serial.print(F("[DBG] motor="));
    Serial.print(expectedPos);
    Serial.print(F("° enc="));
    Serial.print(realPos);
    Serial.print(F("° diff="));
    Serial.print(err);
    Serial.print(F("° | cil="));
    Serial.print(_targetAngle);
    Serial.print(F("° steps="));
    Serial.print(_stepper->getCurrentPosition());
    Serial.print(F(" deltaSteps="));
    Serial.println(deltaSteps);
    // ----------------------------
*/
    if (abs(err) > STALL_ANGLE_ERR_DEG) {
        Serial.print(F("[STORAGE] ZASEKUTI: ocekavano="));
        Serial.print(expectedPos);
        Serial.print(F("° skutecne="));
        Serial.print(realPos);
        Serial.print(F("° odchylka="));
        Serial.print(err);
        Serial.println(F("°"));
        handleStall();
        return;
    }

    _lastAngle = realPos;
}

// ---------------------------------------------------------------
//  moveToSlot()
// ---------------------------------------------------------------
bool StepperPositioner::moveToSlot(uint8_t slot) {
    if (slot < 1 || slot > NUM_POSITIONS) {
        Serial.print(F("[STORAGE] Neplatny slot: "));
        Serial.println(slot);
        return false;
    }

    uint16_t angle = POSITIONS[slot - 1];

    Serial.print(F("[STORAGE] Slot "));
    Serial.print(slot);
    Serial.print(F(" -> "));
    Serial.print(angle);
    Serial.println(F("°"));

    _retryCount = 0;
    return moveToAngle(angle);
}

// ---------------------------------------------------------------
//  moveToAngle()
// ---------------------------------------------------------------
bool StepperPositioner::moveToAngle(int16_t angleDeg) {
    if (_fatalError) {return false;}
    _retryCount = 0;
    _doMoveToAngle(angleDeg);
    return true;
}

void StepperPositioner::_doMoveToAngle(int16_t angleDeg) {
    if (_fatalError) {
        Serial.println(F("[STORAGE] CHYBA: System zablokovan. Pouzij unlock()."));
        return;
    }
    if (!_initialized || _stepper == nullptr || _encoder == nullptr) {
        Serial.println(F("[STORAGE] CHYBA: System neni inicializovan."));
        return;
    }

    _targetAngle = normalizeAngle(angleDeg);
    uint16_t currentEnc = getCurrentAngle();
    int16_t diff  = shortestAngleDiff(currentEnc, _targetAngle);

    if (abs(diff) <= 1) {
        Serial.println(F("[STORAGE] Cil uz dosazen."));
        return;
    }

    _moving          = true;
    _lastCheckMs     = millis() + STALL_CHECK_MS;   // první kontrola až po celém intervalu
    _lastAngle       = currentEnc;
    _moveStartSteps  = _stepper->getCurrentPosition();
    _moveStartAngle  = currentEnc;

    long stepsToMove = (long)diff * (long)_stepsPerRevolution / 360L;

    Serial.print(F("[STORAGE] Pohyb: "));
    Serial.print(currentEnc);
    Serial.print(F("° -> "));
    Serial.print(_targetAngle);
    Serial.print(F("°  ("));
    Serial.print(diff);
    Serial.print(F("°, "));
    Serial.print(stepsToMove);
    Serial.println(F(" kroku)"));

    // Před pohybem vždy zapnout motor bez ohledu na hold mód
    digitalWrite(_enPin, LOW);
    //_stepper->setAutoEnable(true);
    _stepper->move(stepsToMove);
}

// ---------------------------------------------------------------
//  stop()
// ---------------------------------------------------------------
void StepperPositioner::stop() {
    if (_stepper) _stepper->forceStop();
    _moving = false;
    Serial.println(F("[STORAGE] Zastaveno."));
    _applyHoldState(); 
}

void StepperPositioner::setZeroOffset(int32_t offsetCounts) {
    if (_encoder == nullptr) return;
    _encoder->setZeroOffset(offsetCounts);
}

void StepperPositioner::setZeroOffsetDegrees(float offsetDegrees) {
    if (_encoder == nullptr) return;

    int32_t counts = (int32_t)((offsetDegrees * 4096.0f) / 360.0f);
    _encoder->setZeroOffset(counts);
}

// ---------------------------------------------------------------
//  setZero()
// ---------------------------------------------------------------
void StepperPositioner::setZero() {
    if (_encoder == nullptr) return;
    _encoder->update();   // zajistit aktuální sample
    _encoder->setZero();  // nula přímo v enkodéru
    _lastAngle = 0;
    Serial.print(F("[STORAGE] Nulova poloha ulozena, offset = "));
    Serial.print(getZeroOffsetDegrees(), 2);
    Serial.println(F("°"));
}

// ---------------------------------------------------------------
//  unlock()
// ---------------------------------------------------------------
void StepperPositioner::unlock() {
    if (_fatalError) {
        _fatalError  = false;
        _retryCount  = 0;
        _moving      = false;
        Serial.println(F("[STORAGE] System odblokovam. Zadejte novou pozici."));
    } else {
        Serial.println(F("[STORAGE] System nebyl zablokovan."));
    }
}

// ---------------------------------------------------------------
//  getCurrentAngle()
// ---------------------------------------------------------------
int16_t StepperPositioner::getCurrentAngle() const {
    if (_encoder == nullptr) return 0;
    // getTotalAngleDegrees() vraci kumulativni uhel (muze byt >360 nebo zaporny)
    // pro porovnani slotu chceme 0-359
    float total = _encoder->getTotalAngleDegrees();
    return normalizeAngle((int)total);
}

// ---------------------------------------------------------------
//  getCurrentSlot() – nejbližší slot
// ---------------------------------------------------------------
uint8_t StepperPositioner::getCurrentSlot() const {
    uint16_t angle = getCurrentAngle();
    uint8_t nearestSlot = 1;
    uint16_t lastError = 360;

    for (uint8_t i = 0; i < NUM_POSITIONS; i++) {

        int16_t err = shortestAngleDiff(angle, POSITIONS[i]);

        if (abs(err) < lastError) {
            lastError = abs(err);
            nearestSlot = i + 1;
        }
    }

    if (lastError > SLOT_TOLERANCE) {
        return 9;
    }

    return nearestSlot;
}

// ---------------------------------------------------------------
//  getZeroOffset()
// ---------------------------------------------------------------
int32_t StepperPositioner::getZeroOffset() const {
    if (_encoder == nullptr) return 0;
    return _encoder->getZeroOffset();
}

// ---------------------------------------------------------------
//  getZeroOffsetDegrees()
// ---------------------------------------------------------------
float StepperPositioner::getZeroOffsetDegrees() const {
    if (_encoder == nullptr) return 0.0f;

    int32_t counts = _encoder->getZeroOffset();

    int32_t normalized = counts % 4096;
    if (normalized < 0) normalized += 4096;

    return ((float)normalized * 360.0f) / 4096.0f;
}

// ---------------------------------------------------------------
//  printStatus()
// ---------------------------------------------------------------
void StepperPositioner::printStatus(Stream& out) const {
    out.println(F("=== STEPPER POSITIONER ==="));
    out.print(F("Inicializovano : ")); out.println(_initialized ? F("ANO") : F("NE"));
    out.print(F("Fatal error    : ")); out.println(_fatalError  ? F("ANO") : F("NE"));
    out.print(F("Pohyb          : ")); out.println(_moving       ? F("ANO") : F("NE"));
    out.print(F("Aktualni uhel  : ")); out.print(getCurrentAngle()); out.println(F("°"));
    out.print(F("Aktualni slot  : ")); out.println(getCurrentSlot());
    out.print(F("Cilovy uhel    : ")); out.print(_targetAngle);   out.println(F("°"));
    out.print(F("Steps/rev      : ")); out.println(_stepsPerRevolution);
    if (_stepper) {
        out.print(F("Stepper pos    : ")); out.println(_stepper->getCurrentPosition());
    }
}

// ---------------------------------------------------------------
//  setHoldMode() / _applyHoldState()
// ---------------------------------------------------------------
void StepperPositioner::setHoldMode(bool hold) {
    _holdMode = hold;
    if (!_moving) {
        _applyHoldState();
    }
    Serial.print(F("[STORAGE] Hold mode: "));
    Serial.println(hold ? F("ON (drzi polohu)") : F("OFF (volny)"));
}

void StepperPositioner::_applyHoldState() {
    if (_driver == nullptr || _stepper == nullptr) return;
    _stepper->setAutoEnable(false);

    if (_holdMode) {
        // Plný hold proud – motor drží polohu, nelze otočit rukou
        _driver->ihold(20);          // 0..31, max hold proud
        _stepper->setAutoEnable(false);
        digitalWrite(_enPin, LOW);   // EN active-low = motor zapnutý
    } else {
        // Nulový proud – motor se odblokuje, otočit lze volně
        _driver->ihold(0);
        _stepper->setAutoEnable(false);
        digitalWrite(_enPin, HIGH);  // EN high = motor vypnutý
    }
}

// ---------------------------------------------------------------
//  handleStall() – interní uvolnění nebo fatal lock
// ---------------------------------------------------------------
void StepperPositioner::handleStall() {
    if (_stepper == nullptr) return;

    _stepper->forceStop();
    _retryCount++;

    if (_retryCount > MAX_RETRIES) {
        _moving     = false;
        _fatalError = true;
        Serial.println(F("[STORAGE] !!! MOTOR ZABLOKOVAN !!! Pouzij unlock()."));
        _applyHoldState(); 
        return;
    }

    Serial.print(F("[STORAGE] Pokus o uvolneni (retry "));
    Serial.print(_retryCount);
    Serial.println(F(")..."));

    // Znovu cílový úhel (bez resetování retry)
    _doMoveToAngle(_targetAngle);
}

// ---------------------------------------------------------------
//  Pomocné funkce
// ---------------------------------------------------------------
uint16_t StepperPositioner::normalizeAngle(int16_t a) {
    a %= 360;
    if (a < 0) a += 360;
    return a;
}

int16_t StepperPositioner::shortestAngleDiff(int16_t from, int16_t to)
{
    int16_t diff = (int16_t)normalizeAngle(to) - (int16_t)normalizeAngle(from);

    if (diff > 180) diff -= 360;
    if (diff <= -180) diff += 360;

    return diff;
}