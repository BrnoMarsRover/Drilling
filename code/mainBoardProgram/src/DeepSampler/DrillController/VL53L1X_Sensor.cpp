#include "VL53L1X_Sensor.h"

VL53L1X_Sensor::VL53L1X_Sensor(TwoWire &wire, uint8_t address)
    : _wire(&wire),
      _address(address),
      _lastDistance(0),
      _initialized(false),
      _measuring(false),
      _dataReady(false),
      _sampleCount(0),
      _sampleSum(0),
      _averagedDistance(0.0f),
      _samplePending(false)
{
}

bool VL53L1X_Sensor::begin() {
    _sensor.setBus(_wire);
    _sensor.setTimeout(500);

    if (!isConnected()) {
        return false;
    }

    if (!_sensor.init()) {
        return false;
    }

    _sensor.setDistanceMode(VL53L1X::Long);
    _sensor.setMeasurementTimingBudget(50000);

    _initialized = true;
    return true;
}

void VL53L1X_Sensor::setAddress(uint8_t address) {
    _sensor.setAddress(address);
    _address = address;
}

uint8_t VL53L1X_Sensor::getAddress() const {
    return _address;
}

bool VL53L1X_Sensor::isConnected() {
    _wire->beginTransmission(_address);
    return (_wire->endTransmission() == 0);
}

void VL53L1X_Sensor::startContinuous(uint32_t period_ms) {
    _sensor.startContinuous(period_ms);
}

void VL53L1X_Sensor::stopContinuous() {
    _sensor.stopContinuous();
}

uint16_t VL53L1X_Sensor::readSingle() {
    _lastDistance = _sensor.readSingle() - 15; // -15 offset
    return _lastDistance;
}

uint16_t VL53L1X_Sensor::readContinuous() {
    _lastDistance = _sensor.read()- 15 ; // -15 offset
    return _lastDistance;
}

float VL53L1X_Sensor::getDistanceCM() {
    return _lastDistance / 10.0f;
}

float VL53L1X_Sensor::getDistanceM() {
    return _lastDistance / 1000.0f;
}

bool VL53L1X_Sensor::isWithinRange(uint16_t min_mm, uint16_t max_mm) {
    return (_lastDistance >= min_mm && _lastDistance <= max_mm);
}

bool VL53L1X_Sensor::startMeasure() {
    if (!_initialized) {
        return false;
    }

    // Resetujeme stav
    _dataReady    = false;
    _measuring    = true;
    _sampleCount  = 0;
    _sampleSum    = 0;
    _samplePending = false;

    // Spustíme kontinuální měření senzoru (pokud ještě neběží).
    // Perioda 50 ms odpovídá timing budget 50 000 µs nastaveném v begin().
    _sensor.startContinuous(50);

    return true;
}

bool VL53L1X_Sensor::dataReady() {
    return _dataReady;
}

float VL53L1X_Sensor::getDistanceMM() {
    return _averagedDistance;
}

void VL53L1X_Sensor::update() {
    if (!_initialized || !_measuring || _dataReady) return;

    // Má senzor připravený výsledek? Pokud ne, vrátíme se a zkusíme příště
    if (!_sensor.dataReady()) return;

    // Přečteme bez blokování
    uint16_t sample = _sensor.read(false);

    if (sample > 0 && sample < 65535) {
        _sampleSum += sample;
        _sampleCount++;
    }

    if (_sampleCount >= AVERAGE_SAMPLES) {
        _averagedDistance = static_cast<float>(_sampleSum) / _sampleCount - 15; // -15 offset
        _dataReady = true;
        _measuring = false;
        _sensor.stopContinuous();
    }
}