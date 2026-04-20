#include "VL53L1X_Sensor.h"

VL53L1X_Sensor::VL53L1X_Sensor(TwoWire &wire, uint8_t address)
    : _wire(&wire), _address(address), _lastDistance(0), _initialized(false) {
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

float VL53L1X_Sensor::readSingle() {
    for(uint8_t i = 0; i < 10; i++){
        _lastDistance = _sensor.readSingle();
        _averageDistance += _lastDistance;
    }
    _averageDistance = _averageDistance/10;

    return _lastDistance;
}

uint16_t VL53L1X_Sensor::readContinuous() {
    _lastDistance = _sensor.read();
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