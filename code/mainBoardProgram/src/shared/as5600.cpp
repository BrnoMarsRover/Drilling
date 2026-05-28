#include "as5600.h"

AS5600L::AS5600L(TwoWire &wire, uint8_t address)
  : _wire(&wire), _address(address) {
}

bool AS5600L::begin() {
  delay(10);

  if (!isConnected()) {
    return false;
  }

  uint16_t angle;
  if (!read16(REG_RAW_ANGLE, angle)) {
    return false;
  }

  _rawAngle = angle & 0x0FFF;
  if (_direction) {
    _rawAngle = (COUNTS_PER_REV - 1) - _rawAngle;
  }
  _lastRawAngle = _rawAngle;
  _turnCounter = 0;
  _zeroOffsetCounts = _rawAngle;
  _hasSample = true;
  _initialized = true;

  return true;
}

void AS5600L::setAddress(uint8_t address) {
  _address = address;
}

uint8_t AS5600L::getAddress() const {
  return _address;
}

bool AS5600L::isConnected() {
  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}

bool AS5600L::read8(uint8_t reg, uint8_t &value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  if (_wire->requestFrom((int)_address, 1) != 1) {
    return false;
  }

  value = _wire->read();
  return true;
}

bool AS5600L::read16(uint8_t reg, uint16_t &value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  if (_wire->requestFrom((int)_address, 2) != 2) {
    return false;
  }

  uint8_t msb = _wire->read();
  uint8_t lsb = _wire->read();
  value = ((uint16_t)msb << 8) | lsb;
  value &= 0x0FFF;
  return true;
}

uint8_t AS5600L::readStatus() {
  uint8_t status = 0;
  read8(REG_STATUS, status);
  return status;
}

bool AS5600L::magnetDetected() {
  uint8_t s = readStatus();
  return (s & (1 << 5)) != 0;   // MD
}

bool AS5600L::magnetTooWeak() {
  uint8_t s = readStatus();
  return (s & (1 << 4)) != 0;   // ML
}

bool AS5600L::magnetTooStrong() {
  uint8_t s = readStatus();
  return (s & (1 << 3)) != 0;   // MH
}

uint16_t AS5600L::readRawAngle() {
  uint16_t value = 0;
  if (read16(REG_RAW_ANGLE, value)) {
    if (_direction) value = (COUNTS_PER_REV - 1) - value;
    _rawAngle = value;
  }
  return _rawAngle;
}

uint16_t AS5600L::readAngle() {
  uint16_t value = 0;
  read16(REG_ANGLE, value);
  return value;
}

bool AS5600L::update() {
  if (!_initialized) return false;

  uint16_t newRaw;
  if (!read16(REG_RAW_ANGLE, newRaw)) {
    return false;
  }

  newRaw &= 0x0FFF;

  // Obrácení směru: zrcadlíme hodnotu kolem středu rozsahu
  if (_direction) {
    newRaw = (COUNTS_PER_REV - 1) - newRaw;
  }

  if (!_hasSample) {
    _rawAngle = newRaw;
    _lastRawAngle = newRaw;
    _hasSample = true;
    return true;
  }

  int32_t delta = (int32_t)newRaw - (int32_t)_lastRawAngle;

  // Přetečení přes 0° / 360°
  if (delta > WRAP_THRESHOLD) {
    _turnCounter--;
  } else if (delta < -WRAP_THRESHOLD) {
    _turnCounter++;
  }

  _lastRawAngle = newRaw;
  _rawAngle = newRaw;

  return true;
}

void AS5600L::setZero() {
  _zeroOffsetCounts = absoluteCounts();
}

int32_t AS5600L::getZeroOffset() const {
  return _zeroOffsetCounts;
}

void AS5600L::setDirection(bool direction) {
  _direction = direction;
}

float AS5600L::getAngleDegrees() {
  return (_rawAngle * 360.0f) / (float)COUNTS_PER_REV;
}

int32_t AS5600L::absoluteCounts() const {
  return _turnCounter * COUNTS_PER_REV + (int32_t)_rawAngle;
}

int32_t AS5600L::relativeCounts() const {
  return absoluteCounts() - _zeroOffsetCounts;
}

int32_t AS5600L::getTurns() const {
  int32_t counts = relativeCounts();

  if (counts >= 0) {
    return counts / COUNTS_PER_REV;
  } else {
    return -(((-counts) / COUNTS_PER_REV));
  }
}

void AS5600L::setZeroOffset(int32_t offsetCounts) {
  _zeroOffsetCounts = offsetCounts;
}

float AS5600L::getRevolutions() const {
  return (float)relativeCounts() / (float)COUNTS_PER_REV;
}

float AS5600L::getTotalAngleDegrees() const {
  return getRevolutions() * 360.0f;
}

