#include "INA219_Sensor.h"

// ---------------------------------------------------------------------------
// Konstruktor
// ---------------------------------------------------------------------------
INA219_Sensor::INA219_Sensor(TwoWire &wire, uint8_t address)
    : _wire(&wire),
      _address(address),
      _initialized(false),
      _bvr(RANGE_32V),
      _pga(PGA_8),
      _badc(ADC_12BIT),
      _sadc(ADC_12BIT),
      _rShunt(0.1f),
      _currentLSB(0.0f),
      _powerLSB(0.0f),
      _lastCurrentA(0.0f),
      _lastBusVoltageV(0.0f),
      _lastPowerW(0.0f),
      _measuring(false),
      _dataReady(false)
{
}

// ---------------------------------------------------------------------------
// Inicializace
// ---------------------------------------------------------------------------
bool INA219_Sensor::begin(float rShuntOhm, float maxCurrentA,
                           BusVoltageRange bvr, PGAGain pga,
                           ADCResolution badc, ADCResolution sadc)
{
    if (!isConnected()) {
        return false;
    }

    _bvr  = bvr;
    _pga  = pga;
    _badc = badc;
    _sadc = sadc;

    reset();
    delay(1); // počkat na dokončení resetu

    applyConfiguration();
    setCalibration(rShuntOhm, maxCurrentA);

    _initialized = true;
    return true;
}

void INA219_Sensor::reset() {
    // Bit 15 = RST – softwarový reset, self-clearing
    writeRegister(REG_CONFIG, 0x8000);
}

// ---------------------------------------------------------------------------
// Konfigurace
// ---------------------------------------------------------------------------
void INA219_Sensor::applyConfiguration() {
    // Bit 13: BRNG (bus voltage range)
    // Bity 11-12: PG1, PG0 (PGA gain)
    // Bity 7-10: BADC
    // Bity 3-6:  SADC
    // Bity 0-2:  MODE = 0b111 (shunt + bus, continuous)
    uint16_t config = 0x0000;

    config |= (uint16_t)(_bvr  & 0x01) << 13;
    config |= (uint16_t)(_pga  & 0x03) << 11;
    config |= (uint16_t)(_badc & 0x0F) << 7;
    config |= (uint16_t)(_sadc & 0x0F) << 3;
    config |= 0x07; // MODE: shunt + bus, continuous

    writeRegister(REG_CONFIG, config);
}

void INA219_Sensor::setCalibration(float rShuntOhm, float maxCurrentA) {
    _rShunt = rShuntOhm;

    // Current_LSB = MaxCurrent / 2^15
    _currentLSB = maxCurrentA / 32768.0f;

    // Power_LSB = 20 * Current_LSB
    _powerLSB = 20.0f * _currentLSB;

    // Cal = trunc(0.04096 / (Current_LSB * R_shunt))
    uint16_t cal = (uint16_t)(0.04096f / (_currentLSB * _rShunt));

    writeRegister(REG_CALIBRATION, cal);
}

void INA219_Sensor::setBusVoltageRange(BusVoltageRange bvr) {
    _bvr = bvr;
    applyConfiguration();
}

void INA219_Sensor::setPGAGain(PGAGain pga) {
    _pga = pga;
    applyConfiguration();
}

void INA219_Sensor::setBusADC(ADCResolution res) {
    _badc = res;
    applyConfiguration();
}

void INA219_Sensor::setShuntADC(ADCResolution res) {
    _sadc = res;
    applyConfiguration();
}

// ---------------------------------------------------------------------------
// Jednorázová (blokující) čtení
// ---------------------------------------------------------------------------
float INA219_Sensor::readCurrentA() {
    int16_t raw = (int16_t)readRegister(REG_CURRENT);
    _lastCurrentA = raw * _currentLSB;
    return _lastCurrentA;
}

float INA219_Sensor::readCurrentMA() {
    return readCurrentA() * 1000.0f;
}

float INA219_Sensor::readShuntVoltageMV() {
    int16_t raw = (int16_t)readRegister(REG_SHUNTVOLT);
    // LSB shunt napětí = 10 µV → převod na mV
    return raw * 0.01f;
}

float INA219_Sensor::readBusVoltageV() {
    uint16_t raw = readRegister(REG_BUSVOLT);
    // Bity 15:3 = data, bity 2:0 = příznaky – posun o 3
    // LSB = 4 mV
    _lastBusVoltageV = (float)(raw >> 3) * 0.004f;
    return _lastBusVoltageV;
}

float INA219_Sensor::readPowerW() {
    uint16_t raw = readRegister(REG_POWER);
    _lastPowerW = raw * _powerLSB;
    return _lastPowerW;
}

float INA219_Sensor::readPowerMW() {
    return readPowerW() * 1000.0f;
}

// ---------------------------------------------------------------------------
// Stav a info
// ---------------------------------------------------------------------------
bool INA219_Sensor::isConnected() {
    _wire->beginTransmission(_address);
    return (_wire->endTransmission() == 0);
}

uint8_t INA219_Sensor::getAddress() const {
    return _address;
}

bool INA219_Sensor::isMathOverflow() {
    uint16_t busReg = readRegister(REG_BUSVOLT);
    return (busReg & 0x01) != 0; // bit 0 = OVF
}

bool INA219_Sensor::isConversionReady() {
    uint16_t busReg = readRegister(REG_BUSVOLT);
    return (busReg & 0x02) != 0; // bit 1 = CNVR
}

float INA219_Sensor::getLastCurrentA()    const { return _lastCurrentA; }
float INA219_Sensor::getLastCurrentMA()   const { return _lastCurrentA * 1000.0f; }
float INA219_Sensor::getLastBusVoltageV() const { return _lastBusVoltageV; }
float INA219_Sensor::getLastPowerW()      const { return _lastPowerW; }
float INA219_Sensor::getCurrentLSB()      const { return _currentLSB; }
float INA219_Sensor::getPowerLSB()        const { return _powerLSB; }
float INA219_Sensor::getRShunt()          const { return _rShunt; }

// ---------------------------------------------------------------------------
// Neblokující průměrované měření
// ---------------------------------------------------------------------------
bool INA219_Sensor::startMeasure() {
    if (!_initialized) return false;
    _lastUpdateMs      = 0;       // vynutí první měření okamžitě
    _measuring         = true;
    _filterInitialized = false;
    return true;
}

bool INA219_Sensor::dataReady() {
    return _filterInitialized;
}

float INA219_Sensor::getAveragedCurrentA() {
    return _filteredCurrentA;
}

float INA219_Sensor::getAveragedCurrentMA() {
    return _filteredCurrentA * 1000.0f;
}

void INA219_Sensor::update()
{
    if (!_initialized || !_measuring) return;
   
    uint32_t now = millis();
    if (now - _lastUpdateMs < _intervalMs) return;   // ještě není čas

    if (!isConversionReady()) return;

    _lastUpdateMs = now;

    int16_t raw = (int16_t)readRegister(REG_CURRENT);
    float sample = raw * _currentLSB;

    if (!_filterInitialized) {
        _filteredCurrentA = sample;
        _filterInitialized = true;
    } else {
        _filteredCurrentA += _alpha * (sample - _filteredCurrentA);
    }

    _lastCurrentA = _filteredCurrentA;
}

// ---------------------------------------------------------------------------
// Nízkoúrovňový přístup k registrům
// ---------------------------------------------------------------------------
void INA219_Sensor::writeRegister(uint8_t reg, uint16_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write((uint8_t)(value >> 8));   // MSB první
    _wire->write((uint8_t)(value & 0xFF)); // LSB druhý
    _wire->endTransmission();
}

uint16_t INA219_Sensor::readRegister(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false); // repeated START

    _wire->requestFrom((uint8_t)_address, (uint8_t)2);
    if (_wire->available() < 2) return 0;

    uint16_t msb = _wire->read();
    uint16_t lsb = _wire->read();
    return (msb << 8) | lsb;
}
