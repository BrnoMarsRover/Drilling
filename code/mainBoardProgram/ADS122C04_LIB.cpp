#include "ads122c04_lib.h"
#include <stdlib.h>
#include <math.h>

//const int n_reset = 2;

// ─── Private helpers ──────────────────────────────────────────────────────────
uint8_t ADS122C04::_read_reg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(CMD_RREG(reg));
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);
    return _wire->read();
}

void ADS122C04::_write_reg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(CMD_WREG(reg));
    _wire->write(val);
    _wire->endTransmission();
}

void ADS122C04::_isort(int32_t *arr, uint8_t n) {
    for (uint8_t i = 1; i < n; i++) {
        int32_t key = arr[i];
        int8_t j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// ─── Commands ─────────────────────────────────────────────────────────────────
void ADS122C04::reset(void) {
    _wire->beginTransmission(_addr);
    _wire->write(CMD_RESET);
    _wire->endTransmission();
    delay(1);
}

void ADS122C04::start(void) {
    _wire->beginTransmission(_addr);
    _wire->write(CMD_STARTSYNC);
    _wire->endTransmission();
}

void ADS122C04::powerdown(void) {
    _wire->beginTransmission(_addr);
    _wire->write(CMD_POWERDOWN);
    _wire->endTransmission();
}

// ─── Init ─────────────────────────────────────────────────────────────────────

void ADS122C04::init(void) {
    pinMode(_resetPin, OUTPUT);
    digitalWrite(_resetPin, HIGH);
    delay(1);
    reset();

    // REG0: MUX=0000 (AIN0+/AIN1-), GAIN=111 (x16), PGA_BYPASS=0  → 0x0E // old 1000 -> 0x08
    // REG1: DR=000 (20SPS), MODE=0, CM=1 (continuous), VREF=00 (ext), TS=0 → 0x08
    // REG2: IDAC=101 (500uA), rest 0 → 0x05
    // REG3: I1MUX=011 (AIN2), I2MUX=100 (AIN3) → 0x70
    uint8_t cfg[4] = { 0x0E, 0x08, 0x07, 0x70 };
    for (int i = 0; i < 4; i++) {
        _write_reg(i, cfg[i]);
    }
    start();
}

//void ADS122C04::init(void) { begin(); }  // backward compat

// ─── Gain ─────────────────────────────────────────────────────────────────────
void ADS122C04::set_gain(uint8_t gain) {
    uint8_t reg0 = _read_reg(REG_MUX_GAIN);
    reg0 = (reg0 & 0xF1) | ((gain & 0x07) << 1);
    if (gain > GAIN_4) reg0 &= ~0x01;
    _write_reg(REG_MUX_GAIN, reg0);
    start();
}

// ─── MUX ──────────────────────────────────────────────────────────────────────
void ADS122C04::set_mux(uint8_t mux) {
    uint8_t reg0 = _read_reg(REG_MUX_GAIN);
    reg0 = (reg0 & 0x0F) | (mux & 0xF0);
    _write_reg(REG_MUX_GAIN, reg0);
    start();
}

// ─── Data rate ────────────────────────────────────────────────────────────────
void ADS122C04::set_data_rate(uint8_t dr) {
    uint8_t reg1 = _read_reg(REG_DR_MODE);
    reg1 = (reg1 & 0x1F) | (dr & 0xE0);
    _write_reg(REG_DR_MODE, reg1);
    start();
}

// ─── IDAC ─────────────────────────────────────────────────────────────────────
void ADS122C04::set_idac(uint8_t i1mux, uint8_t i2mux, uint8_t current) {
    uint8_t reg2 = _read_reg(REG_DATA_STATUS);
    reg2 = (reg2 & 0xF8) | (current & 0x07);
    _write_reg(REG_DATA_STATUS, reg2);

    uint8_t reg3 = ((i1mux & 0x07) << 5) | ((i2mux & 0x07) << 2);
    _write_reg(REG_IDAC_MUX, reg3);
    start();
}

// ─── Data ready ───────────────────────────────────────────────────────────────
bool ADS122C04::data_ready(void) {
    return (_read_reg(REG_DATA_STATUS) & 0x80) != 0;
}

// ─── Read raw 24-bit result ───────────────────────────────────────────────────
int32_t ADS122C04::read(void) {
    _wire->beginTransmission(_addr);
    _wire->write(CMD_RDATA);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)3);

    uint8_t b2 = _wire->read();
    uint8_t b1 = _wire->read();
    uint8_t b0 = _wire->read();

    int32_t val = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | (int32_t)b0;
    if (val & 0x800000) val |= 0xFF000000;
    return val;
}

// ─── Single-shot blocking measurement ────────────────────────────────────────
int32_t ADS122C04::measure(void) {
    uint8_t reg1 = _read_reg(REG_DR_MODE);
    _write_reg(REG_DR_MODE, reg1 & ~0x08);
    start();

    uint16_t timeout = 60;
    while (!data_ready() && timeout--) delay(1);

    int32_t result = read();
    _write_reg(REG_DR_MODE, reg1);
    if (reg1 & 0x08) start();
    return result;
}

// ─── Address ──────────────────────────────────────────────────────────────────
void ADS122C04::set_address(uint8_t addr) {
    _addr = addr;
}

// ─── Median ───────────────────────────────────────────────────────────────────
float ADS122C04::read_median(uint8_t n) {
    if (n == 0) n = 1;

    int32_t *buf = (int32_t *)malloc(n * sizeof(int32_t));
    if (!buf) return (float)read();

    for (uint8_t i = 0; i < n; i++) {
        uint16_t timeout = 60;
        while (!data_ready() && timeout--) delay(1);
        buf[i] = read();
    }

    _isort(buf, n);

    float result;
    if (n % 2 == 1) {
        result = (float)buf[n / 2];
    } else {
        result = ((float)buf[n / 2 - 1] + (float)buf[n / 2]) / 2.0f;
    }

    free(buf);
    return result;
}

// ─── Tare ─────────────────────────────────────────────────────────────────────
void ADS122C04::tare(void) {
    float raw = read_median(32);
    tare_grams = cal_a * raw + cal_b;             // store in grams
    Serial.print(F("[TARE] Tare value stored: "));
    Serial.print(tare_grams, 4);
    Serial.println(F(" g"));
}

// ─── Scale calibration ────────────────────────────────────────────────────────
void ADS122C04::scale_calibrate(void) {
    Serial.println(F("=== Scale calibration ==="));
    Serial.println(F("Remove all weight from scale, then press ENTER."));

    while (Serial.available()) Serial.read();
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();

    float adc_zero = read_median(32);
    Serial.print(F("[CAL] ADC at 0 g: "));
    Serial.println(adc_zero, 2);

    Serial.println(F("Place exactly 100 g on scale, then press ENTER."));
    while (Serial.available()) Serial.read();
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();

    float adc_100 = read_median(32);
    Serial.print(F("[CAL] ADC at 100 g: "));
    Serial.println(adc_100, 2);

    if (fabsf(adc_100 - adc_zero) < 1.0f) {
        Serial.println(F("[CAL] ERROR: ADC span too small - check wiring. Calibration aborted."));
        return;
    }

    cal_a = 100.0f / (adc_100 - adc_zero);
    cal_b = 0.0f - cal_a * adc_zero;

    Serial.println(F("[CAL] Calibration complete."));
    Serial.print(F("  a (slope)  = ")); Serial.println(cal_a, 8);
    Serial.print(F("  b (offset) = ")); Serial.println(cal_b, 8);
    Serial.println(F("  y = a*x + b  (y in grams, x = raw ADC)"));
}

// ─── Weight measurement ───────────────────────────────────────────────────────
float ADS122C04::measure_weight(void) {
    float raw    = read_median(10); // will be 6 ideally, measure mode 20 sps, i call measure each 500 ms
    float grams  = cal_a * raw + cal_b;           // apply calibration first
    float tared  = grams - (float)tare_grams;     // subtract tare in grams
    return tared;
}

float ADS122C04::read_temperature(void) {
    // Save current REG1 to restore after reading
    uint8_t reg1 = _read_reg(REG_DR_MODE);

    // Set TS=1 (bit 0), keep other settings
    _write_reg(REG_DR_MODE, reg1 | 0x01);
    start();

    // Wait for conversion
    uint16_t timeout = 60;
    while (!data_ready() && timeout--) delay(1);

    // Read raw 24-bit result
    _wire->beginTransmission(_addr);
    _wire->write(CMD_RDATA);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)3);
    uint8_t b2 = _wire->read();
    uint8_t b1 = _wire->read();
    _wire->read();                        // b0 not used, only 14 MSBs matter

    // Restore previous REG1 (clears TS bit)
    _write_reg(REG_DR_MODE, reg1);
    start();

    // 14-bit result is left-justified in 24-bit word
    // b2 and b1 contain bits 23:8, we need bits 23:10
    int16_t raw = ((int16_t)b2 << 6) | (b1 >> 2);

    // raw is 14-bit two's complement
    // sign extend from bit 13
    if (raw & 0x2000) raw |= 0xC000;

    // 1 LSB = 0.03125°C
    return (float)raw * 0.03125f;
}

// -------- High end fncs ---------
// acquisition in time