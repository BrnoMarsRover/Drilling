#include "ads122c04_lib.h"
#include <stdlib.h>
#include <math.h>
#include <utility>
#include <Preferences.h>

const int n_reset = 2;

Preferences prefs;

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

bool ADS122C04::_verify_regs(const uint8_t *expected, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    uint8_t actual = _read_reg(i);
    if (actual != expected[i]) {
      Serial.print("[ADC] REG");
      Serial.print(i);
      Serial.println(" MISMATCH");
      /*Serial.print(" MISMATCH — expected 0x");
      Serial.print(expected[i], HEX);
      Serial.print(" got 0x");
      Serial.println(actual, HEX);
      */
      return false;
    }
  }
  return true;
}

void ADS122C04::reset(void) {
  _wire->beginTransmission(_addr);
  _wire->write(CMD_RESET);
  _wire->endTransmission();
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

void ADS122C04::begin(void) {
  //pinMode(2,OUTPUT);    // !! ---- FOR BOARD V1 ONLY ---- !!
  //digitalWrite(2, HIGH);    // !! ---- FOR BOARD V1 ONLY ---- !!

  //delay(1);
  reset(); // should not pull down RST pin
  // REG0: MUX=0000 (AIN0+/AIN1-), GAIN=111 (x16), PGA_BYPASS=0  → 0x0E // old 1000 -> 0x08
  // REG1: DR=000 (20SPS), MODE=0, CM=1 (continuous), VREF=00 (ext), TS=0 → 0x08
  // REG2: IDAC=111 (1500uA), rest 0 → 0x07
  // REG3: I1MUX=011 (AIN2), I2MUX=100 (AIN3) → 0x70
  uint8_t cfg[4] = { 0x0E, 0x08, 0x07, 0x70 };
  for (int i = 0; i < 4; i++) _write_reg(i, cfg[i]);
  start();

  if (_verify_regs(cfg, 4)) {
    Serial.print("[ADC] 0x");
    Serial.print(_addr, HEX);
    Serial.println(" OK");
  } else {
    Serial.print("[ADC] 0x");
    Serial.print(_addr, HEX);
    Serial.println(" INIT FAIL");
  }

  if(_addr == 0x44){
    prefs.begin("calibration44", true);
    cal_a = prefs.getFloat("a_44", 0.00467235f);
    cal_b = prefs.getFloat("b_44", -6054.52392578f);
    prefs.end();
  }else if(_addr == 0x45){
    prefs.begin("calibration45", true);
    cal_a = prefs.getFloat("a_45", 0.00467235f);
    cal_b = prefs.getFloat("b_45", -6054.52392578f);
    prefs.end();
  }
}

ADS122C04::~ADS122C04() {
  powerdown();
  _wire = nullptr;
}

void ADS122C04::set_gain(uint8_t gain) { // Gain set
  uint8_t reg0 = _read_reg(REG_MUX_GAIN);
  reg0 = (reg0 & 0xF1) | ((gain & 0x07) << 1);
  if (gain > GAIN_4) reg0 &= ~0x01;
  _write_reg(REG_MUX_GAIN, reg0);
  start();
}

void ADS122C04::set_mux(uint8_t mux) { // MUX set
  uint8_t reg0 = _read_reg(REG_MUX_GAIN);
  reg0 = (reg0 & 0x0F) | (mux & 0xF0);
  _write_reg(REG_MUX_GAIN, reg0);
  start();
}

void ADS122C04::set_data_rate(uint8_t dr) { // Data rate set
  uint8_t reg1 = _read_reg(REG_DR_MODE);
  reg1 = (reg1 & 0x1F) | (dr & 0xE0);
  _write_reg(REG_DR_MODE, reg1);
  start();
}

void ADS122C04::set_idac(uint8_t i1mux, uint8_t i2mux, uint8_t current) { // IDAC set
  uint8_t reg2 = _read_reg(REG_DATA_STATUS);
  reg2 = (reg2 & 0xF8) | (current & 0x07);
  _write_reg(REG_DATA_STATUS, reg2);

  uint8_t reg3 = ((i1mux & 0x07) << 5) | ((i2mux & 0x07) << 2);
  _write_reg(REG_IDAC_MUX, reg3);
  start();
}

bool ADS122C04::data_ready(void) { // Data ready
  return (_read_reg(REG_DATA_STATUS) & 0x80) != 0;
}

int32_t ADS122C04::read(void) { // Read raw 24-bit result
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
/*
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
*/

void ADS122C04::set_address(uint8_t addr) {
  _addr = addr;
}

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

void ADS122C04::tare(void) {
  float raw = read_median(32);
  tare_grams = cal_a * raw + cal_b; // store in grams
  Serial.print(F("[ADC] Tare value stored: "));
  Serial.print(tare_grams, 4);
  Serial.println(F(" g"));
}

void ADS122C04::scale_calibrate_0(void) {
  Serial.println(F("[ADC] Scale calibration 1/2. "));
  Serial.println(F("On scale should be weight of 0g"));

  float adc_zero = read_median(16);
  _cal_adc_zero = adc_zero;
  Serial.print(F("[ADC] at 0g: "));
  Serial.println(adc_zero, 2);

  Serial.println(F("[ADC] Place weight on scale"));
}

void ADS122C04::scale_calibrate_100(void) {
  Serial.println(F("[ADC] Scale calibration 2/2. "));
  Serial.print(F("On scale should be weight of Xg"));

  float adc_100 = read_median(16);
  Serial.print(F("[ADC] at Xg: "));
  Serial.println(adc_100, 2);

  if (fabsf(adc_100 - _cal_adc_zero) < 1.0f) {
    Serial.println(F("[ADC] ERROR: ADC span too small - check wiring. Calibration aborted."));
    return;
  }

  //Serial.print(F("[ADC] Cal. weight:"));
  //Serial.println(_cal_weight, 2);
  cal_a = _cal_weight / (adc_100 - _cal_adc_zero); // OLD 100.0f / (adc_100 - _cal_adc_zero) // _cal_weight
  cal_b = 0.0f - cal_a * _cal_adc_zero;

  if(_addr == 0x44){
    prefs.begin("calibration44", false);
    prefs.putFloat("a_44", cal_a);
    prefs.putFloat("b_44", cal_b);
    prefs.end();
  }else if(_addr == 0x45){
    prefs.begin("calibration45", false);
    prefs.putFloat("a_45", cal_a);
    prefs.putFloat("b_45", cal_b);
    prefs.end();
  }

  Serial.print("[ADC] 0x");
  Serial.print(_addr, HEX);
  Serial.println(" Calibration complete");
  Serial.print(F("a = ")); Serial.println(cal_a, 8);
  Serial.print(F("b = ")); Serial.println(cal_b, 8);
  Serial.println(F("  y = a*x + b  (y in grams, x = raw ADC)"));
}

float ADS122C04::read_temperature(void) { // to update to non-blocking fnc
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
  _wire->read();

  // Restore previous REG1 (clears TS bit)
  _write_reg(REG_DR_MODE, reg1);
  start();

  int16_t raw = ((int16_t)b2 << 6) | (b1 >> 2);

  // raw is 14-bit two's complement, sign extend from bit 13
  if (raw & 0x2000) raw |= 0xC000;

  // 1 LSB = 0.03125°C
  return (float)raw * 0.03125f;
}

void ADS122C04::task_start() {
    _cmdQueue = xQueueCreate(5, sizeof(adc_cmd));  // queue of 5 commands
    _mutex    = xSemaphoreCreateMutex();

    char taskName[16];
    snprintf(taskName, sizeof(taskName), "ADC_0x%02X", _addr); // dynamic "ADC_0x44", "ADC_0x45"

    xTaskCreatePinnedToCore(
        _adcTask,               // fnc to run
        taskName,               // old "ADC_Task" system name
        2048,                   // stack size in bytes 4096
        this,                   // pvParameters
        1,                      // priority low
        &_adc_task_handle,      // handle stored here not used now
        0                       // loop() runs on core 1
    );
}

void ADS122C04::request_measure() {
  _result_ready = false;  // prevents repetitive readings and states that new value is to be written
  adc_cmd cmd = adc_cmd::MEASURE;
  xQueueSend(_cmdQueue, &cmd, 0);
}

void ADS122C04::request_tmp(){
  adc_cmd cmd = adc_cmd::TEMPERATURE;
  xQueueSend(_cmdQueue, &cmd, 0);
}

void ADS122C04::set_tare(){
  adc_cmd cmd = adc_cmd::TARE;
  xQueueSend(_cmdQueue, &cmd, 0);
}

void ADS122C04::set_calibration_0(){
  adc_cmd cmd = adc_cmd::CALIBRATE0;
  xQueueSend(_cmdQueue, &cmd, 0);
}

void ADS122C04::set_calibration_100(float weightX){
  _cal_weight = weightX;
  adc_cmd cmd = adc_cmd::CALIBRATE100;
  xQueueSend(_cmdQueue, &cmd, 0);
}

void ADS122C04::_adcTask(void *pvParameters) {
    ADS122C04 *self = static_cast<ADS122C04*>(pvParameters); // redefine void* to ADS122C04* to use self
    adc_cmd cmd;

    while (true) {
        if (xQueueReceive(self->_cmdQueue, &cmd, portMAX_DELAY)) { // will not go through till right command
            switch (cmd) {
                case adc_cmd::MEASURE: {
                    float raw = self->read_median(10);
                    float w   = self->cal_a * raw + self->cal_b - self->tare_grams;
                    xSemaphoreTake(self->_mutex, portMAX_DELAY);
                    self->_lastWeight  = w;
                    self->_lastWeightRaw = (uint32_t)raw;
                    self->_result_ready = true;
                    xSemaphoreGive(self->_mutex);
                    //UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL); // NULL refers to this task; to erase after testing
                    //Serial.print("[_adcTask] remaining words till overflow: "); // to erase after testing
                    //Serial.println(hwm); // to erase after testing; with 4096 words i wish for this to be max 1000 words left
                    break;
                }
                case adc_cmd::TARE:
                    self->tare();
                    break;

                case adc_cmd::CALIBRATE0:
                    self->scale_calibrate_0();
                    break;

                case adc_cmd::CALIBRATE100:
                    self->scale_calibrate_100();
                    break;

                case adc_cmd::TEMPERATURE: {
                    float t = self->read_temperature();
                    xSemaphoreTake(self->_mutex, portMAX_DELAY);
                    self->_lastTemp = t;
                    xSemaphoreGive(self->_mutex);
                    break;
                }
            }
        }
    }
}

bool ADS122C04::get_result_ready(void) {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    bool r = _result_ready;
    xSemaphoreGive(_mutex);
    return r;
}

WeightResult ADS122C04::get_last_weight(void) {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    WeightResult result = {_lastWeight, _lastWeightRaw};
    xSemaphoreGive(_mutex);
    return result;
}

float ADS122C04::get_last_temp(void) {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    float t = _lastTemp;
    xSemaphoreGive(_mutex);
    return t;
}

bool ADS122C04::get_adc_connected(void) {
    uint8_t reg = 0x0E;
    return _verify_regs(&reg, 1);
}

// -------- High end fncs ---------
// acquisition in time

/*
void ADS122C04::task_start_acqu() {
    _cmdQueue = xQueueCreate(5, sizeof(adc_cmd));  // queue of 5 commands -> i need just 1?
    //_mutex    = xSemaphoreCreateMutex();

    char taskName[16];
    snprintf(taskName, sizeof(taskName), "ADC_0x%02X", _addr); // dynamic "ADC_0x44", "ADC_0x45"

    xTaskCreatePinnedToCore(
        _adc_acq_Task,          // fnc to run
        taskName,               // old "ADC_Task" system name
        4096,                   // stack size in bytes
        this,                   // pvParameters
        1,                      // priority low
        NULL,                   // handle stored here not used now
        1                       // set core 1
    );
}
*/
