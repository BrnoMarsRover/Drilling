#ifndef ADS122C04_LIB_H
#define ADS122C04_LIB_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Commands
#define CMD_RESET     0x06
#define CMD_STARTSYNC 0x08
#define CMD_POWERDOWN 0x02
#define CMD_RDATA     0x10
#define CMD_RREG(reg) (0x20 | ((reg) << 2))
#define CMD_WREG(reg) (0x40 | ((reg) << 2))

// Register addresses
#define REG_MUX_GAIN     0x00
#define REG_DR_MODE      0x01
#define REG_DATA_STATUS  0x02
#define REG_IDAC_MUX     0x03

// MUX settings (bits 7:4)
#define MUX_AIN0_AIN1  0x00
#define MUX_AIN2_AIN3  0x60
#define MUX_SHORT      0xE0

// Gain settings (bits 3:1, raw field values)
#define GAIN_1   0x00
#define GAIN_2   0x01
#define GAIN_4   0x02
#define GAIN_8   0x03
#define GAIN_16  0x04
#define GAIN_32  0x05
#define GAIN_64  0x06
#define GAIN_128 0x07 // used

// Data rate (bits 7:5 of REG1)
#define DR_20SPS  0x00 // used
#define DR_45SPS  0x20
#define DR_90SPS  0x40
#define DR_175SPS 0x60
#define DR_330SPS 0x80
#define DR_600SPS 0xA0
#define DR_1000SPS 0xC0

// IDAC current (bits 2:0 of REG2)
#define IDAC_OFF    0x00
#define IDAC_10UA   0x01
#define IDAC_50UA   0x02
#define IDAC_100UA  0x03
#define IDAC_250UA  0x04
#define IDAC_500UA  0x05
#define IDAC_1000UA 0x06
#define IDAC_1500UA 0x07


class ADS122C04 {
public:
    // Constructor: set default I2C address and calibration values
    // dgnd-dgnd 100 0000 = 0x40
    // dgnd-dvdd 100 0001 = 0x41
    // dvdd-dgnd 100 0100 = 0x44 deep samples
    // dvdd-dvdd 100 0101 = 0x45 surface samples
    ADS122C04(TwoWire &wire, uint8_t addr = 0x44)
        : _wire(&wire), _addr(addr), cal_a(0.00467235f), cal_b(-6054.52392578f), tare_grams(0.0f) // old: cal_a(1.0f), cal_b(0.0f), tare_grams(0.0f) _resetPin(resetPin)
    {
      /*
      delay(1);
      reset(); // should not pull down RST pin
      // REG0: MUX=0000 (AIN0+/AIN1-), GAIN=111 (x16), PGA_BYPASS=0  → 0x0E // old 1000 -> 0x08
      // REG1: DR=000 (20SPS), MODE=0, CM=1 (continuous), VREF=00 (ext), TS=0 → 0x08
      // REG2: IDAC=101 (500uA), rest 0 → 0x05
      // REG3: I1MUX=011 (AIN2), I2MUX=100 (AIN3) → 0x70
      uint8_t cfg[4] = { 0x0E, 0x08, 0x07, 0x70 };
      for (int i = 0; i < 4; i++) _write_reg(i, cfg[i]);
      start();

      if (_verify_regs(cfg, 4)) {
        Serial.print("[ADC] 0x");
        Serial.print(addr, HEX);
        Serial.println(" OK");
      } else {
        Serial.print("[ADC] 0x");
        Serial.print(addr, HEX);
        Serial.println(" INIT FAIL");
      }
      */
    }
    ~ADS122C04();

    // Low-level control
    //void    init(void);
    void    begin(void);
    void    reset(void);
    void    start(void);
    void    update(void);
    void    powerdown(void);
    void    set_gain(uint8_t gain);
    void    set_mux(uint8_t mux);
    void    set_data_rate(uint8_t dr);
    void    set_idac(uint8_t i1mux, uint8_t i2mux, uint8_t current);
    bool    data_ready(void);
    int32_t read(void);
    void    set_address(uint8_t addr);

    // Higher-level scale functions
    float   read_median(uint8_t n);
    void    tare(void);
    void    scale_calibrate_0(void);
    void    scale_calibrate_100(void);
    float   read_temperature(void);

    // FreeRTOS task interface
    void    task_start(void);
    void    request_measure(void);
    void    request_tmp(void);
    void    set_tare(void);
    void    set_calibration_0(void);
    void    set_calibration_100(void);
    void    task_start_acqu(void); // to be written
    float   measure_for_time(uint32_t sec, uint32_t sample_num); // to be an array? each X ms will interript set adc_cmd:MEASURE and store value to an array, ms between samples will be calculated from sample num and sec


    // Rresult getters
    bool    get_result_ready(void);
    std::pair<float,float>   get_last_weight(void);
    float   get_last_temp(void);
    float   get_arr_weight(void); // to be written

private:
    TwoWire *_wire;
    uint8_t  _addr;
    float _cal_adc_zero = 0.0f; // stored between CALIBRATE_0 and CALIBRATE_100
    volatile float   cal_a;
    volatile float   cal_b;
    volatile float   tare_grams;

    // FreeRTOS objects
    QueueHandle_t _cmdQueue           = nullptr;
    SemaphoreHandle_t _mutex          = nullptr;
    TaskHandle_t _adc_task_handle     = nullptr;

    // Shared result state _mutex protected
    float   _lastWeight     = 0.0f;
    float   _lastWeightRaw  = 0.0f;
    float   _lastTemp       = 0.0f;
    bool    _result_ready   = false;

    enum class adc_cmd : uint8_t {
    MEASURE,
    TARE,
    CALIBRATE0,
    CALIBRATE100,
    TEMPERATURE
    };

    static void _adcTask(void *pvParameters);

    uint8_t _read_reg(uint8_t reg);
    void    _write_reg(uint8_t reg, uint8_t val);
    void    _isort(int32_t *arr, uint8_t n);
    bool    _verify_regs(const uint8_t *expected, uint8_t n);
};

#endif // ADS122C04_LIB_H
// ----- High end functions -----