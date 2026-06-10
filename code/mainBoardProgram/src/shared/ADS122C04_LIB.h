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
#include <cstdint>

struct WeightResult
{
  float grams;
  uint32_t raw;
};

class ADS122C04 {
public:
    /*
    struct WeightResult
    {
      float grams;
      uint32_t raw;
    };
    */

    enum class Cmd : uint8_t {
        Reset     = 0x06,
        StartSync = 0x08,
        PowerDown = 0x02,
        RData     = 0x10,
    };

    enum class Reg : uint8_t {
        MuxGain    = 0x00,
        DrMode     = 0x01,
        DataStatus = 0x02,
        IdacMux    = 0x03,
    };

    // MUX input selection (bits 7:4 of REG_MUX_GAIN)
    enum class Mux : uint8_t {
        AIN0_AIN1 = 0x00,
        AIN2_AIN3 = 0x60,
        Short      = 0xE0,
    };

    // PGA gain (bits 3:1 of REG_MUX_GAIN)
    enum class Gain : uint8_t {
        x1   = 0x00,
        x2   = 0x01,
        x4   = 0x02,
        x8   = 0x03,
        x16  = 0x04,
        x32  = 0x05,
        x64  = 0x06,
        x128 = 0x07,
    };

    // Data rate (bits 7:5 of REG_DR_MODE)
    enum class DataRate : uint8_t {
        SPS_20   = 0x00, // used
        SPS_45   = 0x20,
        SPS_90   = 0x40,
        SPS_175  = 0x60,
        SPS_330  = 0x80,
        SPS_600  = 0xA0,
        SPS_1000 = 0xC0,
    };

    // IDAC excitation current (bits 2:0 of REG_IDAC_MUX)
    enum class Idac : uint8_t {
        Off    = 0x00,
        uA_10  = 0x01,
        uA_50  = 0x02,
        uA_100 = 0x03,
        uA_250 = 0x04,
        uA_500 = 0x05,
        mA_1   = 0x06,
        mA_1_5 = 0x07, // used
    };

    ADS122C04(TwoWire &wire, uint8_t addr)
        : _wire(&wire), _addr(addr), _cal_adc_zero(0.0f), _cal_weight(100.0f), cal_a(0.00467235f), cal_b(-654.52392578f), tare_grams(0.0f) // old: cal_a(1.0f), cal_b(0.0f), tare_grams(0.0f) _resetPin(resetPin)
    {
    }
    ~ADS122C04();

    // Low-level control
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

    // Helpers
    template<typename E>
    constexpr uint8_t to_U8(E num) { return static_cast<uint8_t>(num); }
    constexpr uint8_t cmdRReg(uint8_t reg) { return 0x20 | (reg << 2); }
    constexpr uint8_t cmdWReg(uint8_t reg) { return 0x40 | (reg << 2); }

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
    void    set_calibration_100(float);
    void    task_start_acqu(void); // to be written
    float   measure_for_time(uint32_t sec, uint32_t sample_num); // to be an array? each X ms will interript set adc_cmd:MEASURE and store value to an array, ms between samples will be calculated from sample num and sec


    // Rresult getters
    bool    get_result_ready(void);
    WeightResult get_last_weight(void);
    float   get_last_temp(void);
    float   get_arr_weight(void); // to be written
    bool    get_adc_connected();

private:
    TwoWire *_wire;
    uint8_t  _addr;
    volatile float   _cal_adc_zero; // OLD = 0.0f stored between CALIBRATE_0 and CALIBRATE_100
    volatile float   _cal_weight; // OLD = 100.0f
    volatile float   cal_a;
    volatile float   cal_b;
    volatile float   tare_grams;

    // FreeRTOS objects
    QueueHandle_t _cmdQueue           = nullptr;
    SemaphoreHandle_t _mutex          = nullptr;
    TaskHandle_t _adc_task_handle     = nullptr;

    // Shared result state _mutex protected
    float   _lastWeight     = 0.0f;
    uint32_t _lastWeightRaw = 0;
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