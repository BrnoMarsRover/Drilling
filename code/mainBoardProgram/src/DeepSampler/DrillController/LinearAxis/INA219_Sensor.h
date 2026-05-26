#ifndef INA219_SENSOR_H
#define INA219_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// INA219_Sensor – neblokující obálka pro Texas Instruments INA219
//
// Podporuje:
//  - jednorázové i průběžné čtení (proud, napětí, výkon, shunt)
//  - neblokující průměrované měření proudu (metoda update())
//  - nastavení rozsahu sběrnicového napětí (16 / 32 V)
//  - nastavení PGA zisku (shunt rozsah ±40 / ±80 / ±160 / ±320 mV)
//  - nastavení ADC rozlišení a počtu průměrování
//  - programování kalibračního registru (proud_LSB + R_shunt)
//  - softwarový reset
// ---------------------------------------------------------------------------

class INA219_Sensor {
public:
    // -----------------------------------------------------------------------
    // Konstanty
    // -----------------------------------------------------------------------
    static const uint8_t  DEFAULT_ADDRESS   = 0x40; // A1=GND, A0=GND
    static const uint8_t  AVERAGE_SAMPLES   = 10;   // vzorky pro neblokující měření

    // Rozsah sběrnicového napětí (BRNG bit 13)
    enum BusVoltageRange : uint8_t {
        RANGE_16V = 0,
        RANGE_32V = 1   // výchozí
    };

    // PGA zisk – rozsah shunt napětí (bity 11-12)
    enum PGAGain : uint8_t {
        PGA_1   = 0b00,  // ±40 mV
        PGA_2   = 0b01,  // ±80 mV
        PGA_4   = 0b10,  // ±160 mV
        PGA_8   = 0b11   // ±320 mV (výchozí)
    };

    // ADC rozlišení / počet průměrování (pro BADC i SADC)
    enum ADCResolution : uint8_t {
        ADC_9BIT    = 0b0000,  // 84 µs
        ADC_10BIT   = 0b0001,  // 148 µs
        ADC_11BIT   = 0b0010,  // 276 µs
        ADC_12BIT   = 0b0011,  // 532 µs (výchozí)
        ADC_AVG2    = 0b1001,  // 2 vzorky, 1.06 ms
        ADC_AVG4    = 0b1010,  // 4 vzorky, 2.13 ms
        ADC_AVG8    = 0b1011,  // 8 vzorků, 4.26 ms
        ADC_AVG16   = 0b1100,  // 16 vzorků, 8.51 ms
        ADC_AVG32   = 0b1101,  // 32 vzorků, 17.02 ms
        ADC_AVG64   = 0b1110,  // 64 vzorků, 34.05 ms
        ADC_AVG128  = 0b1111   // 128 vzorků, 68.10 ms
    };

    // -----------------------------------------------------------------------
    // Konstruktor
    // -----------------------------------------------------------------------
    INA219_Sensor(TwoWire &wire = Wire, uint8_t address = DEFAULT_ADDRESS);

    // -----------------------------------------------------------------------
    // Inicializace
    // -----------------------------------------------------------------------

    // Inicializuje senzor s daným shunt odporem [Ω] a max. proudem [A].
    // Volitelně nastaví rozsahy a ADC. Vrací true při úspěchu.
    bool begin(float rShuntOhm      = 0.1f,
               float maxCurrentA    = 3.2f,
               BusVoltageRange bvr  = RANGE_32V,
               PGAGain pga          = PGA_8,
               ADCResolution badc   = ADC_12BIT,
               ADCResolution sadc   = ADC_12BIT);

    // Softwarový reset (všechny registry na výchozí hodnoty)
    void reset();

    // -----------------------------------------------------------------------
    // Konfigurace (lze volat i po begin())
    // -----------------------------------------------------------------------

    // Přepočítá a zapíše kalibrační registr z nových hodnot R_shunt a I_max
    void setCalibration(float rShuntOhm, float maxCurrentA);

    // Nastaví rozsah sběrnicového napětí
    void setBusVoltageRange(BusVoltageRange bvr);

    // Nastaví PGA zisk (shunt rozsah)
    void setPGAGain(PGAGain pga);

    // Nastaví ADC rozlišení / průměrování sběrnicového napětí
    void setBusADC(ADCResolution res);

    // Nastaví ADC rozlišení / průměrování shunt napětí
    void setShuntADC(ADCResolution res);

    // -----------------------------------------------------------------------
    // Jednorázová (blokující) čtení
    // -----------------------------------------------------------------------

    // Vrací proud [A] – blokující, čeká na dokončení konverze
    float readCurrentA();

    // Vrací proud [mA]
    float readCurrentMA();

    // Vrací shunt napětí [mV]
    float readShuntVoltageMV();

    // Vrací sběrnicové napětí [V]
    float readBusVoltageV();

    // Vrací výkon [W] (z Power registru)
    float readPowerW();

    // Vrací výkon [mW]
    float readPowerMW();

    // -----------------------------------------------------------------------
    // Stav a info
    // -----------------------------------------------------------------------

    bool    isConnected();
    uint8_t getAddress() const;

    // Vrací true pokud je nastaven Math Overflow Flag
    bool    isMathOverflow();

    // Vrací true pokud je nastaven Conversion Ready bit
    bool    isConversionReady();

    // Vrací naposledy změřený proud [A] (z posledního blokujícího nebo update() čtení)
    float   getLastCurrentA()  const;
    float   getLastCurrentMA() const;
    float   getLastBusVoltageV() const;
    float   getLastPowerW()    const;

    // Vrací LSB proudu [A/bit] a výkonu [W/bit] pro aktuální kalibraci
    float   getCurrentLSB()    const;
    float   getPowerLSB()      const;

    // Vrací nastavený shunt odpor [Ω]
    float   getRShunt()        const;

    // -----------------------------------------------------------------------
    // Neblokující průměrované měření (stejný vzor jako VL53L1X_Sensor)
    // -----------------------------------------------------------------------

    // Zahájí nové průměrované měření AVERAGE_SAMPLES vzorků.
    // Vrací false pokud senzor není inicializován.
    bool startMeasure();

    // Vrací true pokud bylo nasbíráno a zprůměrováno AVERAGE_SAMPLES vzorků.
    bool dataReady();

    // Vrací zprůměrovaný proud [A] z posledního dokončeného měření.
    float getAveragedCurrentA();

    // Vrací zprůměrovaný proud [mA] z posledního dokončeného měření.
    float getAveragedCurrentMA();

    // Volat periodicky ve smyčce – bez blokování načte vzorek,
    // jakmile INA219 hlásí dokončenou konverzi.
    // Po nasbírání AVERAGE_SAMPLES vzorků nastaví flag dataReady.
    void update();

private:
    // -----------------------------------------------------------------------
    // Nízkoúrovňový přístup k registrům
    // -----------------------------------------------------------------------
    void     writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);

    void applyConfiguration();

    // -----------------------------------------------------------------------
    // Registry INA219
    // -----------------------------------------------------------------------
    static const uint8_t REG_CONFIG      = 0x00;
    static const uint8_t REG_SHUNTVOLT  = 0x01;
    static const uint8_t REG_BUSVOLT    = 0x02;
    static const uint8_t REG_POWER      = 0x03;
    static const uint8_t REG_CURRENT    = 0x04;
    static const uint8_t REG_CALIBRATION= 0x05;

    // -----------------------------------------------------------------------
    // Členské proměnné
    // -----------------------------------------------------------------------
    TwoWire *_wire;
    uint8_t  _address = 0x40;

    bool     _initialized;

    // Konfigurace
    BusVoltageRange _bvr;
    PGAGain         _pga;
    ADCResolution   _badc;
    ADCResolution   _sadc;

    // Kalibrace
    float    _rShunt;        // shunt odpor [Ω]
    float    _currentLSB;    // [A/bit]
    float    _powerLSB;      // [W/bit]

    // Poslední naměřené hodnoty
    float    _lastCurrentA;
    float    _lastBusVoltageV;
    float    _lastPowerW;

    // Neblokující průměrování
    bool     _measuring;
    bool     _dataReady;
    float _filteredCurrentA = 0.0f;
    float _alpha = 0.1f;
    bool  _filterInitialized = false;
};

#endif // INA219_SENSOR_H
