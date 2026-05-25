#ifndef VL53L1X_WRAPPER_H
#define VL53L1X_WRAPPER_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

class VL53L1X_Sensor {
public:
    static const uint8_t DEFAULT_ADDRESS = 0x29;
    static const uint8_t AVERAGE_SAMPLES = 10;

    VL53L1X_Sensor(TwoWire &wire = Wire, uint8_t address = DEFAULT_ADDRESS);

    bool begin();
    void setAddress(uint8_t address);
    uint8_t getAddress() const;
    bool isConnected();

    void startContinuous(uint32_t period_ms = 50);
    void stopContinuous();

    uint16_t readSingle();      // Jedno měření "na vyžádání" (blokující)
    uint16_t readContinuous();  // Čtení v běžícím módu (blokující)

    float getDistanceCM();
    float getDistanceM();
    bool isWithinRange(uint16_t min_mm, uint16_t max_mm);

    // --- Neblokující průměrované měření ---
    // Zahájí nové měření, resetuje flag dataReady
    bool startMeasure();

    // Vrací true pokud bylo nasbíráno a zprůměrováno AVERAGE_SAMPLES vzorků
    bool dataReady();

    // Vrací zprůměrovanou vzdálenost v mm z posledního dokončeného měření
    float getDistanceMM();

    // Volat periodicky ve smyčce – přečte výsledek pokud ho senzor má připravený,
    // bez čekání. Jakmile nasbírá AVERAGE_SAMPLES vzorků, nastaví flag dataReady.
    void update();

private:
    TwoWire *_wire;
    uint8_t  _address;
    VL53L1X  _sensor;

    uint16_t _lastDistance;  
    bool     _initialized;

    bool     _measuring;          // probíhá sběr vzorků
    bool     _dataReady;          // bylo dokončeno průměrování
    uint8_t  _sampleCount;        // počet nasbíraných vzorků
    uint32_t _sampleSum;          // součet vzorků
    float    _averagedDistance;   // výsledná průměrná vzdálenost [mm]
    bool     _samplePending;      // čekáme na výsledek aktuálního HW měření
};

#endif