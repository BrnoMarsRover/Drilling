#ifndef VL53L1X_WRAPPER_H
#define VL53L1X_WRAPPER_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h> 

class VL53L1X_Sensor {
public:
    static const uint8_t DEFAULT_ADDRESS = 0x29;

    VL53L1X_Sensor(TwoWire &wire = Wire, uint8_t address = DEFAULT_ADDRESS);

    bool begin();
    void setAddress(uint8_t address);
    uint8_t getAddress() const;
    bool isConnected();

    void startContinuous(uint32_t period_ms = 50);
    void stopContinuous();
    
    float readSingle();      // Jedno měření "na vyžádání"
    uint16_t readContinuous();  // Čtení v běžícím módu
    
    float getDistanceMM();
    float getDistanceCM();
    float getDistanceM();
    bool isWithinRange(uint16_t min_mm, uint16_t max_mm);

private:
    TwoWire *_wire;
    uint8_t _address;
    VL53L1X _sensor;
    uint16_t _lastDistance;
    float _averageDistance;
    bool _initialized = false;
};

#endif