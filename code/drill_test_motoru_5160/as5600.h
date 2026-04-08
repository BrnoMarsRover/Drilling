#pragma once
#include <Arduino.h>
#include <Wire.h>

class AS5600L {
public:
  // Výchozí adresa AS5600L je 0x40
  explicit AS5600L(uint8_t address = 0x40, TwoWire &wire = Wire);

  // Inicializace I2C + první načtení polohy
  bool begin(int sdaPin, int sclPin, uint32_t frequency = 400000);

  // Změna adresy za běhu (pokud máš čidlo naprogramované na jinou adresu)
  void setAddress(uint8_t address);
  uint8_t getAddress() const;

  // Musí se volat pravidelně v loop(), aby se správně počítaly plné otáčky
  bool update();

  // Nastaví aktuální mechanickou polohu jako nulu
  void setZero();

  // Stav senzoru
  bool isConnected();
  uint8_t readStatus();
  bool magnetDetected();
  bool magnetTooWeak();
  bool magnetTooStrong();

  // Základní měření
  uint16_t readRawAngle();      // 0..4095 (RAW ANGLE)
  uint16_t readAngle();         // 0..4095 (ANGLE)
  float getAngleDegrees();      // 0..360

  // Víceotáčkové měření
  int32_t getTurns() const;               // celé otáčky od posledního setZero()
  float getRevolutions() const;           // otáčky včetně zlomku od setZero()
  float getTotalAngleDegrees() const;     // celkový úhel od setZero()

  // Lineární převod
  float getLinearDistanceMM(float mmPerRevolution) const;
  float getLinearDistanceCM(float mmPerRevolution) const;
  float getLinearDistanceM(float mmPerRevolution) const;

private:
  static constexpr uint8_t REG_STATUS     = 0x0B;
  static constexpr uint8_t REG_RAW_ANGLE  = 0x0C;
  static constexpr uint8_t REG_ANGLE      = 0x0E;

  static constexpr int32_t COUNTS_PER_REV = 4096;
  static constexpr int32_t WRAP_THRESHOLD = COUNTS_PER_REV / 2; // 2048

  TwoWire *_wire;
  uint8_t _address;

  bool _initialized = false;
  bool _hasSample = false;

  uint16_t _rawAngle = 0;       // poslední raw úhel 0..4095
  uint16_t _lastRawAngle = 0;   // předchozí raw úhel
  int32_t _turnCounter = 0;     // absolutní čítač plných otáček od zapnutí
  int32_t _zeroOffsetCounts = 0;

  bool read8(uint8_t reg, uint8_t &value);
  bool read16(uint8_t reg, uint16_t &value);

  int32_t absoluteCounts() const;         // celková poloha v counts od startu
  int32_t relativeCounts() const;         // poloha vůči setZero()
};