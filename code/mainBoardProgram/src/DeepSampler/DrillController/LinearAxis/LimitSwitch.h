#pragma once
#include <Arduino.h>

class LimitSwitch {
public:
  // debounceMillis - jak dlouho musi byt ctena hodnota stabilni,
  // nez se prijme jako novy stav (potlaci rusive impulzy ze stepperu)
  LimitSwitch(uint8_t pinNumber, uint16_t debounceMillis = 8);

  void update();
  bool isPressed() const;
  bool wasPressed() const;
  bool wasReleased() const;

  // Okamzite prevezme aktualni hodnotu pinu jako potvrzeny stav, bez debounce.
  // Urceno pro inicializaci - po pinMode() potrebuje vstup cas na ustaleni,
  // takze hodnota z konstruktoru jeste nemusi byt platna.
  void resync();

  // Nefiltrovana hodnota pinu - jen pro diagnostiku, ne pro rizeni
  bool rawIsPressed() const;

private:
  // Novy stav musi byt potvrzen alespon tolika vzorky po sobe.
  // Chrani i pri velmi pomale smycce, kdy by casovy limit sam nestacil.
  static constexpr uint8_t MIN_STABLE_SAMPLES = 2;

  uint8_t switchPin;
  uint16_t debounceMs;

  bool currentState;        // potvrzeny (filtrovany) stav
  bool candidateState;      // posledni ctena hodnota, ceka na potvrzeni
  uint32_t candidateSinceMs;
  uint8_t candidateSamples;

  bool pressedEvent;
  bool releasedEvent;
};
