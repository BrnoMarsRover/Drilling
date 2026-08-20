#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(uint8_t pinNumber, uint16_t debounceMillis) {
  switchPin = pinNumber;
  debounceMs = debounceMillis;
  pinMode(switchPin, INPUT);

  currentState = (digitalRead(switchPin) == LOW);

  candidateState   = currentState;
  candidateSinceMs = millis();
  candidateSamples = MIN_STABLE_SAMPLES;

  pressedEvent = false;
  releasedEvent = false;
}

void LimitSwitch::update() {
  pressedEvent = false;
  releasedEvent = false;

  bool raw = (digitalRead(switchPin) == LOW);
  uint32_t now = millis();

  // Hodnota se lisi od kandidata -> zacina nove mereni stability.
  // Kratky rusivy impulz se timto zahodi, protoze pri dalsim vzorku
  // uz bude hodnota zase puvodni a citac se opet vynuluje.
  if (raw != candidateState) {
    candidateState   = raw;
    candidateSinceMs = now;
    candidateSamples = 1;
    return;
  }

  if (candidateSamples < MIN_STABLE_SAMPLES) {
    candidateSamples++;
  }

  // Kandidat uz odpovida potvrzenemu stavu - neni co menit
  if (candidateState == currentState) return;

  // Prijmout az kdyz je hodnota stabilni dost dlouho i pres dost vzorku
  if ((now - candidateSinceMs) < debounceMs) return;
  if (candidateSamples < MIN_STABLE_SAMPLES) return;

  currentState = candidateState;

  if (currentState) {
    pressedEvent = true;
  } else {
    releasedEvent = true;
  }
}

bool LimitSwitch::isPressed() {
  return currentState;
}

bool LimitSwitch::wasPressed() {
  return pressedEvent;
}

bool LimitSwitch::wasReleased() {
  return releasedEvent;
}

bool LimitSwitch::rawIsPressed() const {
  return digitalRead(switchPin) == LOW;
}
