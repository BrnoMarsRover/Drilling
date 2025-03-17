#include "FIR.h"

//PINS
const uint8_t OutPin_L_EN = 3;
const uint8_t OutPin_R_EN = 4;
const uint8_t OutPin_L_PWM = 5;
const uint8_t OutPin_R_PWM = 6;

const uint8_t InPin_AmmeterRC = A2;
const uint8_t InPin_AmmeterDirect = A3;

uint32_t lastMillis = 0;

FIR currentFilter(8);

void setup() {
  pinMode(OutPin_L_EN, OUTPUT);
  pinMode(OutPin_L_PWM, OUTPUT);
  pinMode(OutPin_R_EN, OUTPUT);
  pinMode(OutPin_R_PWM, OUTPUT);

  pinMode(InPin_AmmeterRC, INPUT);
  pinMode(InPin_AmmeterDirect, INPUT);  

  digitalWrite(OutPin_R_EN, HIGH);
  digitalWrite(OutPin_L_EN, HIGH);

  analogWrite(OutPin_L_PWM, 20);
  analogWrite(OutPin_R_PWM, 0);

  Serial.begin(9600);
}

void loop() {
  uint32_t currentMillis = millis();
  uint32_t timeDiff = currentMillis - lastMillis;
  lastMillis = currentMillis;

  Serial.print("t:");  
  Serial.print(timeDiff);

  uint16_t amDirect = analogRead(InPin_AmmeterDirect);
  uint16_t amRC = analogRead(InPin_AmmeterRC);

  Serial.print(" d:");
  Serial.print(amDirect);
  Serial.print(" f:");
  Serial.println(amRC);
  Serial.print(" ff:");
  Serial.println(currentFilter.updateOutput(amRC));

  ///delay(100);
}
