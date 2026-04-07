#include "ads122c04_lib.h"
#include <Wire.h>

const int btn_tmp = 4;
const int n_reset = 5;

ADS122C04 adc;          // default address 0x40 (A1=DGND, A0=DGND)

void setup() {
    Serial.begin(9600);
    while (!Serial) delay(10);

    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    pinMode(btn_tmp, INPUT_PULLUP); // erase for ESP
    pinMode(n_reset, OUTPUT);
    digitalWrite(n_reset, HIGH);

    Wire.begin();

    adc.init();
    /*
    // Verify register contents
    Serial.println(F("Register readback:"));
    for(uint8_t i = 0; i < 4; i++){
        Wire.beginTransmission(0x45);
        Wire.write(0x20 | (i << 2));    // RREG command
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)0x45, (uint8_t)1);
        uint8_t val = Wire.read();
        Serial.print(F("REG")); Serial.print(i);
        Serial.print(F(" = 0x")); Serial.println(val, HEX);
    }
    Serial.println(F("Expected: 0x0E, 0x08, 0x05, 0x70"));
    */
    // Check chip temperature
    Serial.print(F("Chip temperature: "));
    Serial.print(adc.read_temperature(), 4);
    Serial.println(F(" °C"));
    
    adc.scale_calibrate();
    adc.tare();
    Serial.println(F("Scale ready"));
}

void loop() {
  if(digitalRead(btn_tmp) == LOW){ // erase for ESP
    adc.tare();
  }
  //Serial.println(adc.read());
  //delay(50);
  Serial.print(adc.measure_weight(), 4);
  Serial.println("g");
  delay(1000);
}