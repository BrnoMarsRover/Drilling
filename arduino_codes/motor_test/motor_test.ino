#include <Wire.h>


uint8_t state = 1;
uint8_t error = 0;
int8_t rps = -100;
int8_t torque = 75;
uint8_t temperature = 25;
unsigned long lastTime = 0; // Uloží čas posledního měření


void setup() {
  // put your setup code here, to run once:
  Wire.begin(10);
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}

void receiveEvent(int howMany) {
    time_measure();
    //Serial.println("Recieved");
    //Serial.println(howMany);
    int8_t a = Wire.read();
    Serial.println(a);
    }


void requestEvent(){
  const uint8_t outputArraySize = 4;
  uint8_t outputArray[outputArraySize] = {0};

  uint8_t x = (error << 2) | state;
  outputArray[0] = x;
  memcpy((outputArray+1), &rps, sizeof(rps));
  memcpy((outputArray+2), &torque, sizeof(torque));
  memcpy((outputArray+3), &temperature, sizeof(temperature));
  Wire.write(outputArray, 4);
  //Serial.println("Finito");
  }

    void time_measure()
  {
    unsigned long currentTime = millis();
    unsigned long interval = currentTime - lastTime; 
    lastTime = currentTime; 

    Serial.print("Interval mezi vzorky: ");
    Serial.print(interval);
    Serial.println(" ms");
  }