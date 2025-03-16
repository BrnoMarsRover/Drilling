#include <Wire.h>

uint8_t slot = 191;
uint8_t command = 3;
uint16_t weight = 56;


void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
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
    Serial.println("Recieved");
    //Serial.println(howMany);
    command = Wire.read();
    Serial.println("command: ");
    Serial.println(command);
    }


void requestEvent(){
  const uint8_t outputArraySize = 3;
  uint8_t outputArray[outputArraySize] = {0};

  memcpy((outputArray+0), &weight, sizeof(weight));
  outputArray[2] = slot;
  Wire.write(outputArray, 3);
  //Serial.println("Finito");
  }