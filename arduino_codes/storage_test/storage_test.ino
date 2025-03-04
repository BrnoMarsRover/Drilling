#include <Wire.h>


int x = 3;
uint16_t ahoj = 101;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(x);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}

void receiveEvent(int howMany) {
    elapsedTime = millis() - startTime;
    Serial.println("Recieved");
    Serial.println(howMany);
    uint8_t a = Wire.read();
    Serial.print("Elapsed Time: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");
    x = a + 1;
    startTime = 0;
    }


void requestEvent(){
  const uint8_t outputArraySize = 3;
  uint8_t outputArray[outputArraySize] = {0};

  outputArray[0] = x;
  memcpy((outputArray+1), &ahoj, sizeof(ahoj));
  Wire.write(outputArray, 3);
  Serial.println("Finito");
  }