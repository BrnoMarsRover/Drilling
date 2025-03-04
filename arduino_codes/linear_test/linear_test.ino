#include <Wire.h>


int x = 3;
uint16_t height = 101;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(9);
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
    const uint8_t inputArraySize = 2;
    char inputArray[inputArraySize] = {0};

    Serial.println("Recieved");
    Serial.println(howMany);
    uint8_t a = Wire.read();
    uint8_t b = Wire.read();
    Serial.print("Elapsed Time: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");
    x = a + b;
    startTime = 0;
    }


void requestEvent(){
  const uint8_t outputArraySize = 3;
  uint8_t outputArray[outputArraySize] = {0};

  outputArray[0] = x;
  memcpy((outputArray+1), &height, sizeof(height));
  Wire.write(outputArray, 3);
  Serial.println("Finito");
  }