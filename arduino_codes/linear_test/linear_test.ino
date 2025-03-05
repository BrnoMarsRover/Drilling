#include <Wire.h>

int x = 3;
uint16_t height = 101;
unsigned long elapsedTime_request = 0;
unsigned long elapsedTime_recieve = 0;


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
  Serial.print("Elapsed Time request: ");
  Serial.print(elapsedTime_request);
  Serial.println(" ms");

  Serial.print("Elapsed Time recieve: ");
  Serial.print(elapsedTime_recieve);
  Serial.println(" ms");
  delay(100);
}

void receiveEvent(int howMany) {
    elapsedTime_request = millis() - elapsedTime_request;
    const uint8_t inputArraySize = 2;
    char inputArray[inputArraySize] = {0};
    Serial.println("Recieved");
    Serial.println(howMany);
    uint8_t a = Wire.read();
    uint8_t b = Wire.read();

    x = a + b;
    }


void requestEvent(){
  elapsedTime_recieve = millis() - elapsedTime_recieve;
  const uint8_t outputArraySize = 3;
  uint8_t outputArray[outputArraySize] = {0};

  outputArray[0] = x;
  memcpy((outputArray+1), &height, sizeof(height));
  Wire.write(outputArray, 3);
  Serial.println("Finito");
  }