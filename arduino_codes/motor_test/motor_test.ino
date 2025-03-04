#include <Wire.h>


int x = 3;
float torque = 1.69;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(10);
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

    elapsedTime = millis() - startTime;  // Calculate elapsed time
    Serial.print("Elapsed Time: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");
}

void receiveEvent(int howMany) {
    elapsedTime = millis() - startTime;
    const uint8_t inputArraySize = 4;
    char inputArray[inputArraySize] = {0};

    Serial.println("Recieved");
    Serial.println(howMany);
    char c = Wire.read();
    Wire.readBytes(inputArray, inputArraySize);
    float *floatPtr = (float *)inputArray;
    Serial.println(*floatPtr);
    Serial.print("Elapsed Time: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");
    torque = *floatPtr + 0.2;
    x = c + 1;
    startTime = 0;
    }


void requestEvent(){
  const uint8_t outputArraySize = 5;
  uint8_t outputArray[outputArraySize] = {0};

  outputArray[0] = x;
  memcpy((outputArray+1), &torque, sizeof(torque));
  Wire.write(outputArray, 5);
  Serial.println("Finito");
  }