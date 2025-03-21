#include <Wire.h>

uint8_t state = 4;
uint8_t error = 1;
uint16_t height = 5;
uint16_t toGround = 10;
unsigned long lastTime = 0; // Uloží čas posledního měření


void setup() {
  // put your setup code here, to run once:
  Wire.begin(9);
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
    const uint8_t inputArraySize = 2;
    char inputArray[inputArraySize] = {0};
    //Serial.println("Recieved");
    //Serial.println(howMany);
    uint8_t a = Wire.read();
    uint8_t b = Wire.read();
    }


void requestEvent(){
  const uint8_t outputArraySize = 5;
  uint8_t outputArray[outputArraySize] = {0};

  uint8_t x = (error << 3) | state;
  outputArray[0] = x;
  memcpy((outputArray+1), &height, sizeof(height));
  memcpy((outputArray+3), &toGround, sizeof(toGround));
  Wire.write(outputArray, 5);
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