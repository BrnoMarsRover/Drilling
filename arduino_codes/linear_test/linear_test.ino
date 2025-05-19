/******************************************************************************
 * @file    linear_test.ino
 * @author  Martin Kriz
 * @brief   Code for arduino nano to test communication with linear subsystem.
 * @date    2025-04-19
 ******************************************************************************/

#include <Wire.h>

// Test variables with random values
uint8_t state = 4;
uint8_t error = 0;
uint16_t height = 300;
uint16_t toGround = 10;
unsigned long lastTime = 0;


void setup() {
  Wire.begin(9);
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  height--; // Value change because of testing position regulator
  delay(100);
}

/**
 * @brief   Handler of recieving messages from central microcontroller (nano <- pico).
 * @param   howMany Number of bytes to recieve.
 * @return  void
 */
void receiveEvent(int howMany) {
    const uint8_t inputArraySize = 2;
    char inputArray[inputArraySize] = {0};
    //Serial.println("Recieved");
    //Serial.println(howMany);
    uint8_t a = Wire.read();
    uint8_t b = Wire.read();
    Serial.println(a);
    Serial.println(b);
    }


/**
 * @brief   Handler of requesting messages from central microcontroller (nano -> pico).
 * @return  void
 */
void requestEvent(){
  const uint8_t outputArraySize = 5;
  uint8_t outputArray[outputArraySize] = {0};

  uint8_t x = (error << 3) | state;
  outputArray[0] = x;
  memcpy((outputArray+1), &height, sizeof(height));
  memcpy((outputArray+3), &toGround, sizeof(toGround));
  Wire.write(outputArray, 5);
  //Serial.println("Request done!");
  }
