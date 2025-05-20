/******************************************************************************
 * @file    storage_test.ino
 * @author  Martin Kriz
 * @brief   Code for arduino nano to test communication with storage subsystem.
 * @date    2025-04-19
 ******************************************************************************/

#include <Wire.h>

// Test variables with random values

uint8_t slot = 1;
uint8_t command = 3;
uint16_t weight = 112;


void setup() {
  Wire.begin(8);
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  delay(100);
}

/**
 * @brief   Handler of recieving messages from central microcontroller (nano <- pico).
 * @param   howMany Number of bytes to recieve.
 * @return  void
 */
void receiveEvent(int howMany) {
    Serial.println("Recieved");
    //Serial.println(howMany);
    command = Wire.read();
    Serial.println("command: ");
    Serial.println(command);
    }


/**
 * @brief   Handler of requesting messages from central microcontroller (nano -> pico).
 * @return  void
 */
void requestEvent(){
  const uint8_t outputArraySize = 3;
  uint8_t outputArray[outputArraySize] = {0};

  memcpy(outputArray, &weight, sizeof(weight));
  outputArray[2] = slot << 6;
  Wire.write(outputArray, 3);
  //Serial.println("Request done!");
  }