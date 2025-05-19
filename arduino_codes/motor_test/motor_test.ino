/******************************************************************************
 * @file    motor_test.ino
 * @author  Martin Kriz
 * @brief   Code for arduino nano to test communication with motor subsystem.
 * @date    2025-04-19
 ******************************************************************************/

#include <Wire.h>

// Test variables with random values
uint8_t state = 1;
uint8_t error = 0;
int8_t rps = -100;
int8_t torque = 75;
uint8_t temperature = 25;
unsigned long firstTen = 0;
unsigned long lastTime = 0;
unsigned long sum = 0;
unsigned long counter = 0;
unsigned long tmax = 0;
unsigned long tmin = 10000;


void setup() {
  Wire.begin(10);
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  Serial.print("tavg: ");
  Serial.print(sum/counter);
  Serial.println(" ms");

  Serial.print("tmin: ");
  Serial.print(tmin);
  Serial.println(" ms");

  Serial.print("tmax: ");
  Serial.print(tmax);
  Serial.println(" ms");
  delay(100);
}

/**
 * @brief   Handler of recieving messages from central microcontroller (nano <- pico).
 * @param   howMany Number of bytes to recieve.
 * @return  void
 */
void receiveEvent(int howMany) {
    //Serial.println("Recieved");
    //Serial.println(howMany);
    int8_t a = Wire.read();
    Serial.println(a);
    }

/**
 * @brief   Handler of requesting messages from central microcontroller (nano -> pico).
 * @return  void
 */
void requestEvent(){
  time_measure();
  firstTen++;
  const uint8_t outputArraySize = 4;
  uint8_t outputArray[outputArraySize] = {0};

  uint8_t x = (error << 2) | state;
  outputArray[0] = x;
  memcpy((outputArray+1), &rps, sizeof(rps));
  memcpy((outputArray+2), &torque, sizeof(torque));
  memcpy((outputArray+3), &temperature, sizeof(temperature));
  Wire.write(outputArray, 4);
  //Serial.println("Request done!");
  }

/**
 * @brief   Measures the time intervals between communication events.
 * @return  void
 */
void time_measure()
  {
    unsigned long currentTime = millis();
    unsigned long interval = currentTime - lastTime; 
    lastTime = currentTime; 

    if (firstTen > 10)
    {
      sum = sum + interval;
      counter++;

      if (interval > tmax)
        tmax = interval;

      if (interval < tmin)
        tmin = interval;
    }
  }