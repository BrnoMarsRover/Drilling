#include <Wire.h>


uint8_t state = 1;
uint8_t error = 0;
int8_t rps = -100;
int8_t torque = 75;
uint8_t temperature = 25;
unsigned long firstTen = 0;
unsigned long lastTime = 0; // Uloží čas posledního měření
unsigned long sum = 0;
unsigned long counter = 0;
unsigned long tmax = 0;
unsigned long tmin = 10000;


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

void receiveEvent(int howMany) {

    //Serial.println("Recieved");
    //Serial.println(howMany);
    int8_t a = Wire.read();
    Serial.println(a);
    }


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
  //Serial.println("Finito");
  }

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



    //Serial.print("Interval mezi vzorky: ");
    //Serial.print(interval);
    //Serial.println(" ms");
  }