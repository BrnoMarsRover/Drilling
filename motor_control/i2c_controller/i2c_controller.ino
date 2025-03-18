#include <Wire.h>

uint8_t state = 0;

const uint8_t slaveAddress = 10;
const uint8_t inputArraySize = 4;

uint8_t waitForByte()
{
  while(true)
  {
    if (Serial.available() > 0)
    {
      uint8_t receivedByte = Serial.parseInt();
      //Serial.print("precteno ");
      //Serial.println(receivedByte);
      return receivedByte;
    }
    else
    {
      delay(1000);
      Serial.println("cekam");
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop()
{
  //Serial.print("jsem ve stavu:");
  //Serial.println(state);

  if(state == 0)
  {
    Serial.println("0 pro cteni dat, jine pro nastaveni motoru");

    uint8_t receivedByte = waitForByte();
    //Serial.print("dostal jsem ");
    //Serial.println(receivedByte);
    if (receivedByte == 0)
    {
      //Serial.println("jdu do stavu 3 ");
      state = 3;
    }
    else
    {
      //Serial.println("jdu do stavu 2 ");
      state = 2;
    }
  }
  else if(state == 2)
  {
    Serial.println("zadejte pozadovanou rychlost: |0 ... 200| -> |-3 ... 3|");
    uint8_t setpointByte = waitForByte();
    //Serial.println("rychlost zadana");

    int8_t newSetpoint = int8_t(constrain(int16_t(setpointByte) - 100, -100, 100));
      
    Serial.print("nastavuji rychlost = ");
    Serial.println(newSetpoint);

    Wire.beginTransmission(slaveAddress);
    Wire.write(newSetpoint);
    Wire.endTransmission(); 

    state = 0;
  }
  else if(state == 3)
  {
    Serial.println("zadam o data");
    Wire.requestFrom(slaveAddress, inputArraySize);
    state = 0;
    Serial.println("pozadano o data");
  }

  if(Wire.available() == inputArraySize)
  {
    Serial.println("data prisla");
    uint8_t receivedBytes[inputArraySize];

    for(uint8_t i = 0; i < inputArraySize; i++)
    {
      receivedBytes[i] = Wire.read();
      Serial.println(receivedBytes[i]);
    }

    Serial.print("Stav: ");
    Serial.print(receivedBytes[0]);
    Serial.print(", rychlost: ");
    Serial.print((int8_t)receivedBytes[1]);
    Serial.print(" ot/s");
    Serial.print(", moment: ");
    Serial.print((int8_t)receivedBytes[2]);
    Serial.println(" Nm");
    Serial.print(receivedBytes[3]);
    Serial.println(" °C");
  }

  delay(100);
}