#include <Wire.h>

uint8_t state = 0;
uint8_t dir = 0;
float requestedTorque = 0;

uint8_t waitForByte()
{
  while(true)
  {
    if (Serial.available() > 0)
    {
      return Serial.parseInt();
    }
    else
    {
      delay(50);
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop()
{
  switch(state)
  {
    case 0:
      Serial.println("0 pro zjisteni proudu, jine pro nastaveni motoru");
      state = 1;
      break;

    case 1:
      if (waitForByte() == 0)
      {
        state = 3;
      }
      else
      {
        state = 2;
      }

      break;

    case 2:
      Serial.println("zadejte smer 0/jine");
      if(waitForByte() == 0)
      {
        dir = 0;
      }
      else
      {
        dir = 1;
      }

      Serial.println("zadejte pozadovany moment: |cislo 0 - 200| --> |0 - 2 Nm|");
      requestedTorque = constrain((waitForByte() * 0.01), 0.0, 200.0);

      Serial.print("nastavuji smer = ");
      Serial.print(dir);
      Serial.print(", moment = ");
      Serial.println(requestedTorque);

      Wire.beginTransmission(10);
      Wire.write((uint8_t*)&requestedTorque, 4);
      Wire.endTransmission(); 

      state = 0;
      break;

    case 3:
      Wire.requestFrom(1, 4);
      state = 0;
      Serial.println("pozadano o data");
      break;
  }

  if(Wire.available() == 5)
  {
    Serial.println("data prisla");
    uint8_t receivedBytes[5];

    for(uint8_t i = 0; i < 5; i++)
    {
      receivedBytes[i] = Wire.read();
      Serial.println(receivedBytes[i]);
    }

    float receivedFloat;
    memcpy(&receivedFloat, receivedBytes + 1, sizeof(receivedFloat));
    Serial.print("Stav: ");
    Serial.print(receivedBytes[0]);
    Serial.print(", moment: ");
    Serial.print(receivedFloat);
    Serial.println(" Nm");
  }

  delay(100);
}