#include <Arduino.h>
#include <vector>
#include <HardwareSerial.h>

#include "CubeMarsV2.h"

#define manualControl

enum menuEnum
{
  mainMenu,
  speedSelection
};

enum menuEnum menuState = mainMenu;

int32_t eRPM = 0;

void mainMenuPrint()
{
  Serial.print("\nMenu:\n1: zadat eRPM\n2: pozadat o malo dat\n3: pozadat o vsechna data\n");
}

void printMotorData(CubeMarsV2 motorDriverArg)
{
  Serial.print("MOS tmp: ");
  Serial.println(motorDriverArg.getMOSTmp());
  Serial.print("Motor tmp: ");
  Serial.println(motorDriverArg.getMotorTmp());
  Serial.print("Current: ");
  Serial.println(motorDriverArg.getCurrent());
  Serial.print("Speed: ");
  Serial.println(motorDriverArg.getRPM());
}

CubeMarsV2 motorDriver(Serial2, 16,17);

void setup() {
  Serial.begin(921600);
  Serial2.begin(921600);

  delay(1000);

  mainMenuPrint();
}

void loop()
{
#if defined(manualControl)

  switch(menuState)
  {
    case mainMenu:
      if (Serial.available())
      {
        int32_t input = Serial.parseInt();
        switch(input)
        {
          case 1:
            Serial.println("Zadejte eRPM:");
            menuState = speedSelection;
            break;
          case 2:
            Serial.println("Zadam o data");
            motorDriver.requestTmpCurrRPM();
            printMotorData(motorDriver);
            mainMenuPrint();
            break;
          case 3:
            Serial.println("Zadam o vsechna data");
            motorDriver.requestAllData();
            mainMenuPrint();
            break;
        }
      }
      break;
    case speedSelection:
      if (Serial.available())
      {
        eRPM = Serial.parseInt();
        Serial.print("Zadavam eRPM: ");
        Serial.println(eRPM);

        menuState = mainMenu;
        mainMenuPrint();
      }
      break;
  }

#else


#endif

  /*
  while(Serial1.available())
  {
    int inByte = Serial1.read();
    Serial.println(inByte, HEX);
  }
*/
  while(Serial2.available())
  {
    int inByte = Serial2.read();
    Serial.println(inByte, HEX);
  }
  
  motorDriver.handleRX();
  motorDriver.setERPM(eRPM);

  delay(100);
}