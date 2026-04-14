#include <Arduino.h>
#include <HardwareSerial.h>

#include "CubeMarsV2.h"
#include "LimitSwitch.h"
#include "as5600.h"
#include "LinearAxis.h"
#include "ADS122C04_LIB.h"

#define manualControl

// ===== LINEAR AXIS =====
LinearAxis linearAxis(
  13, // STEP
  12, // DIR
  14, // EN
  5,  // CS (TMC5160)
  18, // SCK
  19, // MISO
  23, // MOSI
  15, // horní koncák
  0,  // dolní koncák
  21, // SDA encoder
  22, // SCL encoder
  0x40 // adresa AS5600
);

enum menuEnum
{
  mainMenu,
  speedSelection
};

enum menuEnum menuState = mainMenu;

int32_t eRPM = 0;

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

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) return;

  if (cmd == "U") {
    linearAxis.moveUp();
  }
  else if (cmd == "D") {
    linearAxis.moveDown();
  }
  else if (cmd == "S") {
    linearAxis.stop();
  }
  else if (cmd == "+") {
    linearAxis.changeSpeedRelative(100);
  }
  else if (cmd == "-") {
    linearAxis.changeSpeedRelative(-100);
  }
  else if (cmd.startsWith("R")) {
    int value = cmd.substring(1).toInt();
    if (value > 0) {
      linearAxis.setSpeed((uint32_t)value);
    } else {
      Serial.println("Chyba: pouzij napr. R100");
    }
  }
  else if (cmd == "X") {
    linearAxis.zero();
  }
  else if (cmd == "?") {
    linearAxis.printStatus(Serial);
  }
  else if (cmd == "SG") {
    linearAxis.setLoadPrintEnabled(true);
  }
  else if (cmd.startsWith("M")) {   //CUBEMARS MOTOR COMMANDS
    int value = cmd.substring(1).toInt();
      eRPM = value
  }
  else if(cmd == "MD")
  {
    printMotorData(motorDriver);
  }
  else if(cmd == "MSR")
  {
    Serial.println(Measure start);
  }
  else if(cmd == "TRE")
  {
    Serial.println(Tare start);
  }
  else if(cmd == "CLB")
  {
    Serial.println(Calibration start);
  }

  else {
    Serial.println("Neznamy prikaz.");
    Serial.println("Pouzij: U, D, S, R100, +, -, A2000, X, ?, MSR, TRE, CLB");
  }
}

/*I2C
SDA...GPIO21
SCL...GPIO22
*/

//---------PERIPHERAL CLASSES
LimitSwitch limitSwitchTop(15);
LimitSwitch limitSwitchBottom(0);

CubeMarsV2 motorDriver(Serial2, Serial0, 16, 17);

ADS122C04 adc;

//---------TIMING
uint32_t nextLoopMillis = 0;
uint32_t deltaMillis = 100;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  delay(1000);

  // linear axis
  if (!linearAxis.begin(600, 16)) {
    Serial.println("Linear axis FAILED");
  }

  adc.init();

  mainMenuPrint();
}

void loop()
{
  if(millis() > nextLoopMillis)
  {
    nextLoopMillis += deltaMillis;

    motorDriver.handleRX();
    motorDriver.setERPM(eRPM);
    motorDriver.requestTmpCurrRPM();


    linearAxis.update();

    if (Serial.available())
    {
      String cmd = Serial.readStringUntil('\n');
      handleCommand(cmd);
    }
  }
}



