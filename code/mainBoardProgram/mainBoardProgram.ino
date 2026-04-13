#include <Arduino.h>
#include <HardwareSerial.h>

#include "CubeMarsV2.h"
#include "LimitSwitch.h"
#include "as5600.h"
#include "LinearAxis.h"

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

/*I2C
SDA...GPIO21
SCL...GPIO22
*/

//---------PERIPHERAL CLASSES
LimitSwitch limitSwitchTop(15);
LimitSwitch limitSwitchBottom(0);

CubeMarsV2 motorDriver(Serial2, Serial0, 16, 17);

enum menuEnum
{
  mainMenu,
  speedSelection
};

enum menuEnum menuState = mainMenu;

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
  else if (cmd.startsWith("RM")) { // mm/s
  float value = cmd.substring(2).toFloat();
  if (value > 0) {
    linearAxis.setSpeedMMps(value);
    Serial.print("Rychlost mm/s: ");
    Serial.println(linearAxis.getSpeedMMps());
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
  else if (cmd == "NSG") {
    linearAxis.setLoadPrintEnabled(false);
  }
  else if (cmd.startsWith("M")) {   //CUBEMARS MOTOR COMMANDS
    int value = cmd.substring(1).toInt();
      motorDriver.setRPM(value);
  }
  else if(cmd == "Z")
  {
    printMotorData(motorDriver);
  }

  else {
    Serial.println("Neznamy prikaz.");
    Serial.println("Pouzij: U, D, S, R100, +, -, A2000, X, ?");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  // linear axis
  if (!linearAxis.begin(600, 16)) {
    Serial.println("Linear axis FAILED");
  }

}

void loop()
{
  motorDriver.update();

  linearAxis.update();

  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}



