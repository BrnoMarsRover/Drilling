#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "CubeMarsV2.h"
#include "LimitSwitch.h"
#include "as5600.h"
#include "LinearAxis.h"
#include "ADS122C04_LIB.h"
#include "VL53L1X_Sensor.h"
#include "RoverComm.h"

//Define to command via ASCII messages through ArduinoIDE console.
//Comment out to utilize command via Python app.
#define SIMPLE_COMMAND


//-------I2C
constexpr uint8_t sdaPin = 21;
constexpr uint8_t sclPin = 22;
constexpr uint32_t i2cFrequency = 100000;
TwoWire I2CBus(0);

void setupI2CBus()
{
  // Print starting message for I2C recovery and setup
  Serial.println(F("Starting I2C Bus Recovery and Setup..."));

  // Check if I2C bus is free
  if (isI2CBusFree())
  {
    Serial.println(F("I2C bus is already free."));
  } else
  {
    // Bus is stuck, attempt recovery
    Serial.println(F("I2C bus is stuck, attempting recovery..."));
    if (i2cRecoverBus())
    {
      Serial.println(F("I2C bus recovery successful."));
    } else
    {
      Serial.println(F("I2C bus recovery failed!"));
    }
  }

  // Initialize the I2C bus
  I2CBus.begin(sdaPin, sclPin, i2cFrequency);
  I2CBus.setTimeOut(50);

  Serial.println(F("I2C Bus Setup Complete."));
}

bool isI2CBusFree()
{
  // Check if the SDA and SCL lines are both high (idle state for I2C)
  return (digitalRead(sdaPin) == HIGH && digitalRead(sclPin) == HIGH);
}

bool i2cRecoverBus()
{
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);

  if (digitalRead(sdaPin) == HIGH && digitalRead(sclPin) == HIGH)
  {
    return true; // Bus is already free
  }

  Serial.println(F("Attempting I2C bus recovery..."));

  // Try to generate clock pulses on SCL to free the bus
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  pinMode(sdaPin, INPUT_PULLUP);

  for (int i = 0; i < 20; i++)
  {  // Try up to 20 clock pulses
    digitalWrite(sclPin, LOW);
    delayMicroseconds(10);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);

    if (digitalRead(sdaPin) == HIGH)
    {
      Serial.println(F("SDA line released. Bus is free."));
      return true; // SDA line released, bus is free
    }
  }

  // Send STOP condition to release the bus
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(10);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(10);

  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);

  bool success = (digitalRead(sdaPin) == HIGH);
  Serial.println(success ? F("I2C recovery successful.") : F("I2C recovery failed."));

  return success;
}

//---------PERIPHERAL CLASSES

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
  I2CBus,  //i2c bus class
  0x42 // adresa AS5600
);

LimitSwitch limitSwitchTop(15);
LimitSwitch limitSwitchBottom(0);

CubeMarsV2 motorDriver(Serial2, Serial0, 16, 17);

VL53L1X_Sensor distanceSensor(I2CBus);

ADS122C04 *adc1 = nullptr; // declaration of ADC Deep sample class
ADS122C04 *adc2 = nullptr; // declaration of ADC Surface sample class


//------UART COMMUNICATION
#ifdef SIMPLE_COMMAND

void handleSimpleCommand()
{
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
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
    else if (cmd == "TOF") {
      Serial.print("Vzdálenost: ");
      Serial.println(distanceSensor.readSingle());
    }
    else if (cmd == "SG") {
      linearAxis.setLoadPrintEnabled(true);
    }
    else if (cmd == "NSG") {
      linearAxis.setLoadPrintEnabled(false);
    }
    else if (cmd == "H") {
      linearAxis.setHeightPrintEnabled(true);
    }
    else if (cmd == "NH") {
      linearAxis.setHeightPrintEnabled(false);
    }
    else if (cmd.startsWith("M")) {   //CUBEMARS MOTOR COMMANDS
      int value = cmd.substring(1).toInt();
        motorDriver.setRPM(value);
    }
    else if(cmd == "Z")
    {
      motorDriver.printMotorInfoToDebug();
    }
    else if (cmd.startsWith("WGH")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 :
                          cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      target->request_measure(); // old float w = target->measure_weight();
    }
    else if (cmd.startsWith("GW")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 :
                          cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      if(target->result_ready() == true){
        float w = target->get_last_weight();
        Serial.print(w, 4);
        Serial.println("g");
        if (w > 2000.0f) Serial.println("[ADC] Scale overweight");
      }
      else {
        Serial.println("[ADC] No weight result ready");
      }
    }
    else if (cmd.startsWith("TRE")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      Serial.println("[ADC] Tare start");
      target->set_tare();
    }
    else if (cmd.startsWith("CLB0")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      Serial.println("[ADC] Calibration start 0g");
      target->set_calibration_0();
    }
    else if (cmd.startsWith("CLB100")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      Serial.println("[ADC] Calibration start 100g");
      target->set_calibration_100();
    }
    else if (cmd.startsWith("ADCTMP")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      Serial.println("[ADC] tmp measured");
      target->request_tmp();
    }
    else if (cmd.startsWith("GT")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      Serial.print("[ADC] Chip tmp: ");
      Serial.print(target->get_last_temp(), 4);
      Serial.println(" °C");
    }
    else if (cmd.startsWith("ADCRST")) {
      //ADS122C04 *target = adc1;
      //if (target == nullptr) { Serial.println("ADC deep sample not assigned"); }
      adc1->reset();
      //ADS122C04 *target = adc2;
      //if (target == nullptr) { Serial.println("ADC surface sample not assigned"); }
      adc2->reset();
      Serial.println("[ADC] resets complete");
    }
    else {
      Serial.println("Neznamy prikaz.");
      Serial.println("Pouzij: U, D, S, R100, +, -, A2000, X, ?, WGH+D/S, TRE+D/S, CLB+0/100+D/S, ADCTMP+D/S, ADCRST, GW+D/S, GT+D/S");
    }
  }
}
#else
RoverComm roverComm(Serial);

void respondToMsg(RoverMessage msg)
{
  switch(msg.getCommandCode();)
  {
    case CMD_RESTART:
      break;
    case CMD_STATE:
    {
      float maxTmp = motorDriver.getMotorTmp();
      if(motorDriver.getMOSTmp() > maxTmp)
      {
        maxTmp = motorDriver.getMOSTmp();
      }
      roverComm.sendState((int8_t)linearAxis.getHeightCM(), (int16_t)motorDriver.getRPM(), (uint8_t)maxTmp, 0, STATE_READY );
      break;
    }
    case CMD_DRILL_AUTO:
      break;
    case CMD_STOP_AUTO:
      break;
    case CMD_CALIBRATE_HEIGHT:
      break;
    case CMD_DRILL_SPEED:
    {
      motorDriver.setRPM(msg.getInt16Arg());
      roverComm.sendAck(CMD_DRILL_SPEED);
      break;
    }
    case CMD_VERTICAL_SPEED:
    {
      linearAxis.setSpeed((int32_t)msg.getInt8Arg()*100);
      roverComm.sendAck(CMD_VERTICAL_SPEED);
      break;
    }
    case CMD_STORAGE_POSITION:
      break;
    case CMD_WEIGH_DEEP:
      break;
    case CMD_WEIGH_SURFACE:
      break;
    case CMD_GET_WEIGHT_DEEP:
      break;
    case CMD_GET_WEIGHT_SURFACE:
      break;
    case CMD_ROCK_OPEN:
      break;
    case CMD_ROCK_CLOSE:
      break;
    case CMD_SAND_OPEN:
      break;
    case CMD_SAND_CLOSE:
      break;
  }
}

#endif

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  setupI2CBus();

  // linear axis
  if (!linearAxis.begin(600, 16)) {
    Serial.println("Linear axis FAILED");
  }
  if (!distanceSensor.begin()) {
    Serial.println(" TOF FAILED");
  }

  motorDriver.begin();

  adc1 = new ADS122C04(
    I2CBus, //i2c bus class
    0x44 // address for deep sample weight
    //2 // n_reset pin
  );
  adc2 = new ADS122C04(
    I2CBus, //i2c bus class
    0x45 // address for deep sample weight
    //2 // n_reset pin
  );
  adc1->begin();
  adc2->begin();
  adc1->task_start(); // may be put at end begin, to be tested
  adc2->task_start();
}

void loop()
{
  motorDriver.update();

  linearAxis.update();

  //adc1->update(); check if data_ready

  #ifdef SIMPLE_COMMAND
  handleSimpleCommand();
  #else
  roverComm.handle();
  if(roverComm.messageAvailable())
  {
    respondToMsg(roverComm.popMessage());
  }
  #endif
}



