#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "src/RoverComm/RoverComm.h"
#include "src/HardwareController/HardwareController.h"

//Define to command via ASCII messages through ArduinoIDE console.
//Comment out to utilize command via Python app.
//#define SIMPLE_COMMAND


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
HardwareController hardwareController(I2CBus, Serial0);


ADS122C04 *adc1 = nullptr; // declaration of ADC Deep sample class
ADS122C04 *adc2 = nullptr; // declaration of ADC Surface sample class

DrillState drillState = STATE_INITIALIZING;

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

void respondToMsg(const RoverMessage& msg)
{
  switch(msg.getCommandCode())
  {
    case CMD_RESTART:
    {
      roverComm.sendAck(CMD_RESTART);
      ESP.restart();
      break;
    }

    case CMD_STATE:
    {

      roverComm.sendState((int16_t)hardwareController.getCarriageHeightMM(), (int16_t)hardwareController.getSpiralRPM(), (uint8_t)hardwareController.getSpiralMotorTmp(), 0, drillState );
      break;
    }

    case CMD_DRILL_AUTO:
    {
      if(drillState == STATE_READY)
      {
        drillState = STATE_AUTO_DRILLING_DOWN;
        roverComm.sendAck(CMD_DRILL_AUTO);
      }
      else
      {
        roverComm.sendNack();
      }
      break;
    }

    case CMD_STOP_AUTO:
    {
      if(drillState == STATE_AUTO_DRILLING_DOWN || drillState == STATE_AUTO_CANT_REACH || drillState == STATE_AUTO_MOVING_UP || drillState == STATE_AUTO_STORING)
      {
        drillState = STATE_READY;
        roverComm.sendAck(CMD_STOP_AUTO);
      }
      else
      {
        roverComm.sendNack();
      }
      break;
    }

    case CMD_CALIBRATE_HEIGHT:
    {
      break;
    }

    case CMD_DRILL_SPEED:
    {
      if(hardwareController.setSpiralRPM(msg.getInt16Arg()))
        roverComm.sendAck(CMD_DRILL_SPEED);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_VERTICAL_SPEED:
    {
      if(hardwareController.setCarriageSpeedMMps(((float)msg.getInt8Arg())*0.1))
        roverComm.sendAck(CMD_VERTICAL_SPEED);
      else
        roverComm.sendNack();
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
  #ifdef SIMPLE_COMMAND
  Serial.begin(115200);
  #else
  Serial.begin(38400);
  #endif
  Serial.setTimeout(10);

  setupI2CBus();

  hardwareController.begin();



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

  drillState = STATE_READY;
}

void loop()
{
  hardwareController.update();

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



