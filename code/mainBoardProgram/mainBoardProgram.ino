#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "src/RoverComm/RoverComm.h"
#include "src/DeepSampler/DeepSampler.h"
#include "src/SurfaceSampleHolder/SurfaceSampleHolder.h"


//-------I2C
constexpr uint8_t sdaPin = 21;
constexpr uint8_t sclPin = 22;
constexpr uint32_t i2cFrequency = 100000;
TwoWire I2CBus(0);

HardwareSerial& roverSerial = Serial0;

void setupI2CBus()
{
  // Print starting message for I2C recovery and setup
  roverSerial.println(F("Starting I2C Bus Recovery and Setup..."));

  // Check if I2C bus is free
  if (isI2CBusFree())
  {
    roverSerial.println(F("I2C bus is already free."));
  } else
  {
    // Bus is stuck, attempt recovery
    roverSerial.println(F("I2C bus is stuck, attempting recovery..."));
    if (i2cRecoverBus())
    {
      roverSerial.println(F("I2C bus recovery successful."));
    } else
    {
      roverSerial.println(F("I2C bus recovery failed!"));
    }
  }

  // Initialize the I2C bus
  I2CBus.begin(sdaPin, sclPin, i2cFrequency);
  I2CBus.setTimeOut(50);

  roverSerial.println(F("I2C Bus Setup Complete."));
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

  roverSerial.println(F("Attempting I2C bus recovery..."));

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
      roverSerial.println(F("SDA line released. Bus is free."));
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
  roverSerial.println(success ? F("I2C recovery successful.") : F("I2C recovery failed."));

  return success;
}

DrillState drillState = STATE_INITIALIZING;

//---------PERIPHERAL CLASSES
DeepSampler deepSampler(I2CBus, roverSerial);
SurfaceSampleHolder surfaceSampleHolder(I2CBus, roverSerial);

//------UART COMMUNICATION
//Define ADVANCE_COMMAND to utilize command via Python app.
//Comment out to command via ASCII messages through ArduinoIDE console.
#define ADVANCED_COMMAND

#ifdef ADVANCED_COMMAND
RoverComm roverComm(roverSerial);

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
      roverComm.sendState(
        deepSampler.getCarriageDepthMM(),
        deepSampler.getCarriageSpeedMMps(),
        0.0,  //vertical drive stepper current
        deepSampler.getSpiralRPM(),
        deepSampler.getSpiralMotorTmp(),
        deepSampler.storageGetCurrentAngle(),  //deep sample storage angle
        drillState );
      break;
    }

    case CMD_DRILL_AUTO:
    {
      if(drillState == STATE_READY)
      {
        if(deepSampler.autoDrillToDepth(2, 60, 10*((float)msg.getUint8Arg()) ) )
          roverComm.sendAck(CMD_DRILL_AUTO);
        else
          roverComm.sendNack();
      }
      else
      {
        roverComm.sendNack();
      }
      break;
    }

    case CMD_STOP_AUTO:
    {
      if(deepSampler.drillSetManualControl())
      {
        roverComm.sendAck(CMD_STOP_AUTO);
      }
      else
      {
        roverComm.sendNack();
      }
      break;
    }

    case CMD_CALIBRATE_CARRIAGE_DEPTH:
    {
      if(deepSampler.setCarriageSpeedMMps(-10.0))
        roverComm.sendAck(CMD_CALIBRATE_CARRIAGE_DEPTH);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_START_DEVICE_CHECK:
    {
      roverComm.sendAck(CMD_START_DEVICE_CHECK);
      break;
    }

    case CMD_GET_DEVICE_STATUS:
    {
      roverComm.sendDeviceStatus(
        deepSampler.verticalStepperConnected(), //vertStepper
        deepSampler.verticalEncoderConnected(), //vertEncoder
        false, //vertCurrentSensor
        deepSampler.spiralMotorConnected(), //spiralMotor
        deepSampler.heightSensorConnected(), //heightSensor
        false, //deepSampleStepper
        false, //deepSampleEncoder
        deepSampler.getAdcConnected(), //deepSampleADC
        surfaceSampleHolder.getAdcConnected() //surfaceSampleADC
      );
      break;
    }

    case CMD_DRILL_SPEED:
    {
      if(deepSampler.setSpiralRPM(msg.getInt16Arg()))
        roverComm.sendAck(CMD_DRILL_SPEED);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_VERTICAL_SPEED:
    {
      if(deepSampler.setCarriageSpeedMMps(((float)msg.getInt8Arg())*0.1))
        roverComm.sendAck(CMD_VERTICAL_SPEED);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_STORAGE_POSITION:
    {
      if(deepSampler.storageMoveToSlot(msg.getUint8Arg()))
        roverComm.sendAck(CMD_STORAGE_POSITION);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_MEASURE_HEIGHT_ABOVE_GROUND:
    {
      if(deepSampler.startDistFromSurfaceMeasure())
        roverComm.sendAck(CMD_MEASURE_HEIGHT_ABOVE_GROUND);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_GET_HEIGHT_ABOVE_GROUND:
    {
      roverComm.sendUint16(CMD_GET_HEIGHT_ABOVE_GROUND, deepSampler.getDistFromSurfaceMM());
      break;
    }
    
    case CMD_WEIGH_DEEP:
    {
      if(deepSampler.requestMeasure())
        roverComm.sendAck(CMD_WEIGH_DEEP);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_WEIGH_SURFACE:
    {
      if(surfaceSampleHolder.requestMeasure())
        roverComm.sendAck(CMD_WEIGH_SURFACE);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_GET_WEIGHT_DEEP:
    {
      if(deepSampler.getResultReady()){
        roverComm.sendWeight(CMD_GET_WEIGHT_DEEP, deepSampler.getLastWeight());
      }
      else
        roverComm.sendNack();
      break;
    }

    case CMD_GET_WEIGHT_SURFACE:
      if(surfaceSampleHolder.getResultReady()){
        roverComm.sendWeight(CMD_GET_WEIGHT_SURFACE, surfaceSampleHolder.getLastWeight());
      }
      else
        roverComm.sendNack();
      break;

    case CMD_CALIBRATE_0_DEEP:
    {
      if(deepSampler.setCalibration0())
        roverComm.sendAck(CMD_CALIBRATE_0_DEEP);
      else
        roverComm.sendNack();
      break;
    }

    case CMD_CALIBRATE_X_DEEP:
    {
      if(deepSampler.setCalibrationX((float)msg.getInt16Arg()))
        roverComm.sendAck(CMD_CALIBRATE_X_DEEP);
      else
        roverComm.sendNack();
      break;
    }
    
    case CMD_CALIBRATE_0_SURFACE:
    {
      if(surfaceSampleHolder.setCalibration0())
        roverComm.sendAck(CMD_CALIBRATE_0_SURFACE);
      else
        roverComm.sendNack();
      break;
    }
    
    case CMD_CALIBRATE_X_SURFACE:
    {
      if(surfaceSampleHolder.setCalibrationX((float)msg.getInt16Arg()))
        roverComm.sendAck(CMD_CALIBRATE_X_SURFACE);
      else
        roverComm.sendNack();
      break;
    }
    
    case CMD_ROCK_OPEN:
    {
      break;
    }

    case CMD_ROCK_CLOSE:
    {
      break;
    }

    case CMD_SAND_OPEN:
    {
      break;
    }
    
    case CMD_SAND_CLOSE:
    {
      break;
    }
  }
}

#else

void handleSimpleCommand()
{
  if (roverSerial.available())
  {
    String cmd = roverSerial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.length() == 0) return;
    /*
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
        roverSerial.println("Chyba: pouzij napr. R100");
      }
    }
    */
    else if (cmd.startsWith("RM")) { // mm/s
      float value = cmd.substring(2).toFloat();
      if (value > 0) {
        deepSampler.setCarriageSpeedMMps(value);
        roverSerial.print("Rychlost mm/s: ");
        //roverSerial.println(linearAxis.getSpeedMMps());
        }
    }
    /*
    else if (cmd == "X") {
      linearAxis.zero();
    }
    else if (cmd == "?") {
      linearAxis.printStatus(roverSerial);
    }
    else if (cmd == "TOF") {
      roverSerial.print("Vzdálenost: ");
      roverSerial.println(distanceSensor.readSingle());
    }
    else if (cmd == "SG") {
      linearAxis.setLoadPrintEnabled(true);
    }
    else if (cmd == "NSG") {
      linearAxis.setLoadPrintEnabled(false);
    }
    else if (cmd == "H") {
      linearAxis.setDepthPrintEnabled(true);
    }
    else if (cmd == "NH") {
      linearAxis.setDepthPrintEnabled(false);
    }
    */
    else if (cmd.startsWith("M")) {   //CUBEMARS MOTOR COMMANDS
      int value = cmd.substring(1).toFloat();
      deepSampler.setSpiralRPM(value);
    }
    /*
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
        roverSerial.print(w, 4);
        roverSerial.println("g");
        if (w > 2000.0f) roverSerial.println("[ADC] Scale overweight");
      }
      else {
        roverSerial.println("[ADC] No weight result ready");
      }
    }
    else if (cmd.startsWith("TRE")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      roverSerial.println("[ADC] Tare start");
      target->set_tare();
    }
    else if (cmd.startsWith("CLB0")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      roverSerial.println("[ADC] Calibration start 0g");
      target->set_calibration_0();
    }
    else if (cmd.startsWith("CLB100")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      roverSerial.println("[ADC] Calibration start 100g");
      target->set_calibration_100();
    }
    else if (cmd.startsWith("ADCTMP")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      roverSerial.println("[ADC] tmp measured");
      target->request_tmp();
    }
    else if (cmd.startsWith("GT")) {
      ADS122C04 *target = cmd.endsWith("D") ? adc1 : cmd.endsWith("S") ? adc2 : nullptr;
      if (target == nullptr) { return; }
      roverSerial.print("[ADC] Chip tmp: ");
      roverSerial.print(target->get_last_temp(), 4);
      roverSerial.println(" °C");
    }
    else if (cmd.startsWith("ADCRST")) {
      //ADS122C04 *target = adc1;
      //if (target == nullptr) { roverSerial.println("ADC deep sample not assigned"); }
      adc1->reset();
      //ADS122C04 *target = adc2;
      //if (target == nullptr) { roverSerial.println("ADC surface sample not assigned"); }
      adc2->reset();
      roverSerial.println("[ADC] resets complete");
    }
    */
    else {
      roverSerial.println("Neznamy prikaz.");
      roverSerial.println("Pouzij: U, D, S, R100, +, -, A2000, X, ?, WGH+D/S, TRE+D/S, CLB+0/100+D/S, ADCTMP+D/S, ADCRST, GW+D/S, GT+D/S");
    }
  }
}
#endif

// 3,3 V MOSFET switch - RESET pin
constexpr uint8_t resetPin = 2;
void startPeripheralReset()
{
  digitalWrite(resetPin, HIGH);
}
void endPeripheralReset()
{
  digitalWrite(resetPin, LOW);
}
void activeDelayReset()
{
  startPeripheralReset();
  delay(100);
  endPeripheralReset();
  delay(100);
}


void setup()
{
  pinMode(resetPin, OUTPUT);
  activeDelayReset();
  #ifdef ADVANCED_COMMAND
  roverSerial.begin(38400);
  #else
  roverSerial.begin(115200);
  roverSerial.setTimeout(10);
  #endif

  setupI2CBus();

  deepSampler.begin();
  surfaceSampleHolder.begin();

  drillState = STATE_READY;
}

void loop()
{
  deepSampler.update();

  #ifdef ADVANCED_COMMAND
  roverComm.handle();
  if(roverComm.messageAvailable())
  {
    respondToMsg(roverComm.popMessage());
  }
  #else
  handleSimpleCommand();
  #endif
}



