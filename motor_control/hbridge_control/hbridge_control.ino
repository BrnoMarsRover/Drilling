#include "PIController.h"
#include "FIR.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#define timeMeasure //mereni delky loopu/poctu loopu za sekundu
//#define printData

static inline float sgn(float val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

enum motorStateEnum
{
  Stopped,
  Running,
  Stopping,
};

//PINS
const uint8_t OutPin_L_EN = 3;
const uint8_t OutPin_R_EN = 4;
const uint8_t OutPin_L_PWM = 5;
const uint8_t OutPin_R_PWM = 6;

const uint8_t InPin_Thermometer = 7;

const uint8_t InPin_L_IS = A0;
const uint8_t InPin_R_IS = A1;
const uint8_t InPin_AmmeterRC = A2;
const uint8_t InPin_AmmeterDirect = A3;

//MOTOR CONTROL
float bridgeDC = 0.0;
float minimumBridgeDC = 0.0;
uint8_t avgCount = 10;
FIR bridgeDCAvg(avgCount);
float motorSpeed = 0.0; //[ot./s]
float sourceVoltage = 24.0;
float windingResistance = 0.2608;
//CONTROLLERS
//Kp  Ki    outputLimit   sumClamp  
PIController speedController(1.7, 2.5, 5, 2.0);
PIController currentController(40, 30, 150, 4.0);
//SETPOINT FILTER
enum motorStateEnum motorState = Stopped;
float requestedSpeed = 0;
float newRequestedSpeed = 0;
float speedSetpoint = 0;
const float minimumSpeedFromHalt = 0.5;
const float minimumSpeedRunning = 0.1;
const float controllerSetpointSR = 0.3;


//AMPMETER
uint16_t ammeterDirect = 0;
float ammeterRC = 0;
float ammeterRCZero = 500;
FIR ammeterRCAvg(avgCount);
//float ammeterVoltage = 0;
float ammeterCurrent = 0; // v amperech
const float Vcc = 4.66;
const float ammeterVoltsToAmps = 12.0;//10.0;
const double CPhi = 0.4834;

//I2C
const uint8_t I2CAddress = 10;
const uint8_t inputArraySize = 1;
uint8_t inputArray[inputArraySize] = {0};
const uint8_t outputArraySize = 4;
uint8_t outputArray[outputArraySize] = {0};
uint32_t lastMessageMillis = 0;
const uint32_t maxMillisWithoutMessage = 500; 

//TIME
uint32_t lastMillis = 0;
float timeDiff = 0;
#ifdef timeMeasure
uint32_t lastMillis_timeMeasure = 0;
uint32_t loops = 0;
#endif


//THERMOMETER
OneWire oneWireThermometer(InPin_Thermometer);
DallasTemperature dallasTemperatureThermometer(&oneWireThermometer);
DeviceAddress thermometerAddress;
float motorTemperature; // [°C]
float maxSafeTemperature = 70.0; // [°C]
bool overheat = false;
uint32_t thermometerLastMillis = 0;
uint32_t thermometerDelay = 2000;

//PROTECTION
uint16_t ISFaultThreshold = 800;
bool bridgeFault = false;


void updateMotorState()
{
  if(newRequestedSpeed != requestedSpeed)
  {
    if(motorState == motorStateEnum::Stopped)
    {
      motorState = motorStateEnum::Running;
      requestedSpeed = newRequestedSpeed;
      speedSetpoint = minimumSpeedFromHalt*sgn(requestedSpeed);
    }
    else if(newRequestedSpeed == 0 || sgn(newRequestedSpeed) != sgn(requestedSpeed))
    {
      requestedSpeed = 0;
      motorState = motorStateEnum::Stopping;
    }
    else if(sgn(newRequestedSpeed) == sgn(requestedSpeed))
    {
      requestedSpeed = newRequestedSpeed;
    }
  }
}

void setBridge(float aVal)
{
  aVal = constrain(aVal, -255.0, 255.0);

  if (aVal < 0)
  {
    analogWrite(OutPin_L_PWM, 0);
    analogWrite(OutPin_R_PWM, uint8_t(abs(aVal)));
  }
  else
  {
    analogWrite(OutPin_L_PWM, uint8_t(aVal));
    analogWrite(OutPin_R_PWM, 0);
  }
}

void calibrateAmmeter()
{
  setBridge(0);
  delay(100);

  const uint8_t avgCount = 16;
  FIR ammeterZeroAvg(avgCount);
  for(uint8_t i = 0; i < avgCount; i++)
  {
    delay(10);
    ammeterZeroAvg.updateOutput(float(analogRead(InPin_AmmeterRC)));
  }

  ammeterRCZero = ammeterZeroAvg.getOutput();
}

void setup()
{
  //PIN SETUP
  pinMode(OutPin_L_EN, OUTPUT);
  pinMode(OutPin_L_PWM, OUTPUT);
  pinMode(OutPin_R_EN, OUTPUT);
  pinMode(OutPin_R_PWM, OUTPUT);

  pinMode(InPin_AmmeterRC, INPUT);
  pinMode(InPin_AmmeterDirect, INPUT);  

  pinMode(InPin_L_IS, INPUT);
  pinMode(InPin_R_IS, INPUT);

  pinMode(13, INPUT);

  //MOTOR SETUP
  digitalWrite(OutPin_R_EN, HIGH);
  digitalWrite(OutPin_L_EN, HIGH);

  calibrateAmmeter();

  //THERMOMETER
  dallasTemperatureThermometer.begin();
  dallasTemperatureThermometer.setWaitForConversion(false);
  dallasTemperatureThermometer.getAddress(thermometerAddress, 0); 
  motorTemperature = 20;
  dallasTemperatureThermometer.requestTemperaturesByAddress(thermometerAddress);

  //I2C
  Wire.begin(I2CAddress);                
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.begin(9600);
}

void loop()
{
  uint32_t currentMillis = millis();

  /*
  if((currentMillis - lastMessageMillis) > maxMillisWithoutMessage)
  {
    requestedSpeed = 0.0;
  } 
  */
  uint16_t LISval = analogRead(InPin_L_IS);
  uint16_t RISval = analogRead(InPin_R_IS);

  if(LISval > ISFaultThreshold || RISval > ISFaultThreshold)
  {
    bridgeFault = true;
    requestedSpeed = 0.0;
  }
  else
  {
    bridgeFault = false;
  }

  timeDiff = 0.001*(float(currentMillis) - float(lastMillis));
  lastMillis = currentMillis;

  if(speedController.setpoint == 0.0 && requestedSpeed == 0.0)
  {
    speedController.reset();
    currentController.reset();
    bridgeDCAvg.updateOutput(0.0);
    ammeterRCAvg.updateOutput(0.0);
    ammeterCurrent = 0.0;
    bridgeDC = 0.0;
    motorSpeed = 0.0;

    motorState = motorStateEnum::Stopped;
    updateMotorState();
  }
  else
  {
    //Data acquisition
    ammeterDirect = analogRead(InPin_AmmeterDirect);
    /* //Safety
    if(ammeterDirect < overloadLow || ammeterDirect > overloadHigh || motorTemperature > 70)
    {
      analogWrite(OutPin_L_PWM, 0);
      analogWrite(OutPin_R_PWM, 0);
      overload = true;
      overloadEndTime = millis() + overloadDelay;
    }
    */
    ammeterRC = float(analogRead(InPin_AmmeterRC));
    //ammeterVoltage = (ammeterAvg)*(Vcc/1023.0);
    ammeterCurrent = ammeterRCAvg.updateOutput((ammeterRC - ammeterRCZero) * (Vcc/1023.0) * ammeterVoltsToAmps);
    motorSpeed = ((sourceVoltage*(bridgeDCAvg.updateOutput(bridgeDC)/255.0)) - windingResistance*ammeterCurrent)/(CPhi*2*3.14);

    //Setpoint filter
    float setpointMaxChange = constrain(timeDiff*controllerSetpointSR, 0.0, 0.1);
    speedSetpoint = constrain(requestedSpeed, speedSetpoint - setpointMaxChange, speedSetpoint + setpointMaxChange);
    if(motorState == motorStateEnum::Stopping)
    {
      if(abs(speedSetpoint) < minimumSpeedRunning)
      {
        speedSetpoint = 0;
      }
    }

    speedController.setpoint = speedSetpoint;

    //Controller action
    currentController.setpoint = speedController.compute(motorSpeed, timeDiff);
    bridgeDC = currentController.compute(ammeterCurrent, timeDiff);
  }
/*
  if(abs(bridgeDC) < minimumBridgeDC)
  {
    if(requestedSpeed == 0)
    {
      bridgeDC = 0;
    }
    else if(requestedSpeed > 0)
    {
      bridgeDC = minimumBridgeDC;
    }
    else
    {
      bridgeDC = -minimumBridgeDC;
    }
  }

  if(bridgeDC < 0)
  {
    digitalWrite(13, true);
  }
  else
  {
    digitalWrite(13, false);
  }
*/
  setBridge(bridgeDC);


  if(currentMillis - thermometerLastMillis > thermometerDelay)
  {
    motorTemperature = dallasTemperatureThermometer.getTempC(thermometerAddress);   
    dallasTemperatureThermometer.requestTemperaturesByAddress(thermometerAddress);
    thermometerLastMillis = millis();

    if (motorTemperature > maxSafeTemperature)
    {
      overheat = true;
      requestedSpeed = 0.0;
    }
    else
    {
      overheat = false;
    }
  }

  
#ifdef printData
  Serial.print(" t:");  
  Serial.print(timeDiff);

  Serial.print(" LIS: ");  
  Serial.print(LISval);

  Serial.print(" RIS: ");  
  Serial.print(RISval);

  Serial.print(" MSt:");
  Serial.print(motorState);

  Serial.print(" NRqSp:");
  Serial.print(newRequestedSpeed);

  Serial.print(" RqSp:");
  Serial.print(requestedSpeed);

  Serial.print(" SpSP:");
  Serial.print(speedSetpoint);

  Serial.print(" Sp:");
  Serial.print(motorSpeed);

  Serial.print(" SpErS:");
  Serial.print(speedController.getErrorSum());

  Serial.print(" ISP:");  
  Serial.print(currentController.setpoint);

  Serial.print(" I:");  
  Serial.print(ammeterCurrent);

  Serial.print(" DC:");
  Serial.print(bridgeDC);

  Serial.print(" tmp:");
  Serial.print(motorTemperature);

  Serial.println("");
#endif


#ifdef timeMeasure
  loops++;

  if(currentMillis - lastMillis_timeMeasure > 5000)
  {
    Serial.println(loops);   
    lastMillis_timeMeasure = currentMillis;
    loops = 0;
  }
#endif
  
}

void receiveEvent(int howMany)
{
  // ___TEST___
  /*
  Serial.print("prisel ");
  Serial.println(howMany);
  
  while(Wire.available() > 0) // loop through all
  {
    uint8_t c = Wire.read(); // receive byte 
    Serial.println(c);         // print the character
  }
  */
  
  if (howMany == inputArraySize)
  {
    if (!overheat && !bridgeFault)
    {
      Wire.readBytes(inputArray, inputArraySize);
      int8_t receivedInt = *((int8_t*)inputArray);
      newRequestedSpeed = constrain(float(receivedInt)*0.03, -3.0, 3.0);
      if(newRequestedSpeed != 0 && abs(newRequestedSpeed) < minimumSpeedFromHalt)
      {
        newRequestedSpeed = minimumSpeedFromHalt*sgn(newRequestedSpeed);
      }

      updateMotorState();

      lastMessageMillis = millis();
    }
  }
}

void requestEvent()
{
  uint8_t state = 0;

  //IS STUCK?
  if(speedController.setpoint != 0.0)
  {
    if (abs(motorSpeed) > 0.1)
      state |= B00000001;
    else
      state |= B00000010;
  }

  //BRIDGE FAULT
  if (bridgeFault)
    state |= B00000100;

  //SEND DATA
  outputArray[0] = state;

  int8_t speedInt = int8_t(constrain(motorSpeed/0.03, -127.0, 128.0));
  outputArray[1] = speedInt;

  int8_t torqueInt = int8_t(constrain((ammeterCurrent*CPhi)/0.03, -127.0, 128.0));
  outputArray[2] = torqueInt;

  uint8_t tempInt = uint8_t(constrain(motorTemperature, 0.0, 255.0));
  outputArray[3] = tempInt;

  Wire.write(outputArray, outputArraySize);
}