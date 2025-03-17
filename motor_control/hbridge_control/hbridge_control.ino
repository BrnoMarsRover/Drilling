#include "PIController.h"
#include "FIR.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#define timeMeasure //mereni delky loopu/poctu loopu za sekundu


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
uint8_t avgCount = 10;
FIR bridgeDCAvg(avgCount);
float motorSpeed = 0.0; //[ot./s]
float sourceVoltage = 24.0;
float windingResistance = 0.2608;
//CONTROLLERS
PIController speedController(2.0, 2.0, 5, 2.0);
PIController currentController(60, 30, 150, 3.0);
//SETPOINT FILTER
float speedTarget = 0;
float controllerSetpointSR = 0.3;


//AMPMETER
uint16_t ammeterDirect = 0;
float ammeterRC = 0;
float ammeterRCZero = 500;
FIR ammeterRCAvg(avgCount);
//float ammeterVoltage = 0;
float ammeterCurrent = 0; // v amperech
const float Vcc = 4.5;
const float ammeterVoltsToAmps = 10.0;
const double CPhi = 0.4834;

//I2C
const uint8_t I2CAddress = 10;
const uint8_t inputArraySize = 4;
uint8_t inputArray[inputArraySize] = {0};
const uint8_t outputArraySize = 1 + 2*sizeof(float);
uint8_t outputArray[outputArraySize] = {0};
uint32_t lastMessageMillis = 0;
const uint32_t messageIntervalMillis = 100; 

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
uint16_t ISFaultThreshold = 500;
bool bridgeFault = false;


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

  if((currentMillis - lastMessageMillis) > messageIntervalMillis)
  {
    speedTarget = 0.0;
  } 

  if(analogRead(InPin_L_IS) > ISFaultThreshold || analogRead(InPin_R_IS) > ISFaultThreshold)
  {
    bridgeFault = true;
    speedTarget = 0.0;
  }
  else
  {
    bridgeFault = false;
  }

  timeDiff = 0.001*(float(currentMillis) - float(lastMillis));
  lastMillis = currentMillis;

  Serial.print(" t:");  
  Serial.print(timeDiff);

  if(speedController.setpoint == 0.0 && speedTarget == 0.0)
  {
    speedController.reset();
    currentController.reset();
    bridgeDCAvg.updateOutput(0.0);
    ammeterRCAvg.updateOutput(0.0);
    ammeterCurrent = 0.0;
    bridgeDC = 0.0;
    motorSpeed = 0.0;
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
    float controllerSetpointMaxChange = constrain(timeDiff*controllerSetpointSR, 0.0, 0.2);
    speedController.setpoint = constrain(speedTarget, speedController.setpoint - controllerSetpointMaxChange, speedController.setpoint + controllerSetpointMaxChange);

    //Controller action
    currentController.setpoint = speedController.compute(motorSpeed, timeDiff);
    bridgeDC = currentController.compute(ammeterCurrent, timeDiff);
  }

  setBridge(bridgeDC);

  
  /*
  Serial.print(" M:");
  Serial.print(6000); // To freeze the lower limit
  Serial.print(" m:");
  Serial.print(-6000); // To freeze the upper limit
*/
  Serial.print(" SpT:");
  Serial.print(speedController.setpoint);

  Serial.print(" Sp:");
  Serial.print(motorSpeed);

  Serial.print(" SpEs:");
  Serial.print(speedController.getErrorSum());

  Serial.print(" IT:");  
  Serial.print(currentController.setpoint);

  Serial.print(" I:");  
  Serial.print(ammeterCurrent);

  Serial.print(" DC:");
  Serial.print(bridgeDC);


#ifdef timeMeasure
  loops++;

  if(currentMillis - lastMillis_timeMeasure > 10000)
  {
    Serial.println(loops);   
    lastMillis_timeMeasure = currentMillis;
    loops = 0;
  }
#endif
  

  if(currentMillis - thermometerLastMillis > thermometerDelay)
  {
    motorTemperature = dallasTemperatureThermometer.getTempC(thermometerAddress);   
    dallasTemperatureThermometer.requestTemperaturesByAddress(thermometerAddress);
    thermometerLastMillis = millis();

    if (motorTemperature > maxSafeTemperature)
    {
      overheat = true;
      speedTarget = 0.0;
    }
    else
    {
      overheat = false;
    }

    Serial.print(" tmp:");
    Serial.print(motorTemperature);
  }


  Serial.println("");
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
      float receivedFloat = *((float*)inputArray);
      speedTarget = constrain(receivedFloat, -3.0, 3.0);

      lastMessageMillis = millis();

      /*
      Serial.print("Received float: ");
      Serial.println(receivedFloat);
      */
    }
  }
}

void requestEvent()
{
  uint8_t state = 0;

  //IS STUCK?
  if(speedController.setpoint != 0.0)
  {
    if (motorSpeed > 0.1)
      state |= B00000001;
    else
      state |= B00000010;
  }

  //TEMPERATURE
  if (motorTemperature < 30.0)
    state |= B00000000;
  else if (motorTemperature < 50.0)
    state |= B00000100;
  else if (motorTemperature < 60.0)
    state |= B00001000;
  else
    state |= B00001100;

  //BRIDGE FAULT
  if (bridgeFault)
    state |= B00010000;

  //SEND DATA
  outputArray[0] = state;

  float torque = ammeterCurrent*CPhi;
  memcpy(outputArray + 1, &motorSpeed, sizeof(motorSpeed));
  memcpy(outputArray + 5, &torque, sizeof(torque));

  Wire.write(outputArray, outputArraySize);
}