#include "MovingAverage.h"
#include "PIController.h"
#include "FIR.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>



//#define timeMeasure //mereni delky loopu/poctu loopu za sekundu
#define thermometer

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
float motorSpeed = 0.0; //[ot./s]
FIR motorSpeedFilter(10);
float sourceVoltage = 24.0;
float windingResistance = 0.2608;
//CONTROLLERS
PIController speedController(1.5, 3, 5, 2.0);
PIController currentController(60, 30, 150, 3.0);
//SETPOINT FILTER
float speedTarget = 0;
float controllerSetpointSR = 0.3;

//OVERLOAD PROTECTION
bool overload = false;
uint16_t overloadHigh = 750;
uint16_t overloadLow = 273;
uint16_t overloadDelay = 1000; //ms
uint32_t overloadEndTime = 0; //ms

//AMPMETER
uint16_t ammeterRC = 0;
uint16_t ammeterDirect = 0;
const uint16_t ammeterDigitalFilterScale = 8;
MovingAverage <uint16_t, 16> ammeterDigitalFilter;
uint16_t ammeterRCAvg = 0;
uint16_t ammeterRCZero = 500;
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

//TIME
uint32_t lastMillis = 0;
float timeDiff = 0;
#ifdef timeMeasure
uint32_t lastMillis_timeMeasure = 0;
uint32_t loops = 0;
#endif

#ifdef thermometer
//THERMOMETER
OneWire oneWireThermometer(InPin_Thermometer);
DallasTemperature dallasTemperatureThermometer(&oneWireThermometer);
DeviceAddress thermometerAddress;
float motorTemperature;
uint32_t thermometerLastMillis = 0;
uint32_t thermometerDelay = 2000;
#endif

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
  MovingAverage <uint16_t, avgCount> ammeterZeroFilter;
  for(uint8_t i = 0; i < avgCount; i++)
  {
    delay(10);
    ammeterZeroFilter.add(ammeterDigitalFilterScale*analogRead(InPin_AmmeterRC));
  }

  ammeterRCZero = ammeterZeroFilter.get();
}

void setup()
{
  pinMode(OutPin_L_EN, OUTPUT);
  pinMode(OutPin_L_PWM, OUTPUT);
  pinMode(OutPin_R_EN, OUTPUT);
  pinMode(OutPin_R_PWM, OUTPUT);

  pinMode(InPin_AmmeterRC, INPUT);
  pinMode(InPin_AmmeterDirect, INPUT);  

  digitalWrite(OutPin_R_EN, HIGH);
  digitalWrite(OutPin_L_EN, HIGH);

  Serial.begin(9600);

  calibrateAmmeter();

#ifdef thermometer
  dallasTemperatureThermometer.begin();
  dallasTemperatureThermometer.setWaitForConversion(false);
  dallasTemperatureThermometer.getAddress(thermometerAddress, 0); 
  motorTemperature = 20;
  dallasTemperatureThermometer.requestTemperaturesByAddress(thermometerAddress);
#endif

  Wire.begin(I2CAddress);                
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop()
{
  uint32_t currentMillis = millis();
  timeDiff = 0.001*(float(currentMillis) - float(lastMillis));
  lastMillis = currentMillis;

  Serial.print(" t:");  
  Serial.print(timeDiff);

  if(speedController.setpoint == 0.0 && speedTarget == 0.0)
  {
    speedController.reset();
    currentController.reset();
    motorSpeedFilter.updateOutput(0.0);
    ammeterDigitalFilter.reset();
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
    ammeterRC = ammeterDigitalFilterScale*analogRead(InPin_AmmeterRC);
    ammeterRCAvg = ammeterDigitalFilter.add(ammeterRC);
    //ammeterVoltage = (ammeterAvg)*(Vcc/1023.0);
    ammeterCurrent = (float(ammeterRCAvg) - float(ammeterRCZero)) * (1.0/float(ammeterDigitalFilterScale)) * (Vcc/1023.0) * ammeterVoltsToAmps;
    float calculatedMotorSpeed = ((sourceVoltage*(bridgeDC/255)) - windingResistance*ammeterCurrent)/(CPhi*2*3.14);
    motorSpeed = motorSpeedFilter.updateOutput(calculatedMotorSpeed);

    //Setpoint filter
    float controllerSetpointMaxChange = constrain(timeDiff*controllerSetpointSR, 0.0, 0.2);
    speedController.setpoint = constrain(speedTarget, speedController.setpoint - controllerSetpointMaxChange, speedController.setpoint + controllerSetpointMaxChange);

    //Controller action
    currentController.setpoint = speedController.compute(motorSpeed, timeDiff);
    bridgeDC = currentController.compute(ammeterCurrent, timeDiff);
  }


  if(!overload)
  {
    setBridge(bridgeDC);
  } 
  else
  {
    if(millis() > overloadEndTime)
    {
      overload = false;
    }
  }
  
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
  long currentMillis = millis();
  loops++;  

  if(currentMillis - lastMillis_timeMeasure > 10000)
  {
    Serial.println(loops);   
    lastMillis_timeMeasure = currentMillis;
    loops = 0;
  }
#endif
  

#ifdef thermometer
  if(millis() - thermometerLastMillis > thermometerDelay)
  {
    motorTemperature = dallasTemperatureThermometer.getTempC(thermometerAddress);   
    dallasTemperatureThermometer.requestTemperaturesByAddress(thermometerAddress);
    thermometerLastMillis = millis();

    Serial.print(" tmp:");
    Serial.print(motorTemperature);
  }
#endif

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
    Wire.readBytes(inputArray, inputArraySize);
    float receivedFloat = *((float*)inputArray);
    speedTarget = receivedFloat;

    /*
    Serial.print("Received float: ");
    Serial.println(receivedFloat);
    */
  }
}

void requestEvent()
{
  if(bridgeDC == 0)
  {
    outputArray[0] = 0;
  }
  else
  {
    outputArray[0] = 1;
  }

  float torque = ammeterCurrent*CPhi;
  memcpy(outputArray + 1, &motorSpeed, sizeof(motorSpeed));
  memcpy(outputArray + 5, &torque, sizeof(torque));

  Wire.write(outputArray, outputArraySize);
}