#include "MovingAverage.h"
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
bool dir = false;
uint8_t bridgeDC = 0;
uint8_t lastbridgeDC = 0;
float bridgeDCMaxChange = 60;

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
const double currentToTorqueCoeff = 1;
double torque = 0.4833;

//PID   //Kp = 0.15; Ki = 0.0005
const double Kp=0.15, Ki=0.0005, Kd=0.00;
float currentTarget = 0;
uint32_t lastMillis = 0;
float errorSum = 0;
float PIDoutput = 0;


//I2C
const uint8_t I2CAddress = 10;
const uint8_t inputArraySize = 4;
uint8_t inputArray[inputArraySize] = {0};
const uint8_t outputArraySize = 5;
uint8_t outputArray[outputArraySize] = {0};



//TIME
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
uint32_t thermometerDelay = 5000;
#endif

void setDriver(bool adir, uint8_t abridgeDC)
{
  /* REDUNDANT??
  if (abridgeDC > 255)
    abridgeDC = 255;
  if (abridgeDC < 0)
    abridgeDC = 0;
  */

  if (adir)
  {
    analogWrite(OutPin_L_PWM, abridgeDC);
    analogWrite(OutPin_R_PWM, 0);
  }
  else
  {
    analogWrite(OutPin_L_PWM, 0);
    analogWrite(OutPin_R_PWM, abridgeDC);
  }
}

void calibrateAmmeter()
{
  setDriver(false, 0);
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
  if(currentTarget == 0)
  {
    bridgeDC = 0;
    errorSum = 0;
  }
  else
  {
    ammeterDirect = analogRead(InPin_AmmeterDirect);
    /*
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
    
    float currentDiff = currentTarget - ammeterCurrent;

    uint32_t currentMillis = millis();
    float timeDiff = 0.001*(float(currentMillis) - float(lastMillis));
    errorSum += currentDiff * timeDiff;
    lastMillis = currentMillis;
    PIDoutput = Kp * currentDiff + Ki * errorSum;

    if(PIDoutput > 0)
    {
      dir = true;
    }
    if(PIDoutput < 0)
    {
      dir = false;
    }

    bridgeDC = constrain(uint8_t(abs(PIDoutput)), lastbridgeDC - timeDiff*bridgeDCMaxChange, lastbridgeDC + timeDiff*bridgeDCMaxChange);
    bridgeDC = constrain(bridgeDC, 20, 150);
  }

  if(!overload)
  {
    //Serial.print("DC: ");
    //Serial.println(bridgeDC);
    //Serial.print("dir: ");
    //Serial.println(uint8_t(dir));
    //setDriver(dir, uint8_t(bridgeDC));

    setDriver(dir, bridgeDC);
    lastbridgeDC = bridgeDC;
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

  Serial.print(" I:");  
  Serial.println(ampmeterCurrent);
*/


/*
  Serial.print(" T:");
  Serial.print(torqueTarget);
  Serial.print(" O:");
  Serial.println(50.0*PIDoutput);
*/

  //Serial.print(" bridgeDC:");
  //Serial.print(bridgeDC);

  //Serial.println("");

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
    //Serial.println(motorTemperature);
    dallasTemperatureThermometer.requestTemperaturesByAddress(thermometerAddress);
    thermometerLastMillis = millis();
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
    Wire.readBytes(inputArray, inputArraySize);
    float receivedFloat = *((float*)inputArray);

    currentTarget = receivedFloat/currentToTorqueCoeff;


    /*
    Serial.print("Received direction: ");
    Serial.println(dir);
    Serial.print("Received torque: ");
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

  float torque = ammeterCurrent*currentToTorqueCoeff;
  memcpy(outputArray + 1, &torque, sizeof(torque));
  Wire.write(outputArray, outputArraySize);
}