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
const uint8_t InPin_AmpmeterFiltered = A2;
const uint8_t InPin_AmpmeterDirect = A3;

//MOTOR CONTROL
bool dir = false;
float motorDC = 0;

//OVERLOAD PROTECTION
bool overload = false;
uint16_t overloadHigh = 750;
uint16_t overloadLow = 273;
uint16_t overloadDelay = 1000; //ms
uint32_t overloadEndTime = 0; //ms


//AMPMETER
uint16_t ampmeterFiltered = 0;
uint16_t ampmeterDirect = 0;
const uint16_t ampmeterFilteredScale = 32;
MovingAverage <uint16_t, 16> ampmeterFilter;
uint16_t ampmeterAvg = 0;
uint16_t ampmeterFilteredZero = 500;
//float ampmeterVoltage = 0;
float ampmeterCurrent = 0; // v amperech
const float Vcc = 4.5;
const float ampmeterVoltsToAmps = 10.0;
const double currentToTorqueCoeff = 1;
double torque = 0;

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

void setDriver(bool adir, uint8_t amotorDC)
{
  /* REDUNDANT??
  if (amotorDC > 255)
    amotorDC = 255;
  if (amotorDC < 0)
    amotorDC = 0;
  */

  if (adir)
  {
    analogWrite(OutPin_L_PWM, amotorDC);
    analogWrite(OutPin_R_PWM, 0);
  }
  else
  {
    analogWrite(OutPin_L_PWM, 0);
    analogWrite(OutPin_R_PWM, amotorDC);
  }
}

void calibrateAmpmeter()
{
  setDriver(false, 0);
  delay(100);

  const uint8_t avgCount = 16;
  MovingAverage <uint16_t, avgCount> ampmeterZeroFilter;
  for(uint8_t i = 0; i < avgCount; i++)
  {
    delay(10);
    ampmeterZeroFilter.add(ampmeterFilteredScale*analogRead(InPin_AmpmeterFiltered));
  }

  ampmeterFilteredZero = ampmeterZeroFilter.get();
}

void setup()
{
  pinMode(OutPin_L_EN, OUTPUT);
  pinMode(OutPin_L_PWM, OUTPUT);
  pinMode(OutPin_R_EN, OUTPUT);
  pinMode(OutPin_R_PWM, OUTPUT);

  pinMode(InPin_AmpmeterFiltered, INPUT);
  pinMode(InPin_AmpmeterDirect, INPUT);  

  digitalWrite(OutPin_R_EN, HIGH);
  digitalWrite(OutPin_L_EN, HIGH);

  Serial.begin(9600);

  calibrateAmpmeter();

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

  ampmeterDirect = analogRead(InPin_AmpmeterDirect);
  /*
  if(ampmeterDirect < overloadLow || ampmeterDirect > overloadHigh)
  {
    analogWrite(OutPin_L_PWM, 0);
    analogWrite(OutPin_R_PWM, 0);
    overload = true;
    overloadEndTime = millis() + overloadDelay;
  }
  */
  ampmeterFiltered = ampmeterFilteredScale*analogRead(InPin_AmpmeterFiltered);
  ampmeterAvg = ampmeterFilter.add(ampmeterFiltered);
  //ampmeterVoltage = (ampmeterAvg)*(Vcc/1023.0);
  ampmeterCurrent = abs((float(ampmeterAvg) - float(ampmeterFilteredZero)) * (1.0/float(ampmeterFilteredScale)) * (Vcc/1023.0) * ampmeterVoltsToAmps);
  
  float currentDiff = currentTarget - ampmeterCurrent;

  uint32_t currentMillis = millis();
  float timeDiff = 0.001*(float(currentMillis) - float(lastMillis));

  errorSum += currentDiff * timeDiff;

  lastMillis = currentMillis;

  PIDoutput = Kp * currentDiff + Ki * errorSum;

  if(currentTarget == 0)
  {
    motorDC = 0;
  }
  else
  {
    motorDC = uint8_t(PIDoutput);
  }



  if(!overload)
  {
    //Serial.print("DC: ");
    Serial.println(motorDC);
    //Serial.print("dir: ");
    //Serial.println(uint8_t(dir));
    //setDriver(dir, uint8_t(motorDC));

    if(motorDC < 0)
    {
      setDriver(false, uint8_t(-1*motorDC));
    }
    else
    {
      setDriver(true, uint8_t(motorDC));
    }
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

  //Serial.print(" motorDC:");
  //Serial.print(motorDC);

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

    /*
    Serial.print("Received direction: ");
    Serial.println(dir);
    Serial.print("Received torque: ");
    Serial.println(receivedFloat);
    */
    currentTarget = receivedFloat/currentToTorqueCoeff;
  }
}

void requestEvent()
{
  if(motorDC == 0)
  {
    outputArray[0] = 0;
  }
  else
  {
    outputArray[0] = 1;
  }

  float torque = ampmeterCurrent*currentToTorqueCoeff;
  memcpy(outputArray + 1, &torque, sizeof(torque));
  Wire.write(outputArray, outputArraySize);
}