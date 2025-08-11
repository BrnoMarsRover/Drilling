#include <Wire.h> // I2C communication library
#include <Arduino.h>
#include <TMCStepper.h> // Library for TMC stepper driver
#include <SoftwareSerial.h> // Software UART
#include <FastAccelStepper.h> // Library for stepper control
//#include <TMC2209.h>

// Define pins
#define RX_PIN 11  // UART receive pin
#define TX_PIN 12  // UART transmit pin

#define DRIVER1_ADDRESS 0b00  // Driver 1 address 
#define DRIVER2_ADDRESS 0b01  // Driver 2 address 

#define RS 0.11f  // Sense resistor value for TMC2209 (Ohms)

#define TOF_PIN 4 //TOF sensor pin

#define ENCODER1_PIN A0  // Encoder 1 analog pin
#define ENCODER2_PIN A1  // Encoder 2 analog pin

#define KONC1_PIN 3 // Limit switch 1
#define KONC2_PIN 2 // Limit switch 2

// Motor driver 1 pins
#define DIR1_PIN 5
#define EN1_PIN 6
#define STEP1_PIN 9
// Motor driver 2 pins
#define DIR2_PIN 7
#define EN2_PIN 8
#define STEP2_PIN 10

#define TIMEOUT_MS 300  // Timeout for receiving I2C messages (ms)

unsigned long lastMessageTime = 0;

SoftwareSerial SerialDriver1(RX_PIN, TX_PIN); // Setup serial communication for the drivers

// TMC2209 driver instances
TMC2209Stepper driver1(&SerialDriver1, RS, DRIVER1_ADDRESS);
TMC2209Stepper driver2(&SerialDriver1, RS, DRIVER2_ADDRESS);

// Stepper engine and stepper motor instances
FastAccelStepperEngine engine;
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

// Encoder structure
struct Encoder {
    int pin;
    int angle;
    long rotations;
    int lastAngle;
    int angleOffset;
    long totalAngle;
};

// Encoders initialization
Encoder encoder1 = {ENCODER1_PIN, 0, 0, 0, 0, };
Encoder encoder2 = {ENCODER2_PIN, 0, 0, 0, 0, };

uint16_t position = 0;

// Limit switch state
bool limitSwitch1Pressed = false;
bool limitSwitch2Pressed = false;

bool homing = false;

// Motor settings
int microsteps = 16;
int current = 500; // RMS current setting
int stepsPerRevolution = 200 * microsteps;

// Loop timer
uint32_t lastUpdateTime = 0; 

// TOF measurement
bool measured = false;
uint16_t height;

// Error handling
int errorStatus = 0;
bool fixed = false;

// States
enum states {
  START_STATE = 0,
  UP_STATE = 3,
  DOWN_STATE = 2,
  WAIT_STATE = 1,
};
states currentState = START_STATE;

// Frequency settings
unsigned int frequency = 500;

// Check TMC2209 driver status
int checkDriverStatus(TMC2209Stepper &driver) {
    if (driver.otpw()) { // overheating
        return 3;
    }
    else if (driver.ot()) { // critical overheating
        return 3;
    }
    if (driver.s2ga() || driver.s2gb()) { // short circuit
        return 3;
    }
    if (driver.s2vsa() || driver.s2vsa()) { // overvoltage
        return 3;
    }
    return 0;
}

// Read distance from TOF sensor
float TOF_distance()
{
    unsigned long totalPulseWidth = 0;
    int numMeasurements = 10;

    for (int i = 0; i < numMeasurements; i++)
    {
        totalPulseWidth += pulseIn(TOF_PIN, HIGH);
        delayMicroseconds(100);
    }
    unsigned long averagePulseWidth = totalPulseWidth / numMeasurements;
    float distance = averagePulseWidth / 10.0 - 10;

    //Serial.println(distance);
    measured = true;
    return distance;
}

// Read and map analog value from AS5600 encoder to 0-360 degrees
int readAnalogAngle(int pin) {
    int value = analogRead(pin); 
    return map(value, 0, 1023, 0, 360);
}

// Update encoder rotations and total angle
void checkRotation(Encoder &encoder, int newAngle) {
    newAngle = (newAngle - encoder.angleOffset + 360) % 360; // Korekce offsetu
    int diff = newAngle - encoder.lastAngle;
    if (diff > 180) encoder.rotations--; // Překročení nuly zpět
    if (diff < -180) encoder.rotations++; // Překročení nuly dopředu
    
    encoder.lastAngle = newAngle;
    encoder.totalAngle = encoder.rotations * 360 + newAngle;
    /*
    Serial.print("offset: ");
    Serial.println(encoder.angleOffset);
    */
    Serial.print("uhel: ");
    Serial.println(encoder.totalAngle);
    

}

// Synchronize encoder zero position
void synchronizeEncoders(Encoder &encoder) {
    encoder.angleOffset = readAnalogAngle(encoder.pin); // Set the starting angle as the zero
    encoder.lastAngle = 0;
    encoder.rotations = 0;
    encoder.totalAngle = 0;
}

// Compare two encoders
int compareEncoders(const Encoder &enc1, const Encoder &enc2) {
    long difference = abs(enc1.totalAngle - enc2.totalAngle);
    Serial.print("rozdil enkoderu: ");
    Serial.println(difference);
    if (difference > 90) 
        return 9;
    else
        return 0;
}

// Compare angle from encoder vs motor steps
int compareEncoderAndSteps(long angleS, long angleE) {
  long difference = abs(angleE - angleS);
  Serial.print("rozdil stepu: ");
  Serial.println(difference);

  if (difference > 90) 
    return 8;
  else
    return 0;
}

// Convert motor steps to angle
long getAngleSteps(FastAccelStepper* stepper) {
    long currentPosition = stepper->getCurrentPosition();
    long angle = (long)(currentPosition * (360.0 / stepsPerRevolution));
    return angle;
}

// Attempt to synchronize two motors based on encoders
bool synchronizeMotors() {
  const int syncSpeed = 500;
  const int syncTolerance = 10;
  const int maxNoMoveCount = 3; // Number of times in a row the motor "doesn't move"
  const int minMovement = 3;    // Minimum movement in degrees to be considered a movement
  const int checkInterval = 50; // interval (ms)

  long lastAngle1 = encoder1.totalAngle;
  long lastAngle2 = encoder2.totalAngle;
  int noMoveCount1 = 0;
  int noMoveCount2 = 0;

  Serial.println("Zahájena synchronizace motorů");
  delay(1000);

  while (true) {
    int angle1 = readAnalogAngle(encoder1.pin);
    int angle2 = readAnalogAngle(encoder2.pin);
    checkRotation(encoder1, angle1);
    checkRotation(encoder2, angle2);

    long diff = encoder1.totalAngle - encoder2.totalAngle;

    stepper1->setSpeedInHz(syncSpeed);
    stepper2->setSpeedInHz(syncSpeed);

    Serial.print("Rozdíl: ");
    Serial.println(diff);

    // Check motor 1 movement
    if (abs(encoder1.totalAngle - lastAngle1) < minMovement) {
      noMoveCount1++;
    } else {
      noMoveCount1 = 0; // Reset counter, motor moves
    }
    lastAngle1 = encoder1.totalAngle;

    // Check motor 2 movement
    if (abs(encoder2.totalAngle - lastAngle2) < minMovement) {
      noMoveCount2++;
    } else {
      noMoveCount2 = 0;
    }
    lastAngle2 = encoder2.totalAngle;

    // If no movement detected, stop synchronization
    if (noMoveCount1 >= maxNoMoveCount || noMoveCount2 >= maxNoMoveCount) {
      Serial.println("Chyba: motor se dostatečně nehýbe.");
      stepper1->forceStop();
      stepper2->forceStop();
      return false;
    }

    // If motors synchronized
    if (abs(diff) <= syncTolerance) {
      stepper1->forceStop();
      stepper2->forceStop();
      return true;
    }

    // Set direction
    if (diff < 0) { 
      stepper1->runBackward();
      stepper2->runForward();
      
    } else {
      stepper1->runForward();
      stepper2->runBackward();
    }

    delay(100);
  }
}

// Reads analog values from both encoders and checks for discrepancies or errors
void readAndCheckEncoders() {
    int angle1 = readAnalogAngle(encoder1.pin);
    int angle2 = readAnalogAngle(encoder2.pin);
    checkRotation(encoder1, angle1);
    checkRotation(encoder2, angle2);

    // Check if encoder angles match stepper motor positions
    if (errorStatus == 0 && !limitSwitch1Pressed && !limitSwitch2Pressed) {
        errorStatus = compareEncoderAndSteps(getAngleSteps(stepper1), encoder1.totalAngle);
    }
    if (errorStatus == 0 && !limitSwitch1Pressed && !limitSwitch2Pressed) {
        errorStatus = compareEncoderAndSteps(getAngleSteps(stepper2), encoder2.totalAngle);
    }
    if (errorStatus == 0 && !limitSwitch1Pressed && !limitSwitch2Pressed) {
        errorStatus = compareEncoders(encoder1, encoder2);
    }
}

// Check drivers for errors
void checkDrivers() {
    if (errorStatus == 0) {
        errorStatus = checkDriverStatus(driver1);
    }
    if (errorStatus == 0) {
        errorStatus = checkDriverStatus(driver2);
    }
}

// Handles recovery when a synchronization error occurs between motors
void handleErrorRecovery() {
    if (errorStatus == 8 || errorStatus == 9) {
        stepper1->forceStop();
        stepper2->forceStop();

        if (synchronizeMotors()) {
            errorStatus = 0;
            stepper1->setCurrentPosition(encoder1.totalAngle * stepsPerRevolution / 360);             // Resets current stepper positions based on encoder angles
            stepper2->setCurrentPosition(encoder2.totalAngle * stepsPerRevolution / 360);
            currentState = 1;
            fixed = true;

            errorStatus = compareEncoderAndSteps(getAngleSteps(stepper1), encoder1.totalAngle);  // Recheck encoder vs stepper alignment
            errorStatus = compareEncoderAndSteps(getAngleSteps(stepper2), encoder2.totalAngle);
            
            delay(500);
        } else {
            //errorStatus = 6; // Synchronization failed
            errorStatus = 0; // jen pro testování
        }
    }
}

// Prints debug information to the Serial monitor
void printDebugInfo() {
    Serial.print("Limit1: ");
    Serial.print(limitSwitch1Pressed);
    Serial.print(" | Limit2: ");
    Serial.println(limitSwitch2Pressed);

    Serial.print("errorStatus: ");
    Serial.println(errorStatus);
    Serial.print("fix: ");
    Serial.println(fixed);
    Serial.print("position: ");
    Serial.println(position);

}

// Updates a calculated position value based on encoder1's angle
void updatePosition() {
    position = (uint16_t)(encoder1.totalAngle / 360 * 2);
    if(!homing)
      position = 60000;
}

// Handles behavior when in the "start" state, including homing logic
void handleStartState() {
  if (!measured) {
    height = TOF_distance();
  }

  fixed = false;

  //if ((limitSwitch1Pressed && limitSwitch2Pressed) || errorStatus > 1) { aby to při timeoutu mohlo vyjet nahoru
  if ((limitSwitch1Pressed && limitSwitch2Pressed) || errorStatus != 0) { // Both limit switches pressed or an error occurred
    synchronizeEncoders(encoder1);
    synchronizeEncoders(encoder2);
    stepper1->setCurrentPosition(0);
    stepper2->setCurrentPosition(0);
    currentState = WAIT_STATE;
    measured = false;
  } 
  else if (limitSwitch2Pressed) { // Only limit switch 2 pressed
    Serial.print("limitSwitch1Pressed ");
    synchronizeEncoders(encoder1);
    stepper1->forceStop();
    stepper1->setCurrentPosition(0);
    stepper2->setSpeedInHz(5000);
    stepper2->runBackward();
  } 
  else if (limitSwitch1Pressed) { // Only limit switch 1 pressed
    Serial.print("limitSwitch2Pressed ");
    synchronizeEncoders(encoder2);
    stepper2->forceStop();
    stepper2->setCurrentPosition(0);
    stepper1->setSpeedInHz(5000);
    stepper1->runBackward();
  } 
  else { // Neither limit switch pressed, move both motors backward
    stepper1->setSpeedInHz(5000);
    stepper2->setSpeedInHz(5000);
    stepper1->runBackward();
    stepper2->runBackward();
  }
}

// Handles behavior when in the "up" state
void handleUpState() {
  if (limitSwitch1Pressed || limitSwitch2Pressed || errorStatus != 0) {
    currentState = WAIT_STATE;
    synchronizeEncoders(encoder1);
    synchronizeEncoders(encoder2);
    measured = false;
    stepper1->setCurrentPosition(0);
    stepper2->setCurrentPosition(0);
    fixed = false;
  } 
  else {
    stepper1->setSpeedInHz(frequency);
    stepper2->setSpeedInHz(frequency);
    stepper1->runBackward();
    stepper2->runBackward();
  }
}

// Handles behavior when in the "down" state
void handleDownState() {
  //if (errorStatus != 0 || pozice > 490){//496 je hrot
  if (errorStatus != 0) { 
    currentState = WAIT_STATE;
  } 
  else {
    stepper1->setSpeedInHz(frequency);
    stepper2->setSpeedInHz(frequency);
    stepper1->runForward();
    stepper2->runForward();
  }
}

// Handles behavior when in the "wait" state
void handleWaitState() {
  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, HIGH);
  stepper1->forceStop();
  stepper2->forceStop();
  /*
  if (limitSwitch2Pressed || errorStatus == 8) { /////idk asi smazat
    errorStatus = 0;
  }
  */
}

// Arduino setup function
void setup() {
    // Pin configuration
    pinMode(ENCODER1_PIN, INPUT);
    pinMode(ENCODER2_PIN, INPUT);
    pinMode(EN1_PIN, OUTPUT);
    pinMode(DIR1_PIN, OUTPUT);
    pinMode(STEP1_PIN, OUTPUT);
    pinMode(EN2_PIN, OUTPUT);
    pinMode(DIR2_PIN, OUTPUT);
    pinMode(TOF_PIN, INPUT);
    //pinMode(STEP2_PIN, OUTPUT);
    pinMode(KONC1_PIN, INPUT_PULLUP);
    pinMode(KONC2_PIN, INPUT_PULLUP); 
  
    digitalWrite(EN1_PIN, LOW);  // Activate TMC2209 driver 1
    digitalWrite(EN2_PIN, LOW);  // Activate TMC2209 driver 2


    Serial.begin(115200);
    SerialDriver1.begin(9600);  // UART communication with TMC2209

    // Initialize motor drivers
    //initializeDrivers();
    driver1.begin();
    driver1.toff(5);  // Enable motor
    driver1.rms_current(current);  // Set current
    driver1.microsteps(microsteps); // Set microstepping
    driver1.intpol(false);       // Disable interpolation to 1/256 microsteps
    driver1.en_spreadCycle(false); // Use StealthChop for silent operation

    driver2.begin();
    driver2.toff(5);  // Enable motor
    driver2.rms_current(current);  // Set current
    driver2.microsteps(microsteps); // Set microstepping
    driver2.intpol(false);       // Disable interpolation to 1/256 microsteps
    driver2.en_spreadCycle(false); // Use StealthChop for silent operation
  
    // Initialize stepper engine
    engine.init();
    stepper1 = engine.stepperConnectToPin(STEP1_PIN);
    stepper2 = engine.stepperConnectToPin(STEP2_PIN);

    if (stepper1) {
        stepper1->setDirectionPin(DIR1_PIN);
        stepper1->setEnablePin(EN1_PIN);
        stepper1->setAutoEnable(true);
        stepper1->setSpeedInHz(frequency);
        stepper1->setAcceleration(2000);
    }
    if (stepper2) {
        stepper2->setDirectionPin(DIR2_PIN);
        stepper2->setEnablePin(EN2_PIN);
        stepper2->setAutoEnable(true);
        stepper2->setSpeedInHz(frequency);
        stepper2->setAcceleration(2000);
    }

    // Synchronize encoder positions
    synchronizeEncoders(encoder1);
    synchronizeEncoders(encoder2);
    //Serial.println("Encoders synchronized.");
    
    Wire.begin(9);
    Wire.onReceive(receiveEvent);  // Setup callback for receiving data
    Wire.onRequest(requestEvent);  // Setup callback for sending data
}

// Arduino loop function
void loop() {
    limitSwitch1Pressed = !digitalRead(KONC1_PIN);
    limitSwitch2Pressed = !digitalRead(KONC2_PIN);
    if(limitSwitch1Pressed || limitSwitch2Pressed)
      homing = true;
    
    if (millis() - lastMessageTime > TIMEOUT_MS) {
      //Serial.println("ERROR: No I2C message received!");
      //errorStatus = 1;
      //currentState = START_STATE;

    }
    if (millis() - lastUpdateTime >= 100)
    {   
    readAndCheckEncoders();
    handleErrorRecovery();
    //checkDrivers();
    printDebugInfo();
    updatePosition();
    lastUpdateTime = millis();
    }

 switch (currentState) {
  case START_STATE:
    handleStartState();
    break;

  case UP_STATE:
    handleUpState();
    break;

  case DOWN_STATE:
    handleDownState();
    break;

  case WAIT_STATE:
    handleWaitState();
    break;
}

}

// I2C receive event
void receiveEvent(int howMany) {
  if (Wire.available() >= 2) {
    int receivedCommand = Wire.read();  // First byte - command received
    
    uint16_t speed = Wire.read();  // Second byte - speed

    if (receivedCommand >= 0 && receivedCommand <= 4) {
      currentState = static_cast<states>(receivedCommand);  // Nastavení stavu
      if((limitSwitch1Pressed || limitSwitch2Pressed) && currentState == UP_STATE){
        currentState = WAIT_STATE;
      }
      if((limitSwitch1Pressed && limitSwitch2Pressed) && currentState == START_STATE){
        currentState = WAIT_STATE;
      }
      if(fixed){
        if(currentState == DOWN_STATE || currentState == UP_STATE){
        currentState = WAIT_STATE;
        Serial.print("fixxxxxxxx ");
        }
      }
      
      // Resetování erroru při úspěšné změně stavu
      if(errorStatus = 1)
        errorStatus = 0; 
      /*
      Serial.print("Přepnuto na stav: ");
      Serial.println(currentState);
      Serial.print(", Rychlost nastavena na: ");
      Serial.println(speed);
      /*
      Serial.print(", error status: ");
      Serial.println(errorStatus);
      */
      lastMessageTime = millis();
    } else {
      errorStatus = 2;
    }
    frequency = 500 + speed * 35;
  }
}

// I2C request event
void requestEvent() {
  byte stateAndError = combineStateAndError(currentState, errorStatus);
  Wire.write(stateAndError);
  Wire.write(lowByte(position));
  Wire.write(highByte(position));
  Wire.write(lowByte(height));
  Wire.write(highByte(height));  
}

// Combine state and error for I2C
byte combineStateAndError(byte state, byte errorCode) {
  byte result = 0;
  result = errorCode << 3 | state;
  return result;
}

void setupDriver(TMC2209Stepper& driver, const char* name, int toffValue, int rmsCurrent, int uSteps, bool intpolSetting, bool spreadCycleSetting) {
    const int maxRetries = 3;
    int retries = 0;
    bool configured = false;

    // Try to configure the driver
    while (!configured && retries < maxRetries) {
        driver.begin();
        driver.toff(toffValue);
        driver.rms_current(rmsCurrent);
        driver.microsteps(uSteps);
        driver.intpol(intpolSetting);
        driver.en_spreadCycle(spreadCycleSetting);

         // Verify if settings were applied correctly
        configured = 
            driver.toff() == toffValue &&
            driver.rms_current() == rmsCurrent &&
            driver.microsteps() == uSteps &&
            driver.intpol() == intpolSetting &&
            driver.en_spreadCycle() == spreadCycleSetting;

        if (!configured) {
            retries++;
            delay(100); // Small delay
        }
    }

    if (configured) {
        Serial.print(name);
        Serial.println(" initialized successfully.");
    } else {
        Serial.print(name);
        Serial.println(" failed to initialize.");
    }
}

// Function to initialize both stepper drivers
void initializeDrivers() {
    const int toffValue = 5;
    const int rmsCurrent = current;
    const int uSteps = microsteps;
    const bool intpolSetting = false;
    const bool spreadCycleSetting = false;

    setupDriver(driver1, "Driver 1", toffValue, rmsCurrent, uSteps, intpolSetting, spreadCycleSetting);
    setupDriver(driver2, "Driver 2", toffValue, rmsCurrent, uSteps, intpolSetting, spreadCycleSetting);
}

