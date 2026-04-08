#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "as5600.h"
#include "LimitSwitch.h"

#define STEP_PIN 13
#define DIR_PIN  12
#define EN_PIN   14

#define CS_PIN   5
#define SCK_PIN  18
#define MISO_PIN 19
#define MOSI_PIN 23

#define R_SENSE 0.075f

#define LIMIT_TOP_PIN 15
#define LIMIT_BOTTOM_PIN 0

#define SDA_PIN 21
#define SCL_PIN 22

#define AS5600_ADDR 0x40
#define MM_PER_REV  2.0f      // kolik mm ujede mechanismus za 1 otáčku

TMC5160Stepper driver = TMC5160Stepper(CS_PIN, R_SENSE);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

AS5600L encoder(AS5600_ADDR);

LimitSwitch limitTop(LIMIT_TOP_PIN);
LimitSwitch limitBottom(LIMIT_BOTTOM_PIN);

// stav řízení
uint32_t speedHz = 500;      // rychlost v krocích za sekundu, limit rychlosti asi kolem 14000 v 15K silně vibruje tyč
uint32_t accelHz = 2000;     // zrychlení v krocích za sekundu^2
int motionState = 0;         // 1 = nahoru, -1 = dolu, 0 = stop

void testSPI() {
  Serial.println("=== TEST SPI ===");
  Serial.print("IOIN: 0x");
  Serial.println(driver.IOIN(), HEX);
  Serial.print("GSTAT: 0x");
  Serial.println(driver.GSTAT(), HEX);
  Serial.print("CHOPCONF: 0x");
  Serial.println(driver.CHOPCONF(), HEX);
}

void printStatus() {
  Serial.println("=== STAV ===");
  Serial.print("Rychlost [Hz]: ");
  Serial.println(speedHz);

  Serial.print("Akcelerace [steps/s^2]: ");
  Serial.println(accelHz);

  Serial.print("Smer: ");
  if (motionState > 0) {
    Serial.println("NAHORU");
  } else if (motionState < 0) {
    Serial.println("DOLU");
  } else {
    Serial.println("STOP");
  }

  if (stepper != nullptr) {
    Serial.print("Pozice: ");
    Serial.println(stepper->getCurrentPosition());
  }
}

void applyMotion() {
  if (stepper == nullptr) return;

  stepper->setSpeedInHz(speedHz);
  stepper->setAcceleration(accelHz);

  if (motionState > 0) {
    stepper->runForward();
  } else if (motionState < 0) {
    stepper->runBackward();
  } else {
    stepper->stopMove();
  }
}

void setDirectionState(int dir) {
  motionState = dir;

  if (motionState > 0) {
    Serial.println("Smer: NAHORU");
  } else if (motionState < 0) {
    Serial.println("Smer: DOLU");
  } else {
    Serial.println("Motor STOP");
  }

  applyMotion();
}

void stopMotor() {
  motionState = 0;

  if (stepper != nullptr) {
    stepper->forceStop();
  }

  Serial.println("Motor zastaven");
}

void stopAndZeroPosition() {
  setDirectionState(0);

  if (stepper != nullptr) {
  stepper->forceStop();
  stepper->setCurrentPosition(0);
  }
  
  encoder.setZero();

  Serial.println("Koncak sepnut -> motor zastaven, pozice nulovana");
}

void changeSpeedRelative(int delta) {
  int32_t newSpeed = (int32_t)speedHz + delta;
  if (newSpeed < 1) newSpeed = 1;
  speedHz = (uint32_t)newSpeed;

  if (stepper != nullptr) {
    stepper->setSpeedInHz(speedHz);
    stepper->applySpeedAcceleration();
  }

  Serial.print("Nova rychlost: ");
  Serial.print(speedHz);
  Serial.println(" Hz");
}

void setSpeedAbsolute(uint32_t newSpeed) {
  if (newSpeed < 1) newSpeed = 1;
  speedHz = newSpeed;

  if (stepper != nullptr) {
    stepper->setSpeedInHz(speedHz);
    stepper->applySpeedAcceleration();
  }

  Serial.print("Rychlost nastavena na: ");
  Serial.print(speedHz);
  Serial.println(" Hz");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) return;

  if (cmd == "U") {
    setDirectionState(1);
  }
  else if (cmd == "D") {
    setDirectionState(-1);
  }
  else if (cmd == "S") {
    setDirectionState(0);
  }
  else if (cmd == "+") {
    changeSpeedRelative(100);
  }
  else if (cmd == "-") {
    changeSpeedRelative(-100);
  }
  else if (cmd.startsWith("R")) {
    int value = cmd.substring(1).toInt();
    if (value > 0) {
      setSpeedAbsolute((uint32_t)value);
    } else {
      Serial.println("Chyba: pouzij napr. R100");
    }
  }
  else if (cmd.startsWith("A")) {
    int value = cmd.substring(1).toInt();
    if (value > 0) {
      accelHz = (uint32_t)value;
      if (stepper != nullptr) {
        stepper->setAcceleration(accelHz);
        stepper->applySpeedAcceleration();
      }
      Serial.print("Akcelerace nastavena na: ");
      Serial.println(accelHz);
    } else {
      Serial.println("Chyba: pouzij napr. A2000");
    }
  }
  else if (cmd == "X") {
    if (stepper != nullptr) {
      stepper->setCurrentPosition(0);
    }
    encoder.setZero();
    Serial.println("Pozice a encoder nulovany");
  }
  else if (cmd == "?") {
    printStatus();
  }
  else {
    Serial.println("Neznamy prikaz.");
    Serial.println("Pouzij: U, D, S, R100, +, -, A2000, X, ?");
  }
}

void setupDriverTMC5160() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  delay(50);

  driver.begin();
  driver.toff(5);            // zapne driver
  driver.rms_current(600);  // proud motoru
  driver.microsteps(16);

  // stealthChop
  driver.en_pwm_mode(false);
  //driver.pwm_autoscale(true);
}

void setupFastAccelStepper() {
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (stepper == nullptr) {
    Serial.println("CHYBA: stepperConnectToPin() selhalo");
    while (true) {
      delay(1000);
    }
  }

  stepper->setDirectionPin(DIR_PIN, true);
  stepper->setEnablePin(EN_PIN, true);   // EN aktivni v LOW
  stepper->setAutoEnable(true);

  // volitelne: kratke zpozdeni po enable
  stepper->setDelayToEnable(50);
  stepper->setDelayToDisable(100);

  stepper->setAcceleration(accelHz);
  stepper->setSpeedInHz(speedHz);
  stepper->setCurrentPosition(0);

  // motor po startu stoji
  stepper->stopMove();
}

void printLoad() {
  uint16_t load = driver.sg_result();

  Serial.print("Zatizeni (SG): ");
  Serial.println(load);
}

void printHeignt() {
  //Serial.print("Uhel [deg]: ");
  //Serial.println(encoder.getAngleDegrees(), 2);

  //Serial.print("Otacky: ");
  //Serial.println(encoder.getRevolutions(), 4);

  Serial.print("Vyska [mm]: ");
  Serial.println(encoder.getLinearDistanceMM(MM_PER_REV), 3);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!encoder.begin(SDA_PIN, SCL_PIN, 400000)) {
    Serial.println("CHYBA: AS5600L nenalezen");
  } else {
    Serial.println("AS5600L OK");
    encoder.setZero();   // aktuální poloha = 0
  }

  setupDriverTMC5160();
  testSPI();
  delay(2000);

  setupFastAccelStepper();

  Serial.println();
  Serial.println("Ovládání přes Serial:");
  Serial.println("U      = nahoru");
  Serial.println("D      = dolu");
  Serial.println("S      = stop");
  Serial.println("R100   = nastavi rychlost na 100 Hz");
  Serial.println("+      = prida 100 Hz");
  Serial.println("-      = ubere 100 Hz");
  Serial.println("A2000  = nastavi akceleraci");
  Serial.println("X      = vynuluje pozici");
  Serial.println("?      = vypise stav");
  Serial.println();
}

void loop() {
  encoder.update();

  limitTop.update();
  limitBottom.update();

  // horni koncak - zastavi pohyb nahoru a vynuluje pozici
  if (limitTop.wasPressed() && motionState > 0) {    
    stopAndZeroPosition();
    Serial.println("Horni koncak sepnut");
  }

  // dolni koncak - zastavi pohyb dolu
  if (limitBottom.wasPressed() && motionState < 0) {
    stopMotor();
    Serial.println("Dolni koncak sepnut");
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    printLoad();
    printHeignt();
  }
}