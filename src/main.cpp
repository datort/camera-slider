#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include <AccelStepper.h>

#define APP_VERSION 1.0

#define CS 27
#define RS 26
#define SDA 33
#define SCK 32
#define RES 25
Adafruit_ST7735 tft = Adafruit_ST7735(CS, RS, SDA, SCK, RES);

#define ROTARY_SW 21
#define ROTARY_CLK 19 
#define ROTARY_DT 18 
ESP32Encoder encoder;
volatile int32_t position = 0;
int32_t lastEncoderValue = 0;
unsigned long lastTurnTime = 0;
const int slowThreshold = 200;

#define DIR 13
#define STEP 14
#define motorInterfaceType 1
AccelStepper motor(motorInterfaceType, STEP, DIR);

#define MOTOR_STOP_LEFT_PIN 36
#define MOTOR_STOP_RIGHT_PIN 39

const uint16_t Black = 0x0000; 
const uint16_t Red = 0xd800; 
const uint16_t Blue = 0x25ff; 
const uint16_t Green = 0x57c0; 
const uint16_t Cyan = 0x07be; 
const uint16_t Magenta = 0xf011; 
const uint16_t Yellow = 0xf740; 
const uint16_t White = 0xFFFF;

bool hasBeenCalibrated = false;
bool hasBeenConfigured = false;

unsigned int currentConfiguration = 0;
const int CURRENT_CONFIGURATION_BOUNDS = 0;
const int CURRENT_CONFIGURATION_SPEED = 1;
const int CURRENT_CONFIGURATION_ACCELERATION = 2;
const int CURRENT_CONFIGURATION_HOLD_TIME = 3;

int boundsLeft = -1;
int boundsRight = -1;
int motorCurrentTarget = -1;

int stopLeft = -1;
int stopRight = -1;
int previousStopLeft = -1;
int previousStopRight = -1;
int slideSpeed = 1000;
int slideAcceleration = 1000;
int slideHoldTime = 1;

int holdUntil = -1;

bool boundReached() {
  return digitalRead(MOTOR_STOP_LEFT_PIN) == HIGH || digitalRead(MOTOR_STOP_RIGHT_PIN) == HIGH;
}

void calibrate() {
  tft.fillScreen(Black);
  tft.setTextColor(White);
  tft.setCursor(10, 10);
  tft.print("Calibration");
  hasBeenCalibrated = false;

  motor.setAcceleration(1000);
  motor.setMaxSpeed(1800);
  motor.moveTo(99999);

  while(!boundReached()) {
    motor.run();
  }

  motor.setMaxSpeed(0);
  motor.stop();

  boundsLeft = motor.currentPosition() - 250;
  tft.setCursor(10, 40);
  tft.print("Bound left: ");
  tft.print(boundsLeft);

  motor.move(-400);
  motor.setMaxSpeed(500);

  while(motor.distanceToGo() != 0) {
    motor.run();
  }

  motor.moveTo(-99999);
  motor.setMaxSpeed(1800);

  while(!boundReached()) {
    motor.run();
  }

  motor.setMaxSpeed(0);
  motor.stop();

  boundsRight = motor.currentPosition() + 250;
      
  tft.setCursor(10, 50);
  tft.print("Bound right: ");
  tft.print(boundsRight);

  int totalScale = boundsLeft - boundsRight;
  motor.moveTo((totalScale / 2) + boundsRight);
  motor.setMaxSpeed(50000);

  while(motor.distanceToGo() != 0) {
    motor.run();
  }

  tft.setCursor(10, 70);
  tft.print("Calibration done");

  motorCurrentTarget = stopLeft = stopRight = motor.currentPosition();
  hasBeenCalibrated = true;
}

void readEncoderTaskBounds(int32_t newEncoderValue) {
  unsigned long currentTime = millis();
  unsigned long timeDifference = currentTime - lastTurnTime;

  int previousSpeed = motor.maxSpeed();
  motor.setMaxSpeed(0);

  int increment = (timeDifference > slowThreshold) ? 20 : 250;

  int newTarget = motorCurrentTarget;
  if (newEncoderValue > lastEncoderValue) {
    newTarget -= increment;
  } else {
    newTarget += increment;
  }
  
  lastEncoderValue = newEncoderValue;
  lastTurnTime = currentTime;
  
  motor.setMaxSpeed(previousSpeed);

  if (newTarget < boundsLeft && newTarget > boundsRight) {
    motorCurrentTarget = newTarget;
    motor.moveTo(motorCurrentTarget);
  }
}

void readEncoderTaskSpeed(int32_t newEncoderValue) {
  unsigned long currentTime = millis();
  unsigned long timeDifference = currentTime - lastTurnTime;
  int increment = (timeDifference > slowThreshold) ? 100 : 1000;

  int newTarget = slideSpeed;
  if (newEncoderValue > lastEncoderValue) {
    newTarget += increment;
  } else {
    newTarget -= increment;
  }
  
  lastEncoderValue = newEncoderValue;
  lastTurnTime = currentTime;

  if (newTarget < 50000 && newTarget > 50) {
    Serial.println(newTarget);
    slideSpeed = newTarget;
  }
}

void readEncoderTaskAcceleration(int32_t newEncoderValue) {
  unsigned long currentTime = millis();
  unsigned long timeDifference = currentTime - lastTurnTime;
  int increment = (timeDifference > slowThreshold) ? 100 : 1000;

  int newTarget = slideAcceleration;
  if (newEncoderValue > lastEncoderValue) {
    newTarget += increment;
  } else {
    newTarget -= increment;
  }
  
  lastEncoderValue = newEncoderValue;
  lastTurnTime = currentTime;

  if (newTarget < 20000 && newTarget > 50) {
    slideAcceleration = newTarget;
  }
}

void readEncoderTaskHoldTime(int32_t newEncoderValue) {
  unsigned long currentTime = millis();
  unsigned long timeDifference = currentTime - lastTurnTime;
  int increment = (timeDifference > slowThreshold) ? 1 : 2;

  int newTarget = slideHoldTime;
  if (newEncoderValue > lastEncoderValue) {
    newTarget += increment;
  } else {
    newTarget -= increment;
  }
  
  lastEncoderValue = newEncoderValue;
  lastTurnTime = currentTime;

  if (newTarget < 21 && newTarget > 0) {
    slideHoldTime = newTarget;
  }
}

void readEncoderTask(void * parameter) {
  while (true) {
    if (hasBeenCalibrated && hasBeenConfigured) continue;

    int32_t newEncoderValue = encoder.getCount();
    
    if (newEncoderValue != lastEncoderValue) {
      switch (currentConfiguration) {
        case CURRENT_CONFIGURATION_BOUNDS:
          readEncoderTaskBounds(newEncoderValue);
          break;
        case CURRENT_CONFIGURATION_SPEED:
          readEncoderTaskSpeed(newEncoderValue);
          break;
        case CURRENT_CONFIGURATION_ACCELERATION:
          readEncoderTaskAcceleration(newEncoderValue);
          break;
        case CURRENT_CONFIGURATION_HOLD_TIME:
          readEncoderTaskHoldTime(newEncoderValue);
          break;
      }
    }

    vTaskDelay(1);
  }
}

void configureSlide() {
  int lastDigit;
  motor.setMaxSpeed(2000);
  motor.setAcceleration(1000);

  tft.fillScreen(Black);
  tft.setTextColor(White);
  tft.setCursor(10, 10);
  tft.print("Configure Slide");

  // Config Start
  tft.setCursor(10, 30);
  tft.print("Start: ");

  if (stopLeft != -1) {
    motorCurrentTarget = stopLeft;
    motor.moveTo(stopLeft);
    tft.print(stopLeft);
  }

  while (digitalRead(ROTARY_SW) == HIGH) {
    if (motorCurrentTarget != stopLeft) {
      tft.fillRect(50, 30, 35, 8, Red);
      tft.setCursor(50, 30);
      tft.print(motorCurrentTarget);

      stopLeft = motorCurrentTarget;
    }

    motor.run();
  }

  tft.fillRect(50, 30, 35, 8, Black);
  tft.setCursor(50, 30);
  tft.print(stopLeft);

  while (digitalRead(ROTARY_SW) == LOW) {}

  // Config End
  tft.setCursor(10, 40);
  tft.print("End:");

  if (stopRight != -1) {
    motorCurrentTarget = stopRight;
    motor.moveTo(stopRight);
    tft.setCursor(50, 40);
    tft.print(stopRight);
  }

  while (digitalRead(ROTARY_SW) == HIGH) {
    if (motorCurrentTarget != stopRight) {
      tft.fillRect(50, 40, 35, 8, Red);
      tft.setCursor(50, 40);
      tft.print(motorCurrentTarget);

      stopRight = motorCurrentTarget;
    }

    motor.run();
  }

  tft.fillRect(50, 40, 35, 8, Black);
  tft.setCursor(50, 40);
  tft.print(stopRight);

  while (digitalRead(ROTARY_SW) == LOW) {}

  // Config Speed
  currentConfiguration = CURRENT_CONFIGURATION_SPEED;
  tft.setCursor(10, 50);
  tft.print("Speed:");

  
  while (digitalRead(ROTARY_SW) == HIGH) {
    if (slideSpeed != lastDigit) {
      tft.fillRect(50, 50, 35, 8, Red);
      tft.setCursor(50, 50);
      tft.print(slideSpeed);

      lastDigit = slideSpeed;
    }
  }

  tft.fillRect(50, 50, 35, 8, Black);
  tft.setCursor(50, 50);
  tft.print(slideSpeed);

  while (digitalRead(ROTARY_SW) == LOW) {}
  motor.setMaxSpeed(slideSpeed);


  // Config Acceleration
  lastDigit = -1;
  currentConfiguration = CURRENT_CONFIGURATION_ACCELERATION;
  tft.setCursor(10, 60);
  tft.print("Accel:");

  while (digitalRead(ROTARY_SW) == HIGH) {
    if (slideAcceleration != lastDigit) {
      tft.fillRect(50, 60, 35, 8, Red);
      tft.setCursor(50, 60);
      tft.print(slideAcceleration);

      lastDigit = slideAcceleration;
    }
  }

  tft.fillRect(50, 60, 35, 8, Black);
  tft.setCursor(50, 60);
  tft.print(slideAcceleration);

  while (digitalRead(ROTARY_SW) == LOW) {}
  motor.setAcceleration(slideAcceleration);


  // Config Hold Time
  lastDigit = -1;
  currentConfiguration = CURRENT_CONFIGURATION_HOLD_TIME;
  tft.setCursor(10, 70);
  tft.print("Hold:");

  while (digitalRead(ROTARY_SW) == HIGH) {
    if (slideHoldTime != lastDigit) {
      tft.fillRect(50, 70, 35, 8, Red);
      tft.setCursor(50, 70);
      tft.print(slideHoldTime);

      lastDigit = slideHoldTime;
    }
  }

  tft.fillRect(50, 70, 35, 8, Black);
  tft.setCursor(50, 70);
  tft.print(slideHoldTime);

  while (digitalRead(ROTARY_SW) == LOW) {}


  currentConfiguration = CURRENT_CONFIGURATION_BOUNDS;

  tft.setCursor(10, 80);
  tft.setTextColor(Green);
  tft.print("Push Button to Start");

  while (digitalRead(ROTARY_SW) == HIGH) {}

  delay(2000);

  hasBeenConfigured = true;

  tft.setTextColor(Red);
  tft.setCursor(10, 90);
  tft.print("Running slide...");
}

void runSlide() {
  while (digitalRead(ROTARY_SW) == LOW) {
    hasBeenConfigured = false;

    return;
  }

  if (motor.distanceToGo() == 0) {
    if (slideHoldTime > 0 && holdUntil == -1) {
      holdUntil = millis() + (slideHoldTime * 1000);

      while (millis() < holdUntil) {}

      holdUntil = -1;
    }

    motor.moveTo(motor.currentPosition() != stopLeft ? stopLeft : stopRight);
  }
}


void setup() {
  Serial.begin(9600);

  pinMode(ROTARY_SW, INPUT_PULLUP);
  pinMode(MOTOR_STOP_LEFT_PIN, INPUT_PULLDOWN);
  pinMode(MOTOR_STOP_RIGHT_PIN, INPUT_PULLDOWN);
  encoder.attachHalfQuad(ROTARY_DT, ROTARY_CLK);
  encoder.setCount(0);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(Black);
  tft.setTextColor(White);
  tft.setFont();

  tft.fillTriangle(0, 30, 0, 0, 30, 0, Magenta);
  tft.fillTriangle(160, 30, 160, 0, 130, 0, Magenta);
  tft.fillTriangle(0, 98, 0, 128, 30, 128, Magenta);
  tft.fillTriangle(160, 98, 160, 128, 130, 128, Magenta);
  tft.setCursor(40, 80);
  tft.print("Camera Slider v");
  tft.print(String(APP_VERSION));
  tft.setCursor(40, 90);
  tft.setTextColor(Blue);
  tft.print("datort.de");
  delay(2500);

  xTaskCreatePinnedToCore(
    readEncoderTask,
    "EncoderTask",
    10000,
    NULL,
    1,
    NULL,
    1
  );

  Serial.println("Camera Slider ready.");
}

void loop() {
  if (!hasBeenCalibrated) calibrate();
  else if(!hasBeenConfigured) configureSlide();
  else if (hasBeenCalibrated && hasBeenConfigured) runSlide();
  
  motor.run();
}