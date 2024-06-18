//pah上升以及暂停
#include <Stepper.h>
#include <Arduino.h>

const int stepsPerRevolution = 20000;
const int startPin = 13;
const int stopPin = 33;
bool isStartPressed = false;
bool isStopPressed = false;
bool isMotorRunning = false;

Stepper myStepper(stepsPerRevolution, 14, 27, 26, 25);

void setup() {
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  int sensorReading = analogRead(A0);
  int motorSpeed = map(sensorReading, 0, 1023, 0, 100);

  bool currentStartState = digitalRead(startPin) == LOW;
  bool currentStopState = digitalRead(stopPin) == LOW;

  if (currentStartState != isStartPressed) {
    isStartPressed = currentStartState;
    if (isStartPressed && !isMotorRunning) {
      // 启动按钮被按下,且电机未运行
      myStepper.setSpeed(motorSpeed);
      isMotorRunning = true;
    }
  }

  if (currentStopState != isStopPressed) {
    isStopPressed = currentStopState;
    if (isStopPressed && isMotorRunning) {
      // 停止按钮被按下,且电机正在运行
      myStepper.setSpeed(0);
      isMotorRunning = false;
    }
  }

  if (isMotorRunning) {
    myStepper.step(stepsPerRevolution / 100);
  }
}