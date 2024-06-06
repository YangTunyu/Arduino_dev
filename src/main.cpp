//include部分
#include "Freenove_WS2812_Lib_for_ESP32.h"
#define LEDS_COUNT  8
#define LEDS_PIN    13
#define CHANNEL     0
#include <Arduino.h>
#define MOTOR_INA 23
#define MOTOR_INB 22
#define SWITCH_PIN 13
#define GREEN_LED 27
#define RED_LED 14









//其余部分
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

enum Mode { GRADIENT, JUMP, FLASH, RAINBOW, BREATH };
Mode mode = GRADIENT;

enum MotorState {STOP, REVERSE, FORWARD};
MotorState motorState = STOP;

bool lastSwitchState = HIGH; // 按钮的上一个状态，初始为未按下
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;











//setup部分
void setup() {
  strip.begin();
  strip.setBrightness(20);

  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP); // 使用内部上拉电阻
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);  
}










//loopb部分
void loop() {
  switch(mode) {
    case GRADIENT:
      for (int j = 0; j < 255; j += 2) {
        for (int i = 0; i < LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel((i * 256 / LEDS_COUNT + j) & 255));
        }
        strip.show();
        delay(10);
      }
      delay(2000);  // 2 seconds delay
      mode = JUMP;
      break;
    case JUMP:
      for (int i = 0; i < LEDS_COUNT; i++) {
        strip.setLedColorData(i, strip.Wheel(random(0, 255)));
      }
      strip.show();
      delay(2000);  // 2 seconds delay
      mode = FLASH;
      break;
    case FLASH:
      for (int i = 0; i < LEDS_COUNT; i++) {
        strip.setLedColorData(i, strip.Wheel(random(0, 255)));
      }
      strip.show();
      delay(100);
      for (int i = 0; i < LEDS_COUNT; i++) {
        strip.setLedColorData(i, 0);
      }
      strip.show();
      delay(100);
      delay(2000);  // 2 seconds delay
      mode = RAINBOW;
      break;
    case RAINBOW:
      for (int j = 0; j < 256; j++) {
        for (int i = 0; i < LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel((i + j) & 255));
        }
        strip.show();
        delay(10);
      }
      delay(2000);  // 2 seconds delay
      mode = BREATH;
      break;
    case BREATH:
      for (int j = 0; j < 255; j++) {
        for (int i = 0; i < LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel(j));
        }
        strip.show();
        delay(10);
      }
      for (int j = 255; j > 0; j--) {
        for (int i = 0; i < LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel(j));
        }
        strip.show();
        delay(10);
      }
      delay(2000);  // 2 seconds delay
      mode = GRADIENT;
      break;
  }

    bool switchState = digitalRead(SWITCH_PIN);

  if (switchState != lastSwitchState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (switchState == LOW) {
      motorState = static_cast<MotorState>((motorState + 1) % 3);
    }
  }

  lastSwitchState = switchState;

  switch (motorState) {
    case STOP:
      digitalWrite(MOTOR_INA, LOW);
      digitalWrite(MOTOR_INB, LOW);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, LOW);
      break;
    case REVERSE:
      digitalWrite(MOTOR_INA, LOW);
      digitalWrite(MOTOR_INB, HIGH);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      break;
    case FORWARD:
      digitalWrite(MOTOR_INA, HIGH);
      digitalWrite(MOTOR_INB, LOW);
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      break;
  }
}
