#include <Arduino.h>

// 定义红外人体感应模块和LED灯的引脚
const int sensorPin = 22; // 人体感应模块的输出引脚连接到GPIO22
const int ledPin = 13;     // LED灯连接到GPIO13

void setup() {
  pinMode(sensorPin, INPUT); // 设置人体感应模块引脚为输入
  pinMode(ledPin, OUTPUT);   // 设置LED灯引脚为输出
}

void loop() {
  if (digitalRead(sensorPin) == HIGH) {
    digitalWrite(ledPin, HIGH); // 如果检测到人体，点亮LED灯
  } else {
    digitalWrite(ledPin, LOW);  // 否则，熄灭LED灯
  }
}