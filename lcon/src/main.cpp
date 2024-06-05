#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>

BH1750 lightSensor;
const int ledPin = 16; // LED的控制引脚连接到GPIO 16
const int pirPin = 14; // HC-SR312微型人体感应模块的信号引脚连接到GPIO 14

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(pirPin, INPUT);

  Wire.begin(21, 22); // 初始化I2C总线，SDA连接到GPIO 21，SCL连接到GPIO 22
  lightSensor.begin();
}

void loop() {
  // 读取光线强度
  uint16_t lux = lightSensor.readLightLevel();
  Serial.print("Light level (lux): ");
  Serial.println(lux);

  // 读取人体感应模块状态
  int motionDetected = digitalRead(pirPin);

  // 当光线暗且有人时，点亮LED
  if (lux < 500 && motionDetected == HIGH) {
    digitalWrite(ledPin, HIGH); // 点亮LED
  } else {
    digitalWrite(ledPin, LOW); // 关闭LED
  }

  delay(1000); // 延迟1秒钟
}
