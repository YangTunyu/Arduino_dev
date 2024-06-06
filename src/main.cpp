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



//yxr
#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>










//其余部分
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

enum Mode { GRADIENT, JUMP, FLASH, RAINBOW, BREATH };
Mode mode = GRADIENT;

enum MotorState {STOP, REVERSE, FORWARD};
MotorState motorState = STOP;

bool lastSwitchState = HIGH; // 按钮的上一个状态，初始为未按下
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;




//yxr
BH1750 lightSensor;
const int ledPin = 16; // LED的控制引脚连接到GPIO 16
const int pirPin = 14; // HC-SR312微型人体感应模块的信号引脚连接到GPIO 14
// 定义引脚
const int analogPin = 36;   // 模拟输入引脚
const int digitalPin = 39;  // 数字输入引脚
const int ioPin = 27;       // I/O接口引脚
const int buzzerPin = 13;   // 蜂鸣器引脚

// 定义阈值
const int threshold = 4000;  // 模拟输入阈值
const int alarmThreshold = 1; // 数字输入阈值




//pzy
// 定义红外人体感应模块和LED灯的引脚
const int sensorPin = 14; // 人体感应模块的输出引脚连接到GPIO22
const int ledPin = 16;     // LED灯连接到GPIO13

int potpin= 15;//定义模拟ADC接口15
int ledpin= 16;//定义数字接口16（PWM 输出）
int val=0;// 暂存来自传感器的变量数值









//setup部分
void setup() {
  strip.begin();
  strip.setBrightness(20);

  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP); // 使用内部上拉电阻
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);  
  
  
  
  
  
  
  //yxr
    Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(pirPin, INPUT);

  Wire.begin(21, 22); // 初始化I2C总线，SDA连接到GPIO 21，SCL连接到GPIO 22
  lightSensor.begin();
  Serial.begin(9600);
  pinMode(analogPin, INPUT);
  pinMode(digitalPin, INPUT);
  pinMode(ioPin, INPUT);
  pinMode(buzzerPin, OUTPUT); 
  digitalWrite(buzzerPin, HIGH);
  // 设置蜂鸣器引脚为输出模式



  //pzy
pinMode(sensorPin, INPUT); // 设置人体感应模块引脚为输入
  pinMode(ledPin, OUTPUT);   // 设置LED灯引脚为输出

  pinMode(ledpin,OUTPUT);//定义数字接口11 为输出
Serial.begin(9600);//设置波特率为9600
//模拟接口自动设置为输入

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
  
  
  
  
  
  
  
  
  
  //yxr
    // 读取光线强度
  uint16_t lux = lightSensor.readLightLevel();
  Serial.print("Light level (lux): ");
  Serial.println(lux);

  // 读取人体感应模块状态
  int motionDetected = digitalRead(pirPin);
  Serial.print("People Condition: ");
  Serial.println(motionDetected);
  

  // 当光线暗且有人时，点亮LED
  if (lux < 500 && motionDetected == HIGH) {
    digitalWrite(ledPin, HIGH); // 点亮LED
    Serial.print("Light Condition: HIGH "); 
  } else {
    digitalWrite(ledPin, LOW); // 关闭LED
    Serial.print("Light Condition: LOW ");
  }
  delay(1000); // 延迟1秒钟
  // 读取模拟输入
  int sensorValue = analogRead(analogPin);

  // 如果模拟输入值超过阈值，触发告警
  if (sensorValue > threshold) {
    Serial.println("Gas leak detected!");
    // 触发蜂鸣器
    digitalWrite(buzzerPin, HIGH);  // 将蜂鸣器引脚设置为高电平
    delay(1000);  // 延迟1秒钟
    digitalWrite(buzzerPin, LOW);  // 将蜂鸣器引脚设置为低电平
  }

  // 读取数字输入
  int digitalValue = digitalRead(digitalPin);

  // 如果数字输入为高电平，表示发生告警
  if (digitalValue == LOW) {
    Serial.println("Digital alarm triggered!");
    // 触发蜂鸣器
    // digitalWrite(buzzerPin, HIGH);  // 将蜂鸣器引脚设置为高电平
    // delay(1000);  // 延迟1秒钟
    digitalWrite(buzzerPin, LOW);  // 将蜂鸣器引脚设置为低电平
  }else{
    digitalWrite(buzzerPin, HIGH); 
  }

  // 读取I/O接口状态
  // int ioValue = digitalRead(ioPin);

  // // 如果I/O接口为高电平，表示发生告警
  // if (ioValue == HIGH) {
  //   Serial.println("I/O alarm triggered!");
  //   // 触发蜂鸣器
  //   digitalWrite(buzzerPin, HIGH);  // 将蜂鸣器引脚设置为高电平
  //   delay(1000);  // 延迟1秒钟
  //   digitalWrite(buzzerPin, LOW);  // 将蜂鸣器引脚设置为低电平
  // }

  delay(1000); // 延迟1秒钟





//pzy
  if (digitalRead(sensorPin) == HIGH) {
    digitalWrite(ledPin, HIGH); // 如果检测到人体，点亮LED灯
  } else {
    digitalWrite(ledPin, LOW);  // 否则，熄灭LED灯
  }


  val=analogRead(potpin);// 读取传感器的模拟值并赋值给val  读取模拟值范围 0-1023
Serial.println(val);//显示val 变量  用来串口监视

analogWrite(ledpin,val/4);// 打开LED 并设置亮度（PWM 输出最大值255）
delay(10);//延时0.01 秒
}
