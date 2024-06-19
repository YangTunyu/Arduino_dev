//include部分
#include "Stepper_28BYJ_48.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Arduino.h>












//引脚定义
int startPin = 13; // 连接到启动按钮的引脚
int stopPin = 23; // 连接到停止按钮的引脚
int reversePin = 22; // 连接到反向旋转按钮的引脚
const int buttonPin1 = 5;         // 开关连接到GPIO2 (D4)
const int ledPin = 13;           // LED连接到GPIO13 (D7)
const int potentiometerPin = 36; // 电位器连接到GPIO36 (A0)













//其余部分
//pah
Stepper_28BYJ_48 stepper(14, 27, 26, 25);

bool isRunning = false; // 用于跟踪电机的状态
int direction = 1; // 用于控制电机的旋转方向
int stepsCount = 0; // 用于计算电机转动的步数

const int stepsPerRevolution = 512; // 一圈步进电机（28BYJ-48）有512步
const int revolutions = 2; // 每次转动二圈

const int maxSteps = 2 * stepsPerRevolution; // 总步数设置为两圈的步数

// yxr照明
int buttonState = 0;        // 开关状态
bool ledState = false;      // LED状态
int lastButtonState = HIGH; // 上一次的开关状态
long lastDebounceTime = 0;  // 上一次按下或释放开关的时间
long debounceDelay = 50;    // 防抖延迟时间
int brightness = 0;         // LED亮度















//setup
//pah
void setup() {
  //pah
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(reversePin, INPUT_PULLUP);
  
  // yxr 照明
  pinMode(buttonPin1, INPUT_PULLUP); // 设置开关引脚为输入，带上拉电阻
  pinMode(ledPin, OUTPUT);          // 设置LED引脚为输出
  Serial.begin(9600);               // 初始化串口通信
}















//loop


void loop() {
  //pah
  if (digitalRead(startPin) == LOW) {
    isRunning = true; // 启动电机
    direction = 1; // 设置旋转方向为正向
  }

  if (digitalRead(reversePin) == LOW) {
    isRunning = true; // 启动电机
    direction = -1; // 设置旋转方向为反向
  }

  if (digitalRead(stopPin) == LOW) {
    isRunning = false; // 停止电机
  }

  if (isRunning) {
    stepper.step(direction); // 如果电机正在运行，则按照设置的方向旋转
    stepsCount += direction; // 正向加1，反向减1
    if (stepsCount >= maxSteps || stepsCount <= -maxSteps) {
      isRunning = false; // 达到最大步数范围时停止电机运行
    }
  }


  // yxr照明
  //  读取开关状态，进行防抖处理
  int reading = digitalRead(buttonPin1);
  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
  }
  if (millis() - lastDebounceTime > debounceDelay)
  {
    if (reading != buttonState)
    {
      buttonState = reading;
      if (buttonState == LOW)
      {
        ledState = !ledState; // 切换LED状态
        Serial.print("Button pressed. LED is now ");
        Serial.println(ledState ? "ON" : "OFF");
      }
    }
  }
  lastButtonState = reading;

  // 读取电位器数值，并映射到LED亮度范围
  int potValue = analogRead(potentiometerPin);
  brightness = map(potValue, 0, 4095, 0, 255);

  // 打印电位器数值到串口监视器
  Serial.print("Potentiometer value: ");
  Serial.println(potValue);

  // 根据LED状态控制LED亮度
  if (ledState)
  {
    analogWrite(ledPin, brightness); // 设置LED亮度
  }
  else
  {
    analogWrite(ledPin, 0); // LED关闭
  }

  delay(10); // 稍作延迟以提高稳定性

}
