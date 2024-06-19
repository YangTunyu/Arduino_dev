// include部分
#include "Stepper_28BYJ_48.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Arduino.h>

// 引脚定义
int startPin = 13;   // 连接到启动按钮的引脚
int stopPin = 23;    // 连接到停止按钮的引脚
int reversePin = 22; // 连接到反向旋转按钮的引脚

const int buttonPin1 = 5;        // 开关连接到GPIO2 (D4)
const int ledPin = 33;           // LED连接到GPIO33 (D7)
const int potentiometerPin = 36; // 电位器连接到GPIO36 (A0)

const int buttonPin2 = 0; // 按钮连接到 GPIO0
const int fanPinA = 2;    // 风扇的A引脚连接到 GPIO2
const int fanPinB = 4;    // 风扇的B引脚连接到 GPIO4

int ledpin2 = 17;          // 连接到LED引脚
const int buttonPin3 = 16; // 连接到开关紫外线按钮引脚

const int buttonPin4 = 25; // 按钮连接到 GPIO25 (D4)
const int ledPin4 = 26; // LED连接到 GPIO26 (D7)
const int fanPinA4 = 27; // 风扇的A引脚连接到 GPIO27 (D2)
const int fanPinB4 = 14; // 风扇的B引脚连接到 GPIO14 (D1)

// 其余部分
// pah
Stepper_28BYJ_48 stepper(14, 27, 26, 25);

bool isRunning = false; // 用于跟踪电机的状态
int direction = 1;      // 用于控制电机的旋转方向
int stepsCount = 0;     // 用于计算电机转动的步数

const int stepsPerRevolution = 512; // 一圈步进电机（28BYJ-48）有512步
const int revolutions = 2;          // 每次转动二圈

const int maxSteps = 2 * stepsPerRevolution; // 总步数设置为两圈的步数

// yxr照明
int buttonState = 0;        // 开关状态
bool ledState = false;      // LED状态
int lastButtonState = HIGH; // 上一次的开关状态
long lastDebounceTime = 0;  // 上一次按下或释放开关的时间
long debounceDelay = 50;    // 防抖延迟时间
int brightness = 0;         // LED亮度

// pzy风扇
int buttonState2 = 0;
bool deviceState = false; // 控制LED和风扇的状态
unsigned long lastButtonPressTime2 = 0;
const unsigned long deviceAutoOffInterval2 = 15000; // 设备自动关闭的时间间隔，15秒

// jcy紫外线
bool ledState2 = false;
int buttonState3 = 0;
unsigned long lastButtonPressTime = 0;
const unsigned long ledAutoOffInterval = 1000; // 15 seconds in milliseconds

//pzy烘干
int buttonState4 = 0;
bool deviceState4 = false; // 控制LED和风扇的状态
unsigned long lastButtonPressTime4 = 0;
const unsigned long deviceAutoOffInterval4 = 15000; // 设备自动关闭的时间间隔，15秒

// setup
// pah
void setup()
{
  // pah
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(reversePin, INPUT_PULLUP);

  // yxr 照明
  pinMode(buttonPin1, INPUT_PULLUP); // 设置开关引脚为输入，带上拉电阻
  pinMode(ledPin, OUTPUT);           // 设置LED引脚为输出
  Serial.begin(9600);                // 初始化串口通信

  // pzy风扇
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(fanPinA, OUTPUT);
  pinMode(fanPinB, OUTPUT);
  Serial.begin(9600);

  // jcy 紫外线
  pinMode(buttonPin3, INPUT_PULLUP); // 设置开关引脚为输入，带上拉电阻
  pinMode(ledpin2, OUTPUT);          // 设置LED引脚为输出
  Serial.begin(9600);

  //pzy烘干
  pinMode(buttonPin4, INPUT_PULLUP);
pinMode(ledPin4, OUTPUT);
pinMode(fanPinA4, OUTPUT);
pinMode(fanPinB4, OUTPUT);
Serial.begin(9600);
}

// loop

void loop()
{
  // pah
  if (digitalRead(startPin) == LOW)
  {
    isRunning = true; // 启动电机
    direction = 1;    // 设置旋转方向为正向
  }

  if (digitalRead(reversePin) == LOW)
  {
    isRunning = true; // 启动电机
    direction = -1;   // 设置旋转方向为反向
  }

  if (digitalRead(stopPin) == LOW)
  {
    isRunning = false; // 停止电机
  }

  if (isRunning)
  {
    stepper.step(direction); // 如果电机正在运行，则按照设置的方向旋转
    stepsCount += direction; // 正向加1，反向减1
    if (stepsCount >= maxSteps || stepsCount <= -maxSteps)
    {
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

  // jcy 紫外线
  buttonState3 = digitalRead(buttonPin3);

  if (buttonState3 == LOW)
  {
    ledState2 = !ledState2;
    digitalWrite(ledpin2, ledState2);
    Serial.print("Button pressed. LED is now ");
    Serial.println(ledState2 ? "ON" : "OFF");

    if (ledState2)
    {
      lastButtonPressTime = millis(); // Record the time when LED was turned ON
    }

    delay(200);
  }

  // Check if it's time to turn off the LED automatically
  if (ledState2 && (millis() - lastButtonPressTime >= ledAutoOffInterval))
  {
    ledState2 = false;
    digitalWrite(ledpin2, LOW);
    Serial.println("Auto turning off LED.");
  }

  // pzy 风干功能
  buttonState2 = digitalRead(buttonPin2);

  if (buttonState2 == LOW)
  {
    deviceState = !deviceState;
    digitalWrite(fanPinA, deviceState);
    digitalWrite(fanPinB, LOW);
    Serial.print("按钮被按下。LED和风扇现在都");
    Serial.println(deviceState ? "开启" : "关闭");

    if (deviceState)
    {
      lastButtonPressTime2 = millis(); // 记录LED和风扇开启的时间
    }

    delay(200);
  }

  // 检查是否到了自动关闭LED和风扇的时间
  if (deviceState && (millis() - lastButtonPressTime2 >= deviceAutoOffInterval2))
  {
    deviceState = false;
    // digitalWrite(ledPin, LOW);
    digitalWrite(fanPinA, LOW);
    digitalWrite(fanPinB, LOW);
    Serial.println("自动关闭LED和风扇。");
  }

//pzy烘干功能
buttonState4 = digitalRead(buttonPin4);

if (buttonState4 == LOW) {
 deviceState4 = !deviceState4;
digitalWrite(ledPin4, deviceState4);
digitalWrite(fanPinA4, deviceState4);
 digitalWrite(fanPinB4, LOW);
Serial.print("按钮被按下。LED和风扇现在都");
 Serial.println(deviceState4 ? "开启" : "关闭");

if (deviceState4) {
 lastButtonPressTime4 = millis(); // 记录LED和风扇开启的时间
 }

 delay(200);
  }

// 检查是否到了自动关闭LED和风扇的时间
if (deviceState4 && (millis() - lastButtonPressTime4 >= deviceAutoOffInterval4)) {
 deviceState4 = false;
 digitalWrite(ledPin4, LOW);
 digitalWrite(fanPinA4, LOW);
 digitalWrite(fanPinB4, LOW);
 Serial.println("自动关闭LED和风扇。");
 }

}
