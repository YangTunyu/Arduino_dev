#include <Arduino.h>

// yxr照明
const int buttonPin = 0;         // 开关连接到GPIO2 (D4)
const int ledPin = 13;           // LED连接到GPIO13 (D7)
const int potentiometerPin = 36; // 电位器连接到GPIO36 (A0)

int buttonState = 0;        // 开关状态
bool ledState = false;      // LED状态
int lastButtonState = HIGH; // 上一次的开关状态
long lastDebounceTime = 0;  // 上一次按下或释放开关的时间
long debounceDelay = 50;    // 防抖延迟时间
int brightness = 0;         // LED亮度

// jcy紫外线
unsigned long lastButtonPressTime = 0;
const unsigned long ledAutoOffInterval = 1000; // 15 seconds in milliseconds

//pzy 风扇
const int buttonPin = 0; // 按钮连接到 GPIO0 
const int fanPinA = 2; // 风扇的A引脚连接到 GPIO4 
const int fanPinB = 4; // 风扇的B引脚连接到 GPIO2 

int buttonState = 0;
bool deviceState = false; // 控制LED和风扇的状态
unsigned long lastButtonPressTime = 0;
const unsigned long deviceAutoOffInterval = 15000; // 设备自动关闭的时间间隔，15秒

void setup()
{
  // yxr 照明
  pinMode(buttonPin, INPUT_PULLUP); // 设置开关引脚为输入，带上拉电阻
  pinMode(ledPin, OUTPUT);          // 设置LED引脚为输出
  Serial.begin(9600);               // 初始化串口通信


  //pzy 风扇
  pinMode(buttonPin, INPUT_PULLUP);
pinMode(fanPinA, OUTPUT);
pinMode(fanPinB, OUTPUT);
Serial.begin(9600);
}

void loop()
{
  // yxr照明
  //  读取开关状态，进行防抖处理
  int reading = digitalRead(buttonPin);
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
  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW)
  {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    Serial.print("Button pressed. LED is now ");
    Serial.println(ledState ? "ON" : "OFF");

    if (ledState)
    {
      lastButtonPressTime = millis(); // Record the time when LED was turned ON
    }

    delay(200);
  }

  // Check if it's time to turn off the LED automatically
  if (ledState && (millis() - lastButtonPressTime >= ledAutoOffInterval))
  {
    ledState = false;
    digitalWrite(ledPin, LOW);
    Serial.println("Auto turning off LED.");
  }

//pzy 风干功能
  buttonState = digitalRead(buttonPin);

if (buttonState == LOW) {
 deviceState = !deviceState;
digitalWrite(fanPinA, deviceState);
digitalWrite(fanPinB, LOW);
Serial.print("按钮被按下。LED和风扇现在都");
 Serial.println(deviceState ? "开启" : "关闭");

if (deviceState) {
 lastButtonPressTime = millis(); // 记录LED和风扇开启的时间
 }

 delay(200);
  }

// 检查是否到了自动关闭LED和风扇的时间
if (deviceState && (millis() - lastButtonPressTime >= deviceAutoOffInterval)) {
 deviceState = false;
 //digitalWrite(ledPin, LOW);
 digitalWrite(fanPinA, LOW);
 digitalWrite(fanPinB, LOW);
 Serial.println("自动关闭LED和风扇。");
 }
}


