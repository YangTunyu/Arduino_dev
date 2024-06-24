// include部分
#include "Stepper_28BYJ_48.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "HX711.h"


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

const int timeButtonPin2 = 19; // 新增的负责加定时时间的按钮连接到 GPIO3

int ledpin2 = 17;          // 连接到LED引脚
const int buttonPin3 = 16; // 连接到开关紫外线按钮引脚

const int buttonPin4 = 25; // 按钮连接到 GPIO25 (D4)
const int ledPin4 = 26;    // LED连接到 GPIO26 (D7)
const int fanPinA4 = 27;   // 风扇的A引脚连接到 GPIO27 (D2)
const int fanPinB4 = 14;   // 风扇的B引脚连接到 GPIO14 (D1)

const int timeButtonPin = 18; // 新增的负责加定时时间的按钮连接到 GPIO3 (D3)

const int buzzerPin = 21; // 连接到蜂鸣器的引脚

// 其余部分
// pah
Stepper_28BYJ_48 stepper(14, 27, 26, 25);

bool isRunning = false; // 用于跟踪电机的状态
int direction = 1;      // 用于控制电机的旋转方向
int stepsCount = 0;     // 用于计算电机转动的步数

const int stepsPerRevolution = 512; // 一圈步进电机（28BYJ-48）有512步
const int revolutions = 4;          // 每次转动四圈

const int maxSteps = revolutions * stepsPerRevolution; // 总步数设置为四圈的步数


// zby 阻力重量检测
  
bool buzzerState = false; // 蜂鸣器状态

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
const float CALIBRATION_FACTOR = 833.33; // 100000 / 120 = 833.33
const float INITIAL_WEIGHT = 100.0; // Initial weight of the container in grams

HX711 scale;
float initialWeight = 0.0;


// yxr照明
int buttonState = 0;        // 开关状态
bool ledState = false;      // LED状态
int lastButtonState = HIGH; // 上一次的开关状态
long lastDebounceTime = 0;  // 上一次按下或释放开关的时间
long debounceDelay = 50;    // 防抖延迟时间
int brightness = 0;         // LED亮度
bool printed = false;

// pzy风扇
int buttonState2 = 0;
bool deviceState = false; // 控制LED和风扇的状态
unsigned long lastButtonPressTime2 = 0;
unsigned long deviceAutoOffInterval2 = 15000; // 设备自动关闭的时间间隔，15秒

// jcy风干定时
int lastButtonState2 = HIGH;
int timeButtonState2 = HIGH;
int lastTimeButtonState2 = HIGH;

unsigned long debounceDelay2 = 50;   // 去抖动延迟
unsigned long lastDebounceTime2 = 0; // 上次去抖动时间

// jcy紫外线
bool ledState2 = false;
int buttonState3 = 0;
unsigned long lastButtonPressTime = 0;
const unsigned long ledAutoOffInterval = 15000; // 15 seconds in milliseconds

// pzy烘干
int buttonState4 = 0;
bool deviceState4 = false; // 控制LED和风扇的状态
unsigned long lastButtonPressTime4 = 0;
unsigned long deviceAutoOffInterval4 = 15000; // 设备自动关闭的时间间隔，15秒

// jcy烘干定时

int lastButtonState4 = HIGH;
int timeButtonState = HIGH;
int lastTimeButtonState = HIGH;

unsigned long lastDebounceTime4 = 0;
unsigned long debounceDelay4 = 50; // 去抖动延迟


//wifi连接
String ssid; // 声明SSID变量
String password; // 声明密码变量

//http连接web服务器
WebServer server(80);

// setup
// pah
void setup()
{
  Serial.begin(9600);
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

  // jcy风干定时
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(fanPinA, OUTPUT);
  pinMode(fanPinB, OUTPUT);
  pinMode(timeButtonPin2, INPUT_PULLUP);
  Serial.begin(9600);

  // jcy 紫外线
  pinMode(buttonPin3, INPUT_PULLUP); // 设置开关引脚为输入，带上拉电阻
  pinMode(ledpin2, OUTPUT);          // 设置LED引脚为输出
  Serial.begin(9600);

  // pzy烘干
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(ledPin4, OUTPUT);
  pinMode(fanPinA4, OUTPUT);
  pinMode(fanPinB4, OUTPUT);
  Serial.begin(9600);

  // jcy烘干定时
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(ledPin4, OUTPUT);
  pinMode(fanPinA4, OUTPUT);
  pinMode(fanPinB4, OUTPUT);
  pinMode(timeButtonPin, INPUT_PULLUP);
  Serial.begin(9600);

  //zby 阻力重量检测
   Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);

  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(reversePin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT); // 设置蜂鸣器引脚为输出模式

  digitalWrite(buzzerPin, HIGH); // 初始状态为高电平，关闭蜂鸣器

  initialWeight = scale.get_units() - INITIAL_WEIGHT;
  Serial.print("Initial Weight (g): ");
  Serial.println(initialWeight);
  
  //wifi连接
  // 初始化串行通信
Serial.begin(9600);
delay(10);

// 扫描附近的Wi-Fi网络
Serial.println("Start scanning the Wi-Fi network...");
int numNetworks = WiFi.scanNetworks();
Serial.println("Scan complete");

if (numNetworks == 0) {
 Serial.println("No Wi-Fi network found");
} else {
 Serial.print(numNetworks);
 Serial.println("A Wi-Fi network was discovered");
 for (int i = 0; i < numNetworks; ++i) {
// 打印Wi-Fi网络信息
Serial.print(i + 1);
Serial.print(": ");
Serial.print(WiFi.SSID(i));
Serial.print(" (");
Serial.print(WiFi.RSSI(i));
Serial.print(")");
Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
delay(10);
 }
}
//http连接web服务器
// 检查是否有串行数据
  if (Serial.available()) {
    // 读取SSID
    if (ssid.isEmpty()) {
      ssid = Serial.readStringUntil('\n');
      ssid.trim(); // 去除字符串前后的空格和换行符
      Serial.print("Please enter");
      Serial.print(ssid);
      Serial.println("Password of:");
    } else {
      // 读取密码
      password = Serial.readStringUntil('\n');
      password.trim(); // 去除字符串前后的空格和换行符

      // 尝试连接到指定的Wi-Fi网络
      Serial.println("Try to connect to " + ssid);
      WiFi.begin(ssid.c_str(), password.c_str());

      // 等待连接
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }

      // 连接成功
      Serial.println("");
      Serial.println("The Wi-Fi is connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());

      // 清空SSID和密码变量,以便下次重新输入
      ssid = "";
      password = "";
    }
  }

  server.handleClient();
}

// loop

// 全局变量和设置不变

void loop()
{
  // 电机控制
  static unsigned long lastMotorCheck = 0;
  int stepControl = 15;                // 步进控制变量，可以根据需要调整
  if (millis() - lastMotorCheck >= 10) // 每10毫秒检查一次电机状态
  {
    lastMotorCheck = millis();
    if (digitalRead(startPin) == LOW && stepsCount < maxSteps)
     if (digitalRead(startPin) == LOW && stepsCount < maxSteps && isRunning == 1)
    {
      isRunning = true;        // 启动电机
      direction = stepControl; // 设置旋转方向为正向，并控制步进
    }

    if (digitalRead(reversePin) == LOW && stepsCount > 0)
     if (digitalRead(reversePin) == LOW && stepsCount > 0 && isRunning == 1)
    {
      isRunning = true;         // 启动电机
      direction = -stepControl; // 设置旋转方向为反向，并控制步进
    }

    if (digitalRead(stopPin) == LOW)
    {
      isRunning = false; // 停止电机
    }

    if (isRunning)
    {
      stepper.step(direction); // 如果电机正在运行，则按照设置的方向旋转
      stepsCount += direction; // 正向加stepControl，反向减stepControl
      if (stepsCount >= maxSteps || stepsCount <= 0)
      {
        isRunning = false; // 达到最大步数范围时停止电机运行
      }
    }

     if (isRunning == 1)
    {
      stepper.step(direction); // 如果电机正在运行，则按照设置的方向旋转
      stepsCount += direction; // 正向加stepControl，反向减stepControl
      if (stepsCount >= maxSteps || stepsCount <= 0)
      {
        isRunning = 0; // 达到最大步数范围时停止电机运行
      }
    }
  }



  //阻力重力检测功能
  if (scale.wait_ready_timeout(1000)) {
    float reading = scale.get_units() - INITIAL_WEIGHT;
    if (reading < 0) {
      reading = 0; // 确保重量不为负值
    }
    Serial.print("Item Weight (g): ");
    Serial.println(reading);

    if (reading > 2500) {
      digitalWrite(stopPin, HIGH); // 触发停止按钮
      isRunning = 0; // 停止电机
      buzzerState = true; // 切换蜂鸣器状态
      digitalWrite(buzzerPin, LOW); // 根据状态控制蜂鸣器
    } else if (abs(reading - initialWeight) > 20) {
      digitalWrite(stopPin, HIGH); // 触发停止按钮
      isRunning = 0; // 停止电机
      buzzerState = !buzzerState; // 切换蜂鸣器状态
      digitalWrite(buzzerPin, buzzerState ? LOW : HIGH); // 根据状态控制蜂鸣器
      delay(500); // 延时0.5秒
      digitalWrite(buzzerPin, HIGH); // 关闭蜂鸣器
    } else {
      digitalWrite(stopPin, LOW); // 重置停止按钮
      if (initialWeight <= 2500) {
        if (digitalRead(startPin) == LOW && stepsCount < maxSteps)
        {
          isRunning = 1;        // 启动电机
          direction = stepControl; // 设置旋转方向为正向，并控制步进
        }

        if (digitalRead(reversePin) == LOW && stepsCount > 0)
        {
          isRunning = 1;         // 启动电机
          direction = -stepControl; // 设置旋转方向为反向，并控制步进
        }
      }
    }
  } else {
    Serial.println("HX711 not found.");
  }

  // 其他功能
  static unsigned long lastOtherCheck = 0;
  if (millis() - lastOtherCheck >= 10) // 每10毫秒检查一次其他功能
  {
    lastOtherCheck = millis();

    // 照明功能
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
    if (!printed) {
    Serial.print("Potentiometer value: ");
    Serial.println(potValue);
    printed = true; // 将标志设为 true，表示已经打印过
  }

    // 根据LED状态控制LED亮度
    if (ledState)
    {
      analogWrite(ledPin, brightness); // 设置LED亮度
    }
    else
    {
      analogWrite(ledPin, 0); // LED关闭
    }

    // 紫外线功能
    buttonState3 = digitalRead(buttonPin3);

    if (buttonState3 == LOW)
    {
      ledState2 = !ledState2;
      digitalWrite(ledpin2, ledState2);
      Serial.print("Button pressed. LED is now ");
      Serial.println(ledState2 ? "ON" : "OFF");

      if (ledState2)
      {
        lastButtonPressTime = millis(); // 记录LED开启的时间
      }
    }

    // 检查是否到了自动关闭LED的时间
    if (ledState2 && (millis() - lastButtonPressTime >= ledAutoOffInterval))
    {
      ledState2 = false;
      digitalWrite(ledpin2, LOW);
      Serial.println("Auto turning off LED.");
    }

    // 风扇功能
    buttonState2 = digitalRead(buttonPin2); // 风干按钮

    if (buttonState2 == LOW)
    {
      deviceState = !deviceState;
      digitalWrite(fanPinA, deviceState);
      digitalWrite(fanPinB, LOW);
      Serial.print("风干按钮被按下。风扇现在");
      Serial.println(deviceState ? "开启" : "关闭");

      if (deviceState)
      {
        lastButtonPressTime2 = millis(); // 记录风扇开启的时间
      }
    }

    // 检查是否到了自动关闭风扇的时间
    if (deviceState && (millis() - lastButtonPressTime2 >= deviceAutoOffInterval2))
    {
      deviceState = false;
      digitalWrite(fanPinA, LOW);
      digitalWrite(fanPinB, LOW);
      Serial.println("自动关闭风扇。");
    }

    // jcy风干定时
    int reading2 = digitalRead(buttonPin2);
    int timeReading2 = digitalRead(timeButtonPin2);

    // 处理风扇开关按钮
    if (reading2 != lastButtonState2)
    {
      lastDebounceTime2 = millis();
    }

    if ((millis() - lastDebounceTime2) > debounceDelay2)
    {
      if (reading2 != buttonState2)
      {
        buttonState2 = reading2;
        if (buttonState2 == LOW)
        {
          deviceState = !deviceState;
          digitalWrite(fanPinA, deviceState);
          digitalWrite(fanPinB, LOW);
          Serial.print("按钮被按下，风扇现在");
          Serial.println(deviceState ? "开启" : "关闭");

          if (deviceState)
          {
            lastButtonPressTime2 = millis(); // 记录风扇开启的时间
          }
          else
          {
            // 设备关闭后重置自动关闭时间为初始值
            deviceAutoOffInterval2 = 15000;
            digitalWrite(fanPinA, LOW);
            digitalWrite(fanPinB, LOW);
            Serial.println("手动关闭风扇。");
          }
        }
      }
    }

    lastButtonState2 = reading2;

    // 自动关闭风扇功能
    if (deviceState && (millis() - lastButtonPressTime2 >= deviceAutoOffInterval2))
    {
      deviceState = false;
      digitalWrite(fanPinA, LOW);
      digitalWrite(fanPinB, LOW);

      // 自动关闭后重置为初始状态
      deviceAutoOffInterval2 = 15000;
      lastButtonPressTime2 = 0;
      Serial.println("自动关闭风扇。");
    }

    // 处理增加定时时间的按钮
    if (timeReading2 != lastTimeButtonState2)
    {
      lastDebounceTime2 = millis();
    }

    if ((millis() - lastDebounceTime2) > debounceDelay2)
    {
      if (timeReading2 != timeButtonState2)
      {
        timeButtonState2 = timeReading2;
        if (timeButtonState2 == LOW)
        {
          // 增加定时关闭时间，最大不超过 30 秒
          if (deviceAutoOffInterval2 <= 27000) // 限制在 27 秒内增加
          {
            deviceAutoOffInterval2 += 3000;
          }
          else
          {
            deviceAutoOffInterval2 = 30000; // 达到 30 秒后不再增加
          }
          Serial.print("定时关闭时间增加 3 秒，当前定时时间为：");
          Serial.println(deviceAutoOffInterval2);
        }
      }
    }

    lastTimeButtonState2 = timeReading2;

    // 烘干功能
    buttonState4 = digitalRead(buttonPin4); // 烘干按钮

    if (buttonState4 == LOW)
    {
      deviceState4 = !deviceState4;
      digitalWrite(fanPinA4, deviceState4);
      digitalWrite(fanPinB4, LOW);
      digitalWrite(ledPin4, deviceState4); // 控制LED灯的状态与风扇同步
      Serial.print("烘干按钮被按下。风扇和LED灯现在");
      Serial.println(deviceState4 ? "开启" : "关闭");

      if (deviceState4)
      {
        lastButtonPressTime4 = millis(); // 记录风扇和LED灯开启的时间
      }
    }

    // 检查是否到了自动关闭风扇和LED灯的时间
    if (deviceState4 && (millis() - lastButtonPressTime4 >= deviceAutoOffInterval4))
    {
      deviceState4 = false;
      digitalWrite(fanPinA4, LOW);
      digitalWrite(fanPinB4, LOW);
      digitalWrite(ledPin4, LOW); // 自动关闭LED灯
      Serial.println("自动关闭风扇和LED灯。");
    }
  }

  // jcy烘干定时
  int reading4 = digitalRead(buttonPin4);
  int timeReading = digitalRead(timeButtonPin);

  if (reading4 != lastButtonState4)
  {
    lastDebounceTime4 = millis();
  }

  if ((millis() - lastDebounceTime4) > debounceDelay4)
  {
    if (reading4 != buttonState4)
    {
      buttonState4 = reading4;
      if (buttonState4 == LOW)
      {
        deviceState4 = !deviceState4;
        digitalWrite(ledPin4, deviceState4);
        digitalWrite(fanPinA4, deviceState4);
        digitalWrite(fanPinB4, LOW);
        Serial.print("The button is pressed. LED and fan are now both ");
        Serial.println(deviceState4 ? "on" : "off");

        if (deviceState4)
        {
          lastButtonPressTime4 = millis(); // 记录 LED 和风扇开启的时间
        }
        else
        {
          deviceAutoOffInterval4 = 15000;
          digitalWrite(ledPin4, LOW);
          digitalWrite(fanPinA4, LOW);
          digitalWrite(fanPinB4, LOW);
        }
      }
    }
  }

  lastButtonState4 = reading4;

  // 检查是否到了自动关闭 LED 和风扇的时间
  if (deviceState4 && (millis() - lastButtonPressTime4 >= deviceAutoOffInterval4))
  {
    deviceState4 = false;
    digitalWrite(ledPin4, LOW);
    digitalWrite(fanPinA4, LOW);
    digitalWrite(fanPinB4, LOW);

    // 在设备关闭后，重置为初始化状态
    deviceAutoOffInterval4 = 15000;
    Serial.println("Automatically turn off LED and fan");

       Serial.println("Device reset to initial state after auto-off");
  }

  if (timeReading != lastTimeButtonState)
  {
    lastDebounceTime4 = millis();
  }

  if ((millis() - lastDebounceTime4) > debounceDelay4)
  {
    if (timeReading != timeButtonState)
    {
      timeButtonState = timeReading;
      if (timeButtonState == LOW)
      {
        deviceAutoOffInterval4 += 3000;
        if (deviceAutoOffInterval4 > 30000)
        {
          deviceAutoOffInterval4 = 30000;
        }
        Serial.println("The timing time is increased by 3 seconds, and the current timing time is:" + String(deviceAutoOffInterval4));
      }
    }
  }

  lastTimeButtonState = timeReading;
  
  //wifi连接
  // 检查是否有串行数据
if (Serial.available()) {
// 读取SSID
 if (ssid.isEmpty()) {
  ssid = Serial.readStringUntil('\n');
ssid.trim(); // 去除字符串前后的空格和换行符
Serial.print("Please enter");
Serial.print(ssid);
Serial.println("Password of:");
 } else {
// 读取密码
password = Serial.readStringUntil('\n');
password.trim(); // 去除字符串前后的空格和换行符

// 尝试连接到指定的Wi-Fi网络
Serial.println("Try to connect to " + ssid);
WiFi.begin(ssid.c_str(), password.c_str());

// 等待连接
while (WiFi.status() != WL_CONNECTED) {
 delay(500);
Serial.print(".");
}

// 连接成功
Serial.println("");
Serial.println("The Wi-Fi is connected");
Serial.println("IP address: ");
Serial.println(WiFi.localIP());

// 清空SSID和密码变量，以便下次重新输入
ssid = "";
password = "";
 }
}


//http连接web服务器
// 检查是否有串行数据
  if (Serial.available()) {
    // 读取SSID
    if (ssid.isEmpty()) {
      ssid = Serial.readStringUntil('\n');
      ssid.trim(); // 去除字符串前后的空格和换行符
      Serial.print("Please enter");
      Serial.print(ssid);
      Serial.println("Password of:");
    } else {
      // 读取密码
      password = Serial.readStringUntil('\n');
      password.trim(); // 去除字符串前后的空格和换行符

      // 尝试连接到指定的Wi-Fi网络
      Serial.println("Try to connect to " + ssid);
      WiFi.begin(ssid.c_str(), password.c_str());

      // 等待连接
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }

      // 连接成功
      Serial.println("");
      Serial.println("The Wi-Fi is connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());

      // 清空SSID和密码变量,以便下次重新输入
      ssid = "";
      password = "";
    }
  }

  server.handleClient();
}


void handleRoot() {
  server.send(200, "text/plain", "Hello from ESP32!");
}

