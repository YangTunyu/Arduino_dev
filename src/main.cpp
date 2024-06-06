// include部分
#include "Freenove_WS2812_Lib_for_ESP32.h"
#define LEDS_COUNT 8
#define LEDS_PIN 13
#define CHANNEL 0
#include <Arduino.h>
#define MOTOR_INA 23
#define MOTOR_INB 22
#define SWITCH_PIN 13
#define GREEN_LED 27
#define RED_LED 14
// yxr
#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>

// JCY
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// JCY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//zby
#include <MQUnifiedsensor.h>
#define alarmPin 18
// Definitions
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin 13                // Analog input 4 of your arduino
#define type "MQ2"            // MQ2
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ2CleanAir 4.4  // RS / R0 = 4.4 ppm
// #define calibration_button 13 //Pin to calibrate your sensor
// Declare Sensor
MQUnifiedsensor MQ2(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
// Declare Sensor
//MQUnifiedsensor MQ2(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
#define alarmPin 18
#define type "MQ-2"           // MQ4






// JCY
void serial2ISR()
{
  // 检查是否有可用数据从 ESP32 的串口2 接收
  if (Serial2.available())
  {
    byte receivedData[6];
    // 读取 6 个字节到 receivedData 数组中
    Serial2.readBytes(receivedData, 6);

    // 输出接收到的数据
    Serial.print("Received data: ");
    for (int i = 0; i < 6; i++)
    {
      Serial.print(receivedData[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// 其余部分
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

enum Mode
{
  GRADIENT,
  JUMP,
  FLASH,
  RAINBOW,
  BREATH
};
Mode mode = GRADIENT;

enum MotorState
{
  STOP,
  REVERSE,
  FORWARD
};
MotorState motorState = STOP;

bool lastSwitchState = HIGH; // 按钮的上一个状态，初始为未按下
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// yxr
BH1750 lightSensor;
const int ledPin = 16; // LED的控制引脚连接到GPIO 16
const int pirPin = 14; // HC-SR312微型人体感应模块的信号引脚连接到GPIO 14
// 定义引脚
const int analogPin = 36;  // 模拟输入引脚
const int digitalPin = 39; // 数字输入引脚
const int ioPin = 27;      // I/O接口引脚
const int buzzerPin = 13;  // 蜂鸣器引脚

// 定义阈值
const int threshold = 4000;   // 模拟输入阈值
const int alarmThreshold = 1; // 数字输入阈值

// pzy
//  定义红外人体感应模块和LED灯的引脚
const int sensorPin = 14; // 人体感应模块的输出引脚连接到GPIO22
// const int ledPin = 16;     // LED灯连接到GPIO13

int potpin = 15; // 定义模拟ADC接口15
int ledpin = 16; // 定义数字接口16（PWM 输出）
int val = 0;     // 暂存来自传感器的变量数值

// zby
// int ledPin = 16; // 定义LED连接的引脚
int brightness = 0; // LED亮度变量
int fadeAmount = 1; // 亮度变化步长

// setup部分
void setup()
{
  strip.begin();
  strip.setBrightness(20);

  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP); // 使用内部上拉电阻
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // yxr
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

  // pzy
  pinMode(sensorPin, INPUT); // 设置人体感应模块引脚为输入
  pinMode(ledPin, OUTPUT);   // 设置LED灯引脚为输出

  pinMode(ledpin, OUTPUT); // 定义数字接口11 为输出
  Serial.begin(9600);      // 设置波特率为9600
  // 模拟接口自动设置为输入

  // jcy
  Serial.begin(9600);
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  // JCY
  Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  display.display();
  delay(2000);
  display.clearDisplay();

  dht.begin();
  // Initialize device.
  Serial.println(F("DHT11 Unified Sensor Example"));

  dht.temperature().getSensor(&sensor);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Temperature Sensor"));
  display.println(F("------------------------------------"));
  display.print(F("Sensor Type: "));
  display.println(sensor.name);
  display.print(F("Driver Ver:  "));
  display.println(sensor.version);
  display.print(F("Unique ID:   "));
  display.println(sensor.sensor_id);
  display.print(F("Max Value:   "));
  display.print(sensor.max_value);
  display.println(F("°C"));
  display.print(F("Min Value:   "));
  display.print(sensor.min_value);
  display.println(F("°C"));
  display.print(F("Resolution:  "));
  display.print(sensor.resolution);
  display.println(F("°C"));
  display.println(F("------------------------------------"));
  dht.humidity().getSensor(&sensor);
  display.println(F("Humidity Sensor"));
  display.println(F("Sensor Type: "));
  display.println(sensor.name);
  display.print(F("Driver Ver:  "));
  display.println(sensor.version);
  display.print(F("Unique ID:   "));
  display.println(sensor.sensor_id);
  display.print(F("Max Value:   "));
  display.print(sensor.max_value);
  display.println(F("%"));
  display.print(F("Min Value:   "));
  display.print(sensor.min_value);
  display.println(F("%"));
  display.print(F("Resolution:  "));
  display.print(sensor.resolution);
  display.println(F("%"));
  display.println(F("------------------------------------"));
  delayMS = sensor.min_delay / 1000;
  display.display();

  // JCY
  //  启动串口调试输出
  Serial.begin(115200);
  // 初始化 ESP32 的串口2
  Serial2.begin(921600);

  // JCY
  //  启动串口调试输出
  Serial.begin(115200);
  // 初始化 ESP32 的串口2
  Serial2.begin(921600);
  // 配置串口2的RX引脚为中断模式
  pinMode(16, INPUT_PULLUP);

  // zby
  pinMode(ledPin, OUTPUT); // 设置LED引脚为输出模式
  // Init serial port
  Serial.begin(115200);
  // Set math model to calculate the PPM concentration and the value of constants
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(30000000);
  MQ2.setB(-8.308); // Configure the equation to to calculate CH4 concentration

  MQ2.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0))
  {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1)
      ;
  }
  if (calcR0 == 0)
  {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1)
      ;
  }
  /*****************************  MQ CAlibration ********************************************/
  MQ2.serialDebug(true);
  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin, HIGH);

  // Init serial port
Serial.begin(115200);
// Set math model to calculate the PPM concentration and the value of constants
MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
MQ2.setA(30000000);
MQ2.setB(-8.308); // Configure the equation to to calculate CH4 concentration

MQ2.init();

Serial.print("Calibrating please wait.");
//float calcR0 = 0;
for (int i = 1; i <= 10; i++)
{
MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
Serial.print(".");
}
MQ2.setR0(calcR0 / 10);
Serial.println("  done!.");

if (isinf(calcR0))
{
Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
while (1)
;
}
if (calcR0 == 0)
{
Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
while (1)
;
}
/*****************************  MQ CAlibration ********************************************/
MQ2.serialDebug(true);
}

// loop部分
void loop()
{
  switch (mode)
  {
  case GRADIENT:
    for (int j = 0; j < 255; j += 2)
    {
      for (int i = 0; i < LEDS_COUNT; i++)
      {
        strip.setLedColorData(i, strip.Wheel((i * 256 / LEDS_COUNT + j) & 255));
      }
      strip.show();
      delay(10);
    }
    delay(2000); // 2 seconds delay
    mode = JUMP;
    break;
  case JUMP:
    for (int i = 0; i < LEDS_COUNT; i++)
    {
      strip.setLedColorData(i, strip.Wheel(random(0, 255)));
    }
    strip.show();
    delay(2000); // 2 seconds delay
    mode = FLASH;
    break;
  case FLASH:
    for (int i = 0; i < LEDS_COUNT; i++)
    {
      strip.setLedColorData(i, strip.Wheel(random(0, 255)));
    }
    strip.show();
    delay(100);
    for (int i = 0; i < LEDS_COUNT; i++)
    {
      strip.setLedColorData(i, 0);
    }
    strip.show();
    delay(100);
    delay(2000); // 2 seconds delay
    mode = RAINBOW;
    break;
  case RAINBOW:
    for (int j = 0; j < 256; j++)
    {
      for (int i = 0; i < LEDS_COUNT; i++)
      {
        strip.setLedColorData(i, strip.Wheel((i + j) & 255));
      }
      strip.show();
      delay(10);
    }
    delay(2000); // 2 seconds delay
    mode = BREATH;
    break;
  case BREATH:
    for (int j = 0; j < 255; j++)
    {
      for (int i = 0; i < LEDS_COUNT; i++)
      {
        strip.setLedColorData(i, strip.Wheel(j));
      }
      strip.show();
      delay(10);
    }
    for (int j = 255; j > 0; j--)
    {
      for (int i = 0; i < LEDS_COUNT; i++)
      {
        strip.setLedColorData(i, strip.Wheel(j));
      }
      strip.show();
      delay(10);
    }
    delay(2000); // 2 seconds delay
    mode = GRADIENT;
    break;
  }

  bool switchState = digitalRead(SWITCH_PIN);

  if (switchState != lastSwitchState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (switchState == LOW)
    {
      motorState = static_cast<MotorState>((motorState + 1) % 3);
    }
  }

  lastSwitchState = switchState;

  switch (motorState)
  {
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

  // yxr
  //  读取光线强度
  uint16_t lux = lightSensor.readLightLevel();
  Serial.print("Light level (lux): ");
  Serial.println(lux);

  // 读取人体感应模块状态
  int motionDetected = digitalRead(pirPin);
  Serial.print("People Condition: ");
  Serial.println(motionDetected);

  // 当光线暗且有人时，点亮LED
  if (lux < 500 && motionDetected == HIGH)
  {
    digitalWrite(ledPin, HIGH); // 点亮LED
    Serial.print("Light Condition: HIGH ");
  }
  else
  {
    digitalWrite(ledPin, LOW); // 关闭LED
    Serial.print("Light Condition: LOW ");
  }
  delay(1000); // 延迟1秒钟
  // 读取模拟输入
  int sensorValue = analogRead(analogPin);

  // 如果模拟输入值超过阈值，触发告警
  if (sensorValue > threshold)
  {
    Serial.println("Gas leak detected!");
    // 触发蜂鸣器
    digitalWrite(buzzerPin, HIGH); // 将蜂鸣器引脚设置为高电平
    delay(1000);                   // 延迟1秒钟
    digitalWrite(buzzerPin, LOW);  // 将蜂鸣器引脚设置为低电平
  }

  // 读取数字输入
  int digitalValue = digitalRead(digitalPin);

  // 如果数字输入为高电平，表示发生告警
  if (digitalValue == LOW)
  {
    Serial.println("Digital alarm triggered!");
    // 触发蜂鸣器
    // digitalWrite(buzzerPin, HIGH);  // 将蜂鸣器引脚设置为高电平
    // delay(1000);  // 延迟1秒钟
    digitalWrite(buzzerPin, LOW); // 将蜂鸣器引脚设置为低电平
  }
  else
  {
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

  // pzy
  if (digitalRead(sensorPin) == HIGH)
  {
    digitalWrite(ledPin, HIGH); // 如果检测到人体，点亮LED灯
  }
  else
  {
    digitalWrite(ledPin, LOW); // 否则，熄灭LED灯
  }

  val = analogRead(potpin); // 读取传感器的模拟值并赋值给val  读取模拟值范围 0-1023
  Serial.println(val);      // 显示val 变量  用来串口监视

  analogWrite(ledpin, val / 4); // 打开LED 并设置亮度（PWM 输出最大值255）
  delay(10);                    // 延时0.01 秒

  // JCY
  //  Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
  }
  else
  {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
  }
  else
  {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }

  // JCY
  delay(delayMS);
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    display.println(F("Error reading temperature!"));
  }
  else
  {
    display.setCursor(0, 60);
    display.print(F("Temperature: "));
    display.print(event.temperature);
    display.println(F("°C"));
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    display.println(F("Error reading humidity!"));
  }
  else
  {
    display.setCursor(0, 50);
    display.print(F("Humidity: "));
    display.print(event.relative_humidity);
    display.println(F("%"));
  }
  display.display();

  // JCY
  //  发送一串数据到 ESP32 的串口2
  byte sendData[] = {0xF1, 0xF2, 0xF3, 0xF4, 0xFF, 0xF1};
  Serial2.write(sendData, sizeof(sendData));
  // 输出发送的数据
  Serial.print("Sent data: ");
  for (int i = 0; i < sizeof(sendData); i++)
  {
    Serial.print(sendData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  delay(1000);

  // zby
  MQ2.update();
  float smokePPM = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  if (smokePPM > 50)
  {
    Serial.println("Warning: High concentrations of smoke detected");
    digitalWrite(alarmPin, LOW); // 触发警报
  }
  else
  {
    Serial.println("Smoke concentration normal");
    digitalWrite(alarmPin, HIGH); // 关闭警报
  }
  Serial.print("Smoke Concentration: ");
  Serial.print(smokePPM);
  Serial.println(" ppm");
  MQ2.serialDebug(); // Will print the table on the serial port
  delay(4000);
  //  LED逐渐变亮过程
  for (int i = 0; i <= 255; i += fadeAmount)
  {
    analogWrite(ledPin, i);
    delay(100); // 可以调整呼吸灯速度
  }

    MQ2.update();
  //float smokePPM = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  if (smokePPM > 1000)
  {
    Serial.println("Warning: High concentrations of smoke detected");
  }
  MQ2.serialDebug(); // Will print the table on the serial port
  delay(400);


  // LED逐渐变暗
  for (int i = 255; i >= 0; i -= fadeAmount)
  {
    analogWrite(ledPin, i);
    delay(100); // 可以调整呼吸灯速度
  }
}
