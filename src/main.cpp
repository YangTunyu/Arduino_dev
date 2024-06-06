// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 14 // Digital pin connected to the DHT sensor
                  // Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
                  // Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE DHT11 // DHT 11
// #define DHTTYPE DHT22 // DHT 22 (AM2302)
//  #define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setup()
{
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
}

void loop()
{
  // Delay between measurements.
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
}

// 灯光自动控制
#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>

BH1750 lightSensor;
const int ledPin = 16; // LED的控制引脚连接到GPIO 16
const int pirPin = 14; // HC-SR312微型人体感应模块的信号引脚连接到GPIO 14

void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(pirPin, INPUT);

  Wire.begin(21, 22); // 初始化I2C总线，SDA连接到GPIO 21，SCL连接到GPIO 22
  lightSensor.begin();
}

void loop()
{
  // 读取光线强度
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
    Serial.print("Light Condition: LOW");
  }
  delay(1000); // 延迟1秒钟
}

// 获取光照强度信息
/*

Advanced BH1750 library usage example

This example has some comments about advanced usage features.

Connections

  - VCC to 3V3 or 5V
  - GND to GND
  - SCL to SCL (A5 on Arduino Uno, Leonardo, etc or 21 on Mega and Due, on
    esp8266 free selectable)
  - SDA to SDA (A4 on Arduino Uno, Leonardo, etc or 20 on Mega and Due, on
    esp8266 free selectable)
  - ADD to (not connected) or GND

ADD pin is used to set sensor I2C address. If it has voltage greater or equal
to 0.7VCC voltage (e.g. you've connected it to VCC) the sensor address will be
0x5C. In other case (if ADD voltage less than 0.7 * VCC) the sensor address
will be 0x23 (by default).

*/

#include <BH1750.h>
#include <Wire.h>

/*
  BH1750 can be physically configured to use two I2C addresses:
    - 0x23 (most common) (if ADD pin had < 0.7VCC voltage)
    - 0x5C (if ADD pin had > 0.7VCC voltage)

  Library uses 0x23 address as default, but you can define any other address.
  If you had troubles with default value - try to change it to 0x5C.

*/
BH1750 lightMeter(0x23);

void setup()
{

  Serial.begin(9600);

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin(22, 21);
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);

  /*

    BH1750 has six different measurement modes. They are divided in two groups;
    continuous and one-time measurements. In continuous mode, sensor
    continuously measures lightness value. In one-time mode the sensor makes
    only one measurement and then goes into Power Down mode.

    Each mode, has three different precisions:

      - Low Resolution Mode - (4 lx precision, 16ms measurement time)
      - High Resolution Mode - (1 lx precision, 120ms measurement time)
      - High Resolution Mode 2 - (0.5 lx precision, 120ms measurement time)

    By default, the library uses Continuous High Resolution Mode, but you can
    set any other mode, by passing it to BH1750.begin() or BH1750.configure()
    functions.

    [!] Remember, if you use One-Time mode, your sensor will go to Power Down
    mode each time, when it completes a measurement and you've read it.

    Full mode list:

      BH1750_CONTINUOUS_LOW_RES_MODE
      BH1750_CONTINUOUS_HIGH_RES_MODE (default)
      BH1750_CONTINUOUS_HIGH_RES_MODE_2

      BH1750_ONE_TIME_LOW_RES_MODE
      BH1750_ONE_TIME_HIGH_RES_MODE
      BH1750_ONE_TIME_HIGH_RES_MODE_2

  */

  // begin returns a boolean that can be used to detect setup problems.
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
  {
    Serial.println(F("BH1750 Advanced begin"));
  }
  else
  {
    Serial.println(F("Error initialising BH1750"));
  }
}

void loop()
{
  if (lightMeter.measurementReady())
  {
    float lux = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lx");
  }
}

// 七彩氛围灯实现
#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT 8
#define LEDS_PIN 13
#define CHANNEL 0

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

void setup()
{
  strip.begin();
  strip.setBrightness(20);
}

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
}

// 烟雾浓度报警器
#include <MQUnifiedsensor.h>

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
#define alarmPin 18
void setup()
{
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
}

void loop()
{

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
}

// 人体感应自动开灯
#include <Arduino.h>

// 定义红外人体感应模块和LED灯的引脚
const int sensorPin = 22; // 人体感应模块的输出引脚连接到GPIO22
const int ledPin = 13;    // LED灯连接到GPIO13

void setup()
{
  pinMode(sensorPin, INPUT); // 设置人体感应模块引脚为输入
  pinMode(ledPin, OUTPUT);   // 设置LED灯引脚为输出
}

void loop()
{
  if (digitalRead(sensorPin) == HIGH)
  {
    digitalWrite(ledPin, HIGH); // 如果检测到人体，点亮LED灯
  }
  else
  {
    digitalWrite(ledPin, LOW); // 否则，熄灭LED灯
  }
}

// 获取温湿度信息
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 14

#define DHTTYPE DHT11 // DHT 11

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setup()
{
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

  delayMS = sensor.min_delay / 1000;
}

void loop()
{
  // Delay between measurements.
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
}

// 烟雾浓度数据获取
#include <MQUnifiedsensor.h>

// Definitions
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin 13                // Analog input 4 of your arduino
#define type "MQ-2"           // MQ4
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ4CleanAir 4.4  // RS / R0 = 4.4 ppm
// #define calibration_button 13 //Pin to calibrate your sensor

// Declare Sensor
MQUnifiedsensor MQ2(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

void setup()
{
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
    calcR0 += MQ2.calibrate(RatioMQ4CleanAir);
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
}

void loop()
{

  MQ2.update();
  float smokePPM = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  if (smokePPM > 1000)
  {
    Serial.println("Warning: High concentrations of smoke detected");
  }
  MQ2.serialDebug(); // Will print the table on the serial port
  delay(400);
}

// 显示温湿度信息
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

#define DHTPIN 14
#define DHTTYPE DHT11 // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

void setup()
{
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
  Serial.println(F("DHTxx Unified Sensor Example"));
  sensor_t sensor;
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
}

void loop()
{
  delay(delayMS);
  sensors_event_t event;
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
}

// LED无极调光
#include <Arduino.h>

int potpin = 15; // 定义模拟接口15
int ledpin = 13; // 定义数字接口13（PWM 输出）
int val = 0;     // 暂存来自传感器的变量数值
void setup()
{
  pinMode(ledpin, OUTPUT); // 定义数字接口11 为输出
  Serial.begin(9600);      // 设置波特率为9600
  // 模拟接口自动设置为输入
}
void loop()
{
  val = analogRead(potpin); // 读取传感器的模拟值并赋值给val  读取模拟值范围 0-1023
  Serial.println(val);      // 显示val 变量  用来串口监视

  analogWrite(ledpin, val / 4); // 打开LED 并设置亮度（PWM 输出最大值255）
  delay(10);                    // 延时0.01 秒
}

// ESP32双机双工通信
#include <Arduino.h>

void setup()
{
  // 启动串口调试输出
  Serial.begin(115200);
  // 初始化 ESP32 的串口2
  Serial2.begin(250000);
}

void loop()
{
  // 发送一串数据到 ESP32 的串口2
  byte sendData[] = {0xF1, 0xF2, 0xF3, 0xF4, 0xFF, 0xF1};
  Serial.println("88");
  Serial2.write(sendData, sizeof(sendData));
  // 输出发送的数据
  Serial.print("Sent data:111 ");
  for (int i = 0; i < sizeof(sendData); i++)
  {
    Serial.print(sendData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  delay(1000);
}

#include <Arduino.h>

// 中断处理函数
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

void setup()
{
  // 启动串口调试输出
  Serial.begin(115200);
  // 初始化 ESP32 的串口2
  Serial2.begin(250000);
  // 配置串口2的RX引脚为中断模式
  pinMode(16, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(16), serial2ISR, FALLING);
}

void loop()
{
  // do nothing
}


//可燃气体泄露告警
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
}