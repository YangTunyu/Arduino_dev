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

//灯光自动控制
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
    Serial.print("Light Condition: LOW");
  }
  delay(1000); // 延迟1秒钟
}


//七彩氛围灯实现
#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT  8
#define LEDS_PIN    13
#define CHANNEL     0

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

enum Mode { GRADIENT, JUMP, FLASH, RAINBOW, BREATH };
Mode mode = GRADIENT;

void setup() {
  strip.begin();
  strip.setBrightness(20);  
}

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
}

//烟雾浓度报警器
#include <MQUnifiedsensor.h>

//Definitions
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin 13 //Analog input 4 of your arduino
#define type "MQ2" //MQ2
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ2CleanAir 4.4  //RS / R0 = 4.4 ppm 
//#define calibration_button 13 //Pin to calibrate your sensor

//Declare Sensor
MQUnifiedsensor MQ2(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
#define alarmPin 18
void setup() {
  //Init serial port
  Serial.begin(115200);
  //Set math model to calculate the PPM concentration and the value of constants
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(30000000); MQ2.setB(-8.308); // Configure the equation to to calculate CH4 concentration

  MQ2.init(); 
 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  MQ2.serialDebug(true);
  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin,HIGH);
}

  void loop() {

  MQ2.update();
  float smokePPM = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  if(smokePPM > 50  ) {Serial.println("Warning: High concentrations of smoke detected");
    digitalWrite(alarmPin, LOW); // 触发警报
 } else {
    Serial.println("Smoke concentration normal");
    digitalWrite(alarmPin, HIGH); // 关闭警报
 }
 Serial.print("Smoke Concentration: "); 
 Serial.print(smokePPM); 
 Serial.println(" ppm");
  MQ2.serialDebug(); // Will print the table on the serial port
  delay(4000);  
}




