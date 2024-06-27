#include <WiFi.h>
#include <WebServer.h>
#include "Stepper_28BYJ_48.h"
#include "Wire.h"
#include "HX711.h"
#include <Arduino.h>

// Wi-Fi 网络信息
const char* ssid = "LengDian";
const char* password = "22732100QT";

// 创建 Web 服务器对象，端口号为80
WebServer server(80);

// 定义引脚
const int ledPin = 3; // LED引脚，根据实际情况调整
const int ledButtonPin = 5; // 物理开关引脚用于LED控制
const int uvPin = 2;  // UV灯引脚，根据实际情况调整
const int fanPinA = 1; // 风扇引脚A
const int fanPinB = 4; // 风扇引脚B
const int dryLedPin = 26;  // 新的LED灯引脚用于烘干功能
const int uvButtonPin = 16; // 物理按钮引脚用于UV控制
const int fanButtonPin = 0; // 物理按钮引脚用于风扇控制
const int dryControlButtonPin = 25; // 物理按钮引脚用于同时控制风扇和LED灯
const int fanDurationButtonPin = 19; // 物理按钮引脚用于增加风扇运行时间
const int dryDurationButtonPin = 18; // 物理按钮引脚用于同时增加风扇和LED灯的运行时间
const int startPin = 13; // 连接到启动按钮的引脚
const int stopPin = 23; // 连接到停止按钮的引脚
const int reversePin = 22; // 连接到反向旋转按钮的引脚
const int buzzerPin = 21; // 连接到蜂鸣器的引脚

// 初始化步进电机
Stepper_28BYJ_48 stepper(14, 27, 33, 32);
const int stepsPerRevolution = 512; // 一圈步进电机（28BYJ-48）有512步
int stepControl = 15; // 步进控制变量，每次移动15步

// 状态变量
bool ledState = false; // LED初始状态为关闭
bool uvState = false;  // UV灯初始状态为关闭
bool fanState = false; // 风扇初始状态为关闭
bool dryLedState = false; // 新的LED灯初始状态为关闭
bool isMotorRunning = false; // 电机运行状态
bool canMoveDown = false; // 电机是否可以下降，初始状态为false
bool canMoveUp = true; // 电机是否可以上升，初始状态为true
bool buzzerState = false; // 蜂鸣器状态
bool isPaused = false; // 电机暂停状态
bool dryingFanState = false; // 烘干时的风扇状态
int direction = 1; // 电机方向
int stepsCount = 0; // 电机步数计数

// 倒计时变量
unsigned long uvStartTime = 0;
unsigned long fanStartTime = 0;
unsigned long dryStartTime = 0;
const unsigned long defaultDuration = 15000; // 默认15秒
unsigned long fanDuration = defaultDuration; // 风扇默认倒计时15秒
unsigned long dryDuration = defaultDuration; // 新的LED灯默认倒计时15秒
unsigned long uvDuration = defaultDuration; // UV灯默认倒计时15秒

const int LOADCELL_DOUT_PIN = 15;
const int LOADCELL_SCK_PIN = 17;
const float CALIBRATION_FACTOR = 833.33; // 100000 / 120 = 833.33
const float INITIAL_WEIGHT = 100.0; // 初始重量

HX711 scale;
float initialWeight = 0.0;

void resetDurations() {
  fanDuration = defaultDuration;
  dryDuration = defaultDuration;
  uvDuration = defaultDuration;
}

void handleLightOn() {
  digitalWrite(ledPin, HIGH);
  ledState = true;
  server.send(200, "text/plain", "LED is ON");
}

void handleLightOff() {
  digitalWrite(ledPin, LOW);
  ledState = false;
  server.send(200, "text/plain", "LED is OFF");
}

void handleUVOn() {
  digitalWrite(uvPin, HIGH);
  uvState = true;
  uvStartTime = millis(); // 记录启动时间
  server.send(200, "text/plain", "UV is ON");
}

void handleUVOff() {
  digitalWrite(uvPin, LOW);
  uvState = false;
  server.send(200, "text/plain", "UV is OFF");
}

void handleWindOn() {
  digitalWrite(fanPinA, HIGH);
  digitalWrite(fanPinB, LOW);
  fanState = true;
  fanStartTime = millis(); // 记录启动时间
  server.send(200, "text/plain", "Wind is ON");
  Serial.println("Fan is ON"); // 添加调试信息
  Serial.println("FanPinA: HIGH");
  Serial.println("FanPinB: LOW");
}

void handleWindOff() {
  fanState = false;
  if (!dryingFanState) {
    digitalWrite(fanPinA, LOW);
    digitalWrite(fanPinB, LOW);
  }
  server.send(200, "text/plain", "Wind is OFF");
  Serial.println("Fan is OFF"); // 添加调试信息
  Serial.println("FanPinA: LOW");
  Serial.println("FanPinB: LOW");
}

void handleDryOn() {
  dryingFanState = true;
  digitalWrite(fanPinA, HIGH);
  digitalWrite(fanPinB, LOW);
  digitalWrite(dryLedPin, HIGH);
  dryLedState = true;
  dryStartTime = millis(); // 记录启动时间
  server.send(200, "text/plain", "Dry is ON");
}

void handleDryOff() {
  dryingFanState = false;
  digitalWrite(dryLedPin, LOW);
  dryLedState = false;
  if (!fanState) {
    digitalWrite(fanPinA, LOW);
    digitalWrite(fanPinB, LOW);
  }
  server.send(200, "text/plain", "Dry is OFF");
}

void handleStopAll() {
  handleUVOff(); // 关闭UV灯
  handleWindOff(); // 关闭风扇
  handleDryOff(); // 关闭新的LED灯
  // 停止所有操作
  server.send(200, "text/plain", "All operations stopped");
  isMotorRunning = false; // 停止电机
  isPaused = false; // 重置暂停状态
}

void handlePauseMotor() {
  isMotorRunning = false; // 只暂停电机，不影响其他功能
  server.send(200, "text/plain", "Motor paused");
}

void handleUp() {
  if (canMoveUp && !isPaused && stepsCount < stepsPerRevolution * 4) {
    isMotorRunning = true;
    direction = stepControl; // 设置旋转方向为正向
    server.send(200, "text/plain", "Up");
  } else {
    server.send(200, "text/plain", "Cannot move up further");
  }
}

void handleDown() {
  if (canMoveDown && !isPaused && stepsCount > 0) {
    isMotorRunning = true;
    direction = -stepControl; // 设置旋转方向为反向
    server.send(200, "text/plain", "Down");
  } else {
    server.send(200, "text/plain", "Cannot move down further");
  }
}

void handleSetFanDuration() {
  if (server.hasArg("duration")) {
    fanDuration = server.arg("duration").toInt() * 1000; // 设置新的风扇持续时间
    fanStartTime = millis(); // 更新启动时间
    server.send(200, "text/plain", "Fan duration set to " + String(fanDuration / 1000) + " seconds");
  } else {
    server.send(400, "text/plain", "Missing duration parameter");
  }
}

void handleSetDryDuration() {
  if (server.hasArg("duration")) {
    dryDuration = server.arg("duration").toInt() * 1000; // 设置新的LED灯持续时间
    dryStartTime = millis(); // 更新启动时间
    server.send(200, "text/plain", "Dry duration set to " + String(dryDuration / 1000) + " seconds");
  } else {
    server.send(400, "text/plain", "Missing duration parameter");
  }
}

void setup() {
  // 初始化串行通信
  Serial.begin(115200);
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

  // 配置引脚
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // 确保 LED 初始状态为关闭

  pinMode(uvPin, OUTPUT);
  digitalWrite(uvPin, LOW); // 确保 UV灯初始状态为关闭

  pinMode(fanPinA, OUTPUT);
  pinMode(fanPinB, OUTPUT);
  digitalWrite(fanPinA, LOW); // 确保风扇初始状态为关闭
  digitalWrite(fanPinB, LOW); // 确保风扇初始状态为关闭

  pinMode(dryLedPin, OUTPUT);
  digitalWrite(dryLedPin, LOW); // 确保新的LED灯初始状态为关闭

  pinMode(ledButtonPin, INPUT_PULLUP); // 配置LED按钮引脚
  pinMode(uvButtonPin, INPUT_PULLUP); // 配置UV按钮引脚
  pinMode(fanButtonPin, INPUT_PULLUP); // 配置风扇按钮引脚
  pinMode(dryControlButtonPin, INPUT_PULLUP); // 配置同时控制风扇和LED灯的按钮引脚
  pinMode(fanDurationButtonPin, INPUT_PULLUP); // 配置增加风扇运行时间的按钮引脚
  pinMode(dryDurationButtonPin, INPUT_PULLUP); // 配置同时增加风扇和LED灯的运行时间的按钮引脚

  // 连接到 Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // 启动服务器并指定处理函数
  server.on("/light_on", handleLightOn);
  server.on("/light_off", handleLightOff);
  server.on("/uv_on", handleUVOn);
  server.on("/uv_off", handleUVOff);
  server.on("/wind_on", handleWindOn);
  server.on("/wind_off", handleWindOff);
  server.on("/dry_on", handleDryOn); 
  server.on("/dry_off", handleDryOff); 
  server.on("/stop_all", handleStopAll);
  server.on("/up", handleUp);
  server.on("/down", handleDown);
  server.on("/set_fan_duration", handleSetFanDuration); 
  server.on("/set_dry_duration", handleSetDryDuration); 
  server.on("/pause_motor", handlePauseMotor); // 添加暂停电机的处理函数

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // 检查LED物理按钮
  if (digitalRead(ledButtonPin) == LOW) {
    delay(50); // 去抖动
    if (digitalRead(ledButtonPin) == LOW) {
      if (ledState) {
        handleLightOff(); // 如果LED打开则关闭
      } else {
        handleLightOn(); // 如果LED关闭则打开
      }
      while (digitalRead(ledButtonPin) == LOW) {
        delay(10); // 等待按钮释放
      }
    }
  }
  
  // 处理客户端请求
  server.handleClient();

  // 电机控制逻辑
  static unsigned long lastMotorCheck = 0;
  if (millis() - lastMotorCheck >= 10) { // 每10毫秒检查一次电机状态
    lastMotorCheck = millis();
    if (isMotorRunning) {
      stepper.step(direction); // 按照设置的方向旋转
      stepsCount += direction;
      // 更新可移动状态
      if (stepsCount >= stepsPerRevolution * 4) {
        canMoveUp = false;
        canMoveDown = true;
        isMotorRunning = false; // 达到最大步数范围时停止电机运行
      } else if (stepsCount <= 0) {
        canMoveDown = false;
        canMoveUp = true;
        isMotorRunning = false; // 达到最小步数范围时停止电机运行
      } else {
        canMoveUp = true;
        canMoveDown = true;
      }
    }
  }

  // 检查UV灯倒计时
  if (uvState && (millis() - uvStartTime >= uvDuration)) {
    handleUVOff(); // 倒计时结束后关闭UV灯
  }

  // 检查风扇倒计时
  if (fanState && (millis() - fanStartTime >= fanDuration)) {
    handleWindOff(); // 关闭风扇
    fanDuration = defaultDuration;// 恢复默认风扇持续时间
  }

  // 检查新的LED灯倒计时
  if (dryLedState && (millis() - dryStartTime >= dryDuration)) {
    handleDryOff(); // 关闭新的LED灯和风扇
  }

  // 检查UV物理按钮
  if (digitalRead(uvButtonPin) == LOW) {
    delay(50); // 去抖动
    if (digitalRead(uvButtonPin) == LOW) {
      if (uvState) {
        handleUVOff(); // 如果UV灯打开则关闭
      } else {
        handleUVOn(); // 如果UV灯关闭则打开
      }
      while (digitalRead(uvButtonPin) == LOW) {
        delay(10); // 等待按钮释放
      }
    }
  }

  // 检查风扇物理按钮
  if (digitalRead(fanButtonPin) == LOW) {
    delay(50); // 去抖动
    if (digitalRead(fanButtonPin) == LOW) {
      if (fanState) {
        handleWindOff(); // 如果风扇打开则关闭
        fanDuration = defaultDuration;// 恢复默认风扇持续时间
      } else {
        handleWindOn(); // 如果风扇关闭则打开
      }
      while (digitalRead(fanButtonPin) == LOW) {
        delay(10); // 等待按钮释放
      }
    }
  }

  // 检查同时控制风扇和LED灯的物理按钮
  if (digitalRead(dryControlButtonPin) == LOW) {
    delay(50); // 去抖动
    if (digitalRead(dryControlButtonPin) == LOW) {
      if (dryLedState) {
        handleDryOff(); // 关闭LED灯和风扇
        dryDuration = defaultDuration;// 恢复默认风扇和LED灯持续时间
      } else {
        handleDryOn(); // 打开LED灯和风扇
      }
      while (digitalRead(dryControlButtonPin) == LOW) {
        delay(10); // 等待按钮释放
      }
    }
  }

  // 检查增加风扇运行时间的物理按钮
  if (digitalRead(fanDurationButtonPin) == LOW) {
    delay(50); // 去抖动
    if (digitalRead(fanDurationButtonPin) == LOW) {
      if (fanDuration <= 27000) { // 上限30秒
        fanDuration += 3000; // 每次增加3秒
      }
      Serial.println("Fan duration increased to " + String(fanDuration / 1000) + " seconds");
      while (digitalRead(fanDurationButtonPin) == LOW) {
        delay(10); // 等待按钮释放
      }
    }
  }

  // 检查同时增加风扇和LED灯运行时间的物理按钮
  if (digitalRead(dryDurationButtonPin) == LOW) {
    delay(50); // 去抖动
    if (digitalRead(dryDurationButtonPin) == LOW) {
      if (dryDuration <= 27000) { // 上限30秒
        dryDuration += 3000; // 每次增加3秒
        fanDuration = dryDuration; // 同步风扇持续时间
      }
      Serial.println("Fan and Dry LED duration increased to " + String(dryDuration / 1000) + " seconds");
      while (digitalRead(dryDurationButtonPin) == LOW) {
        delay(10); // 等待按钮释放
      }
    }
  }

  // 监控重量变化并进行处理
  if (scale.wait_ready_timeout(1000)) {
    float reading = scale.get_units() - INITIAL_WEIGHT;
    if (reading < 0) {
      reading = 0; // 确保重量不为负值
    }
    Serial.print("Item Weight (g): ");
    Serial.println(reading);

    if (reading > 2500) {
      isMotorRunning = false; // 停止电机
      buzzerState = true; // 切换蜂鸣器状态
      digitalWrite(buzzerPin, LOW); // 根据状态控制蜂鸣器
    } else if (abs(reading - initialWeight) > 20) {
      isMotorRunning = false; // 停止电机
      buzzerState = !buzzerState; // 切换蜂鸣器状态
      digitalWrite(buzzerPin, buzzerState ? LOW : HIGH); // 根据状态控制蜂鸣器
      delay(500); // 延时0.5秒
      digitalWrite(buzzerPin, HIGH); // 关闭蜂鸣器
    } else {
      if (initialWeight <= 2500) {
        if (digitalRead(startPin) == LOW && stepsCount < stepsPerRevolution * 4) {
          isMotorRunning = true; // 启动电机
          direction = stepControl; // 设置旋转方向为正向，并控制步进
        }

        if (digitalRead(reversePin) == LOW && stepsCount > 0) {
          isMotorRunning = true; // 启动电机
          direction = -stepControl; // 设置旋转方向为反向，并控制步进
        }
      }
    }
  } else {
    Serial.println("HX711 not found.");
  }

  // 检查停止按钮
  if (digitalRead(stopPin) == LOW) {
    isMotorRunning = false; // 停止电机
    digitalWrite(buzzerPin, HIGH); // 高电平关闭蜂鸣器
    buzzerState = false; // 关闭蜂鸣器
    initialWeight = scale.get_units() - INITIAL_WEIGHT; // 更新初始重量
  }
}
