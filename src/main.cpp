//include部分
//pah上升、下降以及暂停
#include "Stepper_28BYJ_48.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>












//引脚定义
int startPin = 13; // 连接到启动按钮的引脚
int stopPin = 23; // 连接到停止按钮的引脚
int reversePin = 22; // 连接到反向旋转按钮的引脚













//其余部分
//pah
Stepper_28BYJ_48 stepper(14, 27, 26, 25);

bool isRunning = false; // 用于跟踪电机的状态
int direction = 1; // 用于控制电机的旋转方向
int stepsCount = 0; // 用于计算电机转动的步数

const int stepsPerRevolution = 512; // 一圈步进电机（28BYJ-48）有512步
const int revolutions = 2; // 每次转动二圈

const int maxSteps = 2 * stepsPerRevolution; // 总步数设置为两圈的步数















//setup
//pah
void setup() {
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(reversePin, INPUT_PULLUP);
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

}
