//pah上升、下降以及暂停
#include "Stepper_28BYJ_48.h"

int startPin = 13; // 连接到启动按钮的引脚
int stopPin = 23; // 连接到停止按钮的引脚
int reversePin = 22; // 连接到反向旋转按钮的引脚

Stepper_28BYJ_48 stepper(14, 27, 26, 25);

bool isRunning = false; // 用于跟踪电机的状态
int direction = 1; // 用于控制电机的旋转方向

void setup() {
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(reversePin, INPUT_PULLUP);
}

void loop() {
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
  }
}
