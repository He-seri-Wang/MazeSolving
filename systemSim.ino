#include <math.h>

// =====================================================
// 红外传感器
// =====================================================
const int SENSOR_PIN = A1;

const float ADC_REF_VOLTAGE = 5.0;
const int ADC_MAX = 1023;

// =====================================================
// 侧边按钮
// HIGH = 撞到
// LOW  = 没撞到
// =====================================================
const int LEFT_BUMPER_PIN = 4;
const int RIGHT_BUMPER_PIN = 2;

bool leftBumperHit = false;
bool rightBumperHit = false;
bool sideBumperHit = false;

// =====================================================
// H 桥引脚
// D7, D8   = 前后电机
// D12, D13 = 左右电机
// =====================================================
const int FB_MOTOR_IN1 = 7;
const int FB_MOTOR_IN2 = 8;

const int LR_MOTOR_IN1 = 12;
const int LR_MOTOR_IN2 = 13;

// =====================================================
// 开机电机测试
// 如果不想测试，改成 false
// =====================================================
const bool ENABLE_MOTOR_TEST = true;
const unsigned long MOTOR_TEST_TIME_MS = 800;

// =====================================================
// 五点标定表
// 电压从大到小，距离从小到大
// =====================================================
const int TABLE_SIZE = 5;

float calibVoltage[TABLE_SIZE] = {
  4.55,
  3.80,
  3.30,
  2.85,
  2.55
};

float calibDistance[TABLE_SIZE] = {
  1.0,
  2.5,
  4.0,
  8.0,
  15.0
};

// =====================================================
// 滤波参数
// =====================================================
const int SAMPLE_COUNT = 7;
const int HISTORY_SIZE = 9;

float voltageHistory[HISTORY_SIZE];
int historyIndex = 0;
bool historyFilled = false;

const float EMA_ALPHA = 0.3;
const float MAX_STEP_VOLTAGE = 0.4;

// =====================================================
// 红外状态
// =====================================================
float rawVoltage = 0.0;
float limitedVoltage = 0.0;

float gapFilteredVoltage = 0.0;
float stopFilteredVoltage = 0.0;

float gapDistance = 15.0;
float stopDistance = 15.0;

bool filterReady = false;

// =====================================================
// 前方判断
// =====================================================
const float BIG_GAP_DISTANCE_CM = 10.0;
const float OBSTACLE_STOP_DISTANCE_CM = 3.5;

// 障碍消失后，距离大于这个值才恢复前进
const float CLEAR_DISTANCE_CM = 6.0;

const int GAP_CONFIRM_COUNT = 8;
const int STOP_CONFIRM_COUNT = 3;
const int CLEAR_CONFIRM_COUNT = 5;

int gapCount = 0;
int stopCount = 0;
int clearCount = 0;

bool bigGapDetected = false;
bool obstacleDetected = false;
bool frontClearDetected = false;

// =====================================================
// comb-shaped 墙面处理
// =====================================================
float minStopDistanceWindow = 999.0;
unsigned long lastWindowReset = 0;
const unsigned long WINDOW_TIME_MS = 120;

// =====================================================
// 定时
// =====================================================
unsigned long lastSensorTime = 0;
const unsigned long SENSOR_INTERVAL_MS = 15;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 200;

// =====================================================
// 机器人状态
// =====================================================
enum RobotState {
  STATE_SIDE_MOVE,
  STATE_FORWARD_MOVE,
  STATE_WAIT_CLEAR
};

RobotState robotState = STATE_SIDE_MOVE;

// 1  = 向右
// -1 = 向左
int sideDirection = 1;

// =====================================================
// 工具函数
// =====================================================
float adcToVoltage(int adc) {
  return adc * ADC_REF_VOLTAGE / ADC_MAX;
}

void sortFloatArray(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        float t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }
}

float readMedianVoltage() {
  float samples[SAMPLE_COUNT];

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    samples[i] = adcToVoltage(analogRead(SENSOR_PIN));
  }

  sortFloatArray(samples, SAMPLE_COUNT);
  return samples[SAMPLE_COUNT / 2];
}

float applyLimiter(float newV, float oldV) {
  float diff = newV - oldV;

  if (diff > MAX_STEP_VOLTAGE) return oldV + MAX_STEP_VOLTAGE;
  if (diff < -MAX_STEP_VOLTAGE) return oldV - MAX_STEP_VOLTAGE;

  return newV;
}

void addHistory(float v) {
  voltageHistory[historyIndex] = v;
  historyIndex++;

  if (historyIndex >= HISTORY_SIZE) {
    historyIndex = 0;
    historyFilled = true;
  }
}

int getHistoryCount() {
  return historyFilled ? HISTORY_SIZE : historyIndex;
}

float getPercentile(float p) {
  int count = getHistoryCount();

  if (count == 0) return limitedVoltage;

  float temp[HISTORY_SIZE];

  for (int i = 0; i < count; i++) {
    temp[i] = voltageHistory[i];
  }

  sortFloatArray(temp, count);

  int idx = (int)((count - 1) * p);

  if (idx < 0) idx = 0;
  if (idx >= count) idx = count - 1;

  return temp[idx];
}

float voltageToDistance(float v) {
  if (v >= calibVoltage[0]) {
    return calibDistance[0];
  }

  if (v <= calibVoltage[TABLE_SIZE - 1]) {
    return calibDistance[TABLE_SIZE - 1];
  }

  for (int i = 0; i < TABLE_SIZE - 1; i++) {
    float v1 = calibVoltage[i];
    float v2 = calibVoltage[i + 1];

    if (v <= v1 && v >= v2) {
      float d1 = calibDistance[i];
      float d2 = calibDistance[i + 1];

      float ratio = (v - v2) / (v1 - v2);

      return d2 + ratio * (d1 - d2);
    }
  }

  return calibDistance[TABLE_SIZE - 1];
}

// =====================================================
// 传感器更新
// =====================================================
void updateSensor() {
  unsigned long now = millis();

  if (now - lastSensorTime < SENSOR_INTERVAL_MS) {
    return;
  }

  lastSensorTime = now;

  rawVoltage = readMedianVoltage();

  if (!filterReady) {
    limitedVoltage = rawVoltage;
  } else {
    limitedVoltage = applyLimiter(rawVoltage, limitedVoltage);
  }

  addHistory(limitedVoltage);

  // gap 用中位数，避免小洞误判
  float gapVoltageCandidate = getPercentile(0.50);

  // stop 偏向近距离，更安全
  float stopVoltageCandidate = getPercentile(0.70);

  if (!filterReady) {
    gapFilteredVoltage = gapVoltageCandidate;
    stopFilteredVoltage = stopVoltageCandidate;
    filterReady = true;
  } else {
    gapFilteredVoltage =
      EMA_ALPHA * gapVoltageCandidate +
      (1.0 - EMA_ALPHA) * gapFilteredVoltage;

    stopFilteredVoltage =
      EMA_ALPHA * stopVoltageCandidate +
      (1.0 - EMA_ALPHA) * stopFilteredVoltage;
  }

  gapDistance = voltageToDistance(gapFilteredVoltage);

  float currentStopDistance = voltageToDistance(stopFilteredVoltage);

  // comb-shaped：停止判断使用短窗口最小值
  if (currentStopDistance < minStopDistanceWindow) {
    minStopDistanceWindow = currentStopDistance;
  }

  if (millis() - lastWindowReset > WINDOW_TIME_MS) {
    lastWindowReset = millis();

    if (minStopDistanceWindow < currentStopDistance) {
      currentStopDistance = minStopDistanceWindow;
    }

    minStopDistanceWindow = 999.0;
  }

  stopDistance = currentStopDistance;

  // 大空隙判断
  if (gapDistance >= BIG_GAP_DISTANCE_CM) {
    gapCount++;
  } else {
    gapCount = 0;
  }

  bigGapDetected = (gapCount >= GAP_CONFIRM_COUNT);

  // 障碍判断
  if (stopDistance <= OBSTACLE_STOP_DISTANCE_CM) {
    stopCount++;
  } else {
    stopCount = 0;
  }

  obstacleDetected = (stopCount >= STOP_CONFIRM_COUNT);

  // 障碍清除判断
  if (stopDistance >= CLEAR_DISTANCE_CM) {
    clearCount++;
  } else {
    clearCount = 0;
  }

  frontClearDetected = (clearCount >= CLEAR_CONFIRM_COUNT);
}

// =====================================================
// 按钮更新
// =====================================================
void updateBumpers() {
  leftBumperHit = (digitalRead(LEFT_BUMPER_PIN) == HIGH);
  rightBumperHit = (digitalRead(RIGHT_BUMPER_PIN) == HIGH);
  sideBumperHit = leftBumperHit || rightBumperHit;
}

// =====================================================
// H 桥控制
// =====================================================
void setHBridge(int in1, int in2, int direction) {
  if (direction > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (direction < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void stopForwardBackMotor() {
  setHBridge(FB_MOTOR_IN1, FB_MOTOR_IN2, 0);
}

void stopLeftRightMotor() {
  setHBridge(LR_MOTOR_IN1, LR_MOTOR_IN2, 0);
}

void stopAllMotors() {
  stopForwardBackMotor();
  stopLeftRightMotor();
}

void moveForward() {
  setHBridge(FB_MOTOR_IN1, FB_MOTOR_IN2, 1);
}

void moveBackward() {
  setHBridge(FB_MOTOR_IN1, FB_MOTOR_IN2, -1);
}

void moveRight() {
  setHBridge(LR_MOTOR_IN1, LR_MOTOR_IN2, 1);
}

void moveLeft() {
  setHBridge(LR_MOTOR_IN1, LR_MOTOR_IN2, -1);
}

void moveSideByDirection() {
  if (sideDirection > 0) {
    moveRight();
  } else {
    moveLeft();
  }
}

// =====================================================
// 对外接口
// =====================================================
int getForwardSignal() {
  return bigGapDetected ? 1 : 0;
}

int getObstacleSignal() {
  return obstacleDetected ? 1 : 0;
}

int getFrontClearSignal() {
  return frontClearDetected ? 1 : 0;
}

int getLeftBumperSignal() {
  return leftBumperHit ? 1 : 0;
}

int getRightBumperSignal() {
  return rightBumperHit ? 1 : 0;
}

int getSideBumperSignal() {
  return sideBumperHit ? 1 : 0;
}

// =====================================================
// 开机电机测试
// =====================================================
void motorSelfTest() {
  if (!ENABLE_MOTOR_TEST) return;

  Serial.println("Motor test: forward/back motor forward");
  moveForward();
  delay(MOTOR_TEST_TIME_MS);
  stopAllMotors();
  delay(300);

  Serial.println("Motor test: forward/back motor backward");
  moveBackward();
  delay(MOTOR_TEST_TIME_MS);
  stopAllMotors();
  delay(300);

  Serial.println("Motor test: left/right motor right");
  moveRight();
  delay(MOTOR_TEST_TIME_MS);
  stopAllMotors();
  delay(300);

  Serial.println("Motor test: left/right motor left");
  moveLeft();
  delay(MOTOR_TEST_TIME_MS);
  stopAllMotors();
  delay(300);

  Serial.println("Motor test done.");
}

// =====================================================
// 状态机
// =====================================================
/*
void updateRobotControl() {
  switch (robotState) {

    case STATE_SIDE_MOVE:
      // 横向扫描时，前后电机停止
      stopForwardBackMotor();

      // 侧边撞到只反转，不停止
      if (getRightBumperSignal() == 1) {
        sideDirection = -1;
      }

      if (getLeftBumperSignal() == 1) {
        sideDirection = 1;
      }

      moveSideByDirection();

      // 发现大空隙，切换前进
      if (getForwardSignal() == 1) {
        stopLeftRightMotor();

        stopCount = 0;
        obstacleDetected = false;

        robotState = STATE_FORWARD_MOVE;
      }

      break;

    case STATE_FORWARD_MOVE:
      // 前进时，左右电机停止
      stopLeftRightMotor();

      // 前方障碍才停止
      if (getObstacleSignal() == 1) {
        stopAllMotors();
        clearCount = 0;
        frontClearDetected = false;
        robotState = STATE_WAIT_CLEAR;
      } else {
        moveForward();
      }

      break;

    case STATE_WAIT_CLEAR:
      // 等待障碍消失
      stopAllMotors();

      // 如果前方恢复到足够远，则继续前进
      if (getFrontClearSignal() == 1) {
        stopCount = 0;
        obstacleDetected = false;
        robotState = STATE_FORWARD_MOVE;
      }

      // 如果你希望障碍消失后回到横向扫描，而不是继续前进，
      // 把上一行改成：
      // robotState = STATE_SIDE_MOVE;

      break;
  }
}
*/
void updateRobotControl() {
  switch (robotState) {

    case STATE_SIDE_MOVE:
      // 横向扫描时，前后电机停止
      stopForwardBackMotor();

      // 侧边撞到只反转，不停止
      if (getRightBumperSignal() == 1) {
        sideDirection = -1;
      }

      if (getLeftBumperSignal() == 1) {
        sideDirection = 1;
      }

      // 左右移动
      moveSideByDirection();

      // 发现大空隙，切换前进
      if (getForwardSignal() == 1) {
        stopLeftRightMotor();

        stopCount = 0;
        obstacleDetected = false;

        robotState = STATE_FORWARD_MOVE;
      }

      break;

    case STATE_FORWARD_MOVE:
      // 前进时，左右电机停止
      stopLeftRightMotor();

      // 前方遇到障碍：停止前后电机，回到横向寻找
      if (getObstacleSignal() == 1) {
        stopForwardBackMotor();

        // 清空大空隙判断，避免刚回横移就立刻再次前进
        gapCount = 0;
        bigGapDetected = false;

        robotState = STATE_WAIT_CLEAR;
      } else {
        moveForward();
      }

      break;

    case STATE_WAIT_CLEAR:
      // 这里不再 stopAllMotors()
      // 只停止前后电机，左右电机继续扫描
      stopForwardBackMotor();

      // 侧边撞到继续反转
      if (getRightBumperSignal() == 1) {
        sideDirection = -1;
      }

      if (getLeftBumperSignal() == 1) {
        sideDirection = 1;
      }

      // 左右继续移动，寻找新的空隙
      moveSideByDirection();

      // 如果重新检测到大空隙，再次切换前进
      if (getForwardSignal() == 1) {
        stopLeftRightMotor();

        stopCount = 0;
        obstacleDetected = false;

        robotState = STATE_FORWARD_MOVE;
      }

      break;
  }
}

// =====================================================
// 调试输出
// =====================================================
void printDebug() {
  unsigned long now = millis();

  if (now - lastPrintTime < PRINT_INTERVAL_MS) {
    return;
  }

  lastPrintTime = now;

  Serial.print("State=");

  if (robotState == STATE_SIDE_MOVE) {
    Serial.print("SIDE");
  } else if (robotState == STATE_FORWARD_MOVE) {
    Serial.print("FORWARD");
  } else {
    Serial.print("WAIT_CLEAR");
  }

  Serial.print(" | Raw=");
  Serial.print(rawVoltage, 3);

  Serial.print(" | GapD=");
  Serial.print(gapDistance, 2);

  Serial.print(" | StopD=");
  Serial.print(stopDistance, 2);

  Serial.print(" | Gap=");
  Serial.print(getForwardSignal());

  Serial.print(" | Obs=");
  Serial.print(getObstacleSignal());

  Serial.print(" | Clear=");
  Serial.print(getFrontClearSignal());

  Serial.print(" | L=");
  Serial.print(getLeftBumperSignal());

  Serial.print(" | R=");
  Serial.print(getRightBumperSignal());

  Serial.print(" | LRDir=");
  Serial.println(sideDirection > 0 ? "RIGHT" : "LEFT");
}

// =====================================================
// setup
// =====================================================
void setup() {
  Serial.begin(9600);

  pinMode(LEFT_BUMPER_PIN, INPUT);
  pinMode(RIGHT_BUMPER_PIN, INPUT);

  pinMode(FB_MOTOR_IN1, OUTPUT);
  pinMode(FB_MOTOR_IN2, OUTPUT);
  pinMode(LR_MOTOR_IN1, OUTPUT);
  pinMode(LR_MOTOR_IN2, OUTPUT);

  stopAllMotors();

  Serial.println("System starting.");

  motorSelfTest();

  Serial.println("System ready.");
}

// =====================================================
// loop
// =====================================================
void loop() {
  updateSensor();
  updateBumpers();
  updateRobotControl();
  printDebug();
}