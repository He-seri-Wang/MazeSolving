// Define SENSOR1_STANDALONE to build this file's setup/loop.
#ifdef SENSOR1_STANDALONE

#include <math.h>

// =====================================================
// 红外传感器引脚
// =====================================================
const int SENSOR_PIN = A2;
const int SENSOR_PIN2 = A4;

const float ADC_REF_VOLTAGE = 5.0;
const int ADC_MAX = 1023;

// =====================================================
// 侧边撞边按钮
// HIGH = 撞到
// LOW  = 没撞到
// =====================================================
const int LEFT_BUMPER_PIN = 4;
const int RIGHT_BUMPER_PIN = 2;
const int ledPin = 8;
const int ledPinRear = 7;
bool leftBumperHit = false;
bool rightBumperHit = false;
bool sideBumperHit = false;

// =====================================================
// 5 点标定表
// 电压需要按从大到小排列
// 距离需要按从小到大排列
//
// 之后现场标定时，主要改 calibVoltage[]。
// =====================================================
const int TABLE_SIZE = 5;

float calibVoltage[TABLE_SIZE] = {
  4.55,  // 1.0 cm
  3.80,  // 2.5 cm
  3.30,  // 4.0 cm
  2.85,  // 8.0 cm
  2.55   // 15.0 cm
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

// EMA 平滑系数
// 越大响应越快，越小越平滑
const float EMA_ALPHA = 0.3;

// 单次允许最大电压跳变，用于抑制异常尖峰
const float MAX_STEP_VOLTAGE = 0.4;

// =====================================================
// 红外传感器状态变量
// =====================================================
float rawVoltage = 0.0;       // A2 raw 电压
float rawVoltageA4 = 0.0;     // A4 raw 电压

float limitedVoltage = 0.0;
float filteredVoltage = 0.0;
float distanceCm = 0.0;

bool filterReady = false;

// =====================================================
// 前进判断
// =====================================================
const float FORWARD_ALLOW_DISTANCE_CM = 6.0;
const int FORWARD_CONFIRM_COUNT = 6;

int forwardCount = 0;
bool forwardAllowed = false;

// =====================================================
// comb-shaped 墙面处理
// 横向移动时，用一个短窗口记录最小距离，避免 1cm 小空隙误判
// =====================================================
float minDistanceWindow = 999.0;
unsigned long lastWindowReset = 0;
const unsigned long WINDOW_TIME_MS = 120;

// =====================================================
// 非阻塞定时
// =====================================================
unsigned long lastSensorTime = 0;
const unsigned long SENSOR_INTERVAL_MS = 15;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 200;

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

// =====================================================
// ADC 多次采样 + 中值滤波，读取 A2
// =====================================================
float readMedianVoltage() {
  float samples[SAMPLE_COUNT];

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    samples[i] = adcToVoltage(analogRead(SENSOR_PIN));
  }

  sortFloatArray(samples, SAMPLE_COUNT);

  return samples[SAMPLE_COUNT / 2];
}

// =====================================================
// ADC 多次采样 + 中值滤波，读取 A4
// 这里只用于显示 A4 raw 值，不参与距离判断
// =====================================================
float readMedianVoltageA4() {
  float samples[SAMPLE_COUNT];

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    samples[i] = adcToVoltage(analogRead(SENSOR_PIN2));
  }

  sortFloatArray(samples, SAMPLE_COUNT);

  return samples[SAMPLE_COUNT / 2];
}

// =====================================================
// 限幅：抑制单次异常跳变
// =====================================================
float applyLimiter(float newV, float oldV) {
  float diff = newV - oldV;

  if (diff > MAX_STEP_VOLTAGE) {
    return oldV + MAX_STEP_VOLTAGE;
  }

  if (diff < -MAX_STEP_VOLTAGE) {
    return oldV - MAX_STEP_VOLTAGE;
  }

  return newV;
}

// =====================================================
// 历史缓存
// =====================================================
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

// =====================================================
// 分位数滤波
// p = 0.5 为中位数
// p = 0.7 偏向较高电压，即偏向近距离，更保守
// =====================================================
float getPercentile(float p) {
  int count = getHistoryCount();

  if (count == 0) {
    return limitedVoltage;
  }

  float temp[HISTORY_SIZE];

  for (int i = 0; i < count; i++) {
    temp[i] = voltageHistory[i];
  }

  sortFloatArray(temp, count);

  int idx = (int)((count - 1) * p);

  if (idx < 0) {
    idx = 0;
  }

  if (idx >= count) {
    idx = count - 1;
  }

  return temp[idx];
}

// =====================================================
// 查表 + 分段线性插值
// =====================================================
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
// 更新红外传感器
// =====================================================
void updateSensor() {
  unsigned long now = millis();

  if (now - lastSensorTime < SENSOR_INTERVAL_MS) {
    return;
  }

  lastSensorTime = now;

  // 1. 多次采样 + 中值
  // A2 用于原本的距离判断
  rawVoltage = readMedianVoltage();

  // A4 只额外读取 raw 值，用于串口显示
  rawVoltageA4 = readMedianVoltageA4();

  // 2. 限幅
  if (!filterReady) {
    limitedVoltage = rawVoltage;
  } else {
    limitedVoltage = applyLimiter(rawVoltage, limitedVoltage);
  }

  // 3. 历史缓存
  addHistory(limitedVoltage);

  // 4. 分位数滤波
  // 使用 0.7，表示更偏向近距离，避免小空隙误判为可前进
  float selectedVoltage = getPercentile(0.7);

  // 5. EMA 平滑
  if (!filterReady) {
    filteredVoltage = selectedVoltage;
    filterReady = true;
  } else {
    filteredVoltage =
      EMA_ALPHA * selectedVoltage +
      (1.0 - EMA_ALPHA) * filteredVoltage;
  }

  // 6. 电压转距离
  float currentDistance = voltageToDistance(filteredVoltage);

  // 7. comb-shaped 墙处理：短时间窗口取最小距离
  if (currentDistance < minDistanceWindow) {
    minDistanceWindow = currentDistance;
  }

  if (millis() - lastWindowReset > WINDOW_TIME_MS) {
    lastWindowReset = millis();

    if (minDistanceWindow < currentDistance) {
      currentDistance = minDistanceWindow;
    }

    minDistanceWindow = 999.0;
  }

  distanceCm = currentDistance;

  // 8. 是否允许前进
  if (distanceCm >= FORWARD_ALLOW_DISTANCE_CM) {
    forwardCount++;
  } else {
    forwardCount = 0;
  }

  forwardAllowed = (forwardCount >= FORWARD_CONFIRM_COUNT);
}

// =====================================================
// 更新撞边按钮
// =====================================================
void updateBumpers() {
  leftBumperHit = (digitalRead(LEFT_BUMPER_PIN) == HIGH);
  rightBumperHit = (digitalRead(RIGHT_BUMPER_PIN) == HIGH);

  sideBumperHit = leftBumperHit || rightBumperHit;
}

// =====================================================
// 对外调用接口
// =====================================================

// 1 = 可以前进
// 0 = 不建议前进
int getForwardSignal() {
  return forwardAllowed ? 1 : 0;
}

// 1 = 左侧撞到
// 0 = 左侧没撞到
int getLeftBumperSignal() {
  return leftBumperHit ? 1 : 0;
}

// 1 = 右侧撞到
// 0 = 右侧没撞到
int getRightBumperSignal() {
  return rightBumperHit ? 1 : 0;
}

// 1 = 任意一侧撞到
// 0 = 两侧都没撞到
int getSideBumperSignal() {
  return sideBumperHit ? 1 : 0;
}

// 如需读取当前距离，也可以调用这个
float getFrontDistanceCm() {
  return distanceCm;
}

// 如需读取当前滤波电压，也可以调用这个
float getFrontFilteredVoltage() {
  return filteredVoltage;
}

// 如需读取 A4 raw 电压，也可以调用这个
float getA4RawVoltage() {
  return rawVoltageA4;
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

  Serial.print("Raw=");
  Serial.print(rawVoltage, 3);

  Serial.print(" V | Fil=");
  Serial.print(filteredVoltage, 3);

  Serial.print(" V | D=");
  Serial.print(distanceCm, 2);

  Serial.print(" cm | Forward=");
  Serial.print(getForwardSignal());

  Serial.print(" | LHit=");
  Serial.print(getLeftBumperSignal());

  Serial.print(" | RHit=");
  Serial.print(getRightBumperSignal());

  Serial.print(" | SideHit=");
  Serial.println(getSideBumperSignal());

  // 额外输出一行 A4 raw 值
  Serial.print("A4 Raw=");
  Serial.print(rawVoltageA4, 3);
  Serial.println(" V");
}

// =====================================================
// setup
// =====================================================
void setup() {
  Serial.begin(9600);

  pinMode(SENSOR_PIN, INPUT);
  pinMode(SENSOR_PIN2, INPUT);

  pinMode(LEFT_BUMPER_PIN, INPUT);
  pinMode(RIGHT_BUMPER_PIN, INPUT);

  pinMode(ledPin, OUTPUT);
  pinMode(ledPinRear, OUTPUT);
  Serial.println("IR sensor + bumper system ready.");
}

// =====================================================
// loop
// =====================================================
void loop() {
  digitalWrite(ledPin, HIGH);
  digitalWrite(ledPinRear, HIGH);
  updateSensor();
  updateBumpers();
  printDebug();

  // 后续电机主控可以直接调用：
  //
  // int forward = getForwardSignal();
  // int leftHit = getLeftBumperSignal();
  // int rightHit = getRightBumperSignal();
  // int sideHit = getSideBumperSignal();
  //
  // if (sideHit == 1) {
  //   // 撞到侧边
  // }
  //
  // if (forward == 1) {
  //   // 可以向前移动
  // } else {
  //   // 不向前移动
  // }
}

#endif
