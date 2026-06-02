#ifdef UNIT_TEST
#include "arduino_mock.h"
#else
#include <Arduino.h>
#endif

#include "config.h"
#include "ir_sensor.h"

float calibVoltage[IR_TABLE_SIZE] = { 4.55f, 3.80f, 3.30f, 2.85f, 2.55f };
float calibDistance[IR_TABLE_SIZE] = { 1.0f, 2.5f, 4.0f, 8.0f, 15.0f };

namespace {
float voltageHistory[IR_HISTORY_SIZE];
int historyIndex = 0;
bool historyFilled = false;

float rawVoltage = 0.0f;
float limitedVoltage = 0.0f;
float gapFilteredVoltage = 0.0f;
float stopFilteredVoltage = 0.0f;
float gapDistance = 15.0f;
float stopDistance = 15.0f;

bool filterReady = false;

unsigned long lastSensorTime = 0;

#ifdef UNIT_TEST
#ifdef __cplusplus
extern "C" {
#endif
// Reset internal state for unit tests
void resetIrSensorForUnitTest();
#ifdef __cplusplus
}
#endif
#endif

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
  float samples[IR_SAMPLE_COUNT];

  for (int i = 0; i < IR_SAMPLE_COUNT; i++) {
    samples[i] = adcToVoltage(analogRead(IR_SENSOR_PIN));
  }

  sortFloatArray(samples, IR_SAMPLE_COUNT);
  return samples[IR_SAMPLE_COUNT / 2];
}

float applyLimiter(float newV, float oldV) {
  float diff = newV - oldV;

  if (diff > IR_MAX_STEP_VOLTAGE) {
    return oldV + IR_MAX_STEP_VOLTAGE;
  }

  if (diff < -IR_MAX_STEP_VOLTAGE) {
    return oldV - IR_MAX_STEP_VOLTAGE;
  }

  return newV;
}

void addHistory(float v) {
  voltageHistory[historyIndex] = v;
  historyIndex++;

  if (historyIndex >= IR_HISTORY_SIZE) {
    historyIndex = 0;
    historyFilled = true;
  }
}

int getHistoryCount() {
  return historyFilled ? IR_HISTORY_SIZE : historyIndex;
}

float getPercentile(float p) {
  int count = getHistoryCount();

  if (count == 0) {
    return limitedVoltage;
  }

  float temp[IR_HISTORY_SIZE];

  for (int i = 0; i < count; i++) {
    temp[i] = voltageHistory[i];
  }

  sortFloatArray(temp, count);

  int idx = (int)((count - 1) * p);
  idx = constrain(idx, 0, count - 1);
  return temp[idx];
}

float voltageToDistance(float v) {
  if (v >= calibVoltage[0]) {
    return calibDistance[0];
  }

  if (v <= calibVoltage[IR_TABLE_SIZE - 1]) {
    return calibDistance[IR_TABLE_SIZE - 1];
  }

  for (int i = 0; i < IR_TABLE_SIZE - 1; i++) {
    float v1 = calibVoltage[i];
    float v2 = calibVoltage[i + 1];

    if (v <= v1 && v >= v2) {
      float d1 = calibDistance[i];
      float d2 = calibDistance[i + 1];
      float ratio = (v - v2) / (v1 - v2);
      return d2 + ratio * (d1 - d2);
    }
  }

  return calibDistance[IR_TABLE_SIZE - 1];
}
}

void setupIrSensor() {
  pinMode(IR_SENSOR_PIN, INPUT);
}

bool updateIrSensor() {
  unsigned long now = millis();
  if (now - lastSensorTime < IR_SENSOR_INTERVAL_MS) {
    return false;
  }

  lastSensorTime = now;

  rawVoltage = readMedianVoltage();

  if (!filterReady) {
    limitedVoltage = rawVoltage;
  } else {
    limitedVoltage = applyLimiter(rawVoltage, limitedVoltage);
  }

  addHistory(limitedVoltage);

  float gapCandidate = getPercentile(0.50f);
  float stopCandidate = getPercentile(0.70f);

  if (!filterReady) {
    gapFilteredVoltage = gapCandidate;
    stopFilteredVoltage = stopCandidate;
    filterReady = true;
  } else {
    gapFilteredVoltage = IR_EMA_ALPHA * gapCandidate + (1.0f - IR_EMA_ALPHA) * gapFilteredVoltage;
    stopFilteredVoltage = IR_EMA_ALPHA * stopCandidate + (1.0f - IR_EMA_ALPHA) * stopFilteredVoltage;
  }

  gapDistance = voltageToDistance(gapFilteredVoltage);
  stopDistance = voltageToDistance(stopFilteredVoltage);

  return true;
}

float getGapDistanceCm() {
  return gapDistance;
}

float getStopDistanceCm() {
  return stopDistance;
}

float getRawVoltage() {
  return rawVoltage;
}

#ifdef UNIT_TEST
// Provide a C-linkage reset function so tests can reinitialize internal state.
extern "C" void resetIrSensorForUnitTest() {
  historyIndex = 0;
  historyFilled = false;

  rawVoltage = 0.0f;
  limitedVoltage = 0.0f;
  gapFilteredVoltage = 0.0f;
  stopFilteredVoltage = 0.0f;
  gapDistance = 15.0f;
  stopDistance = 15.0f;

  filterReady = false;
  lastSensorTime = 0;

  for (int i = 0; i < IR_HISTORY_SIZE; i++) {
    voltageHistory[i] = 0.0f;
  }
}
#endif
