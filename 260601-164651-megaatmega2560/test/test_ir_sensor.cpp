// Lightweight native test runner without Unity dependency
#define UNIT_TEST

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include "../src/ir_sensor.cpp"

// Mocks for Arduino functions declared in arduino_mock.h
static int adc_sequence[4096];
static int adc_len = 0;
static int adc_idx = 0;
static unsigned long fake_millis = 0;

int analogRead(int pin) {
  if (adc_idx < adc_len) {
    return adc_sequence[adc_idx++];
  }
  return adc_len > 0 ? adc_sequence[adc_len - 1] : 0;
}

unsigned long millis() {
  return fake_millis;
}

void pinMode(int pin, int mode) {}
int digitalRead(int pin) { return 0; }

static void set_adc_sequence_fill(int value, int repeat) {
  for (int i = 0; i < repeat; i++) {
    adc_sequence[adc_len++] = value;
  }
}

static void reset_mocks() {
  adc_len = 0;
  adc_idx = 0;
  fake_millis = 0;
  resetIrSensorForUnitTest();
}

static int tests_run = 0;
static int tests_failed = 0;

void fail(const std::string &msg) {
  std::cout << "FAIL: " << msg << "\n";
  tests_failed++;
}

void assert_true(bool cond, const std::string &msg) {
  tests_run++;
  if (!cond) fail(msg);
}

void assert_float_within(float tol, float expected, float actual, const std::string &msg) {
  tests_run++;
  if (std::fabs(expected - actual) > tol) {
    fail(msg + " (expected=" + std::to_string(expected) + ", actual=" + std::to_string(actual) + ")");
  }
}

void test_read_median_and_mapping() {
  reset_mocks();

  int adc_val = static_cast<int>(roundf(calibVoltage[0] * ADC_MAX / ADC_REF_VOLTAGE));
  set_adc_sequence_fill(adc_val, IR_SAMPLE_COUNT);

  fake_millis += IR_SENSOR_INTERVAL_MS;
  bool updated = updateIrSensor();
  assert_true(updated, "updateIrSensor should return true");

  float raw = getRawVoltage();
  assert_float_within(0.02f, calibVoltage[0], raw, "raw voltage mismatch");

  float gap = getGapDistanceCm();
  assert_float_within(0.5f, calibDistance[0], gap, "gap distance mapping mismatch");
}

void test_percentile_and_history() {
  reset_mocks();

  int adc_low = static_cast<int>(roundf(calibVoltage[IR_TABLE_SIZE-1] * ADC_MAX / ADC_REF_VOLTAGE));
  int adc_high = static_cast<int>(roundf(calibVoltage[0] * ADC_MAX / ADC_REF_VOLTAGE));

  for (int u = 0; u < IR_HISTORY_SIZE; u++) {
    set_adc_sequence_fill(adc_low, IR_SAMPLE_COUNT - 1);
    set_adc_sequence_fill(adc_high, 1);
    fake_millis += IR_SENSOR_INTERVAL_MS;
    updateIrSensor();
  }

  int adc_mid = static_cast<int>(roundf((calibVoltage[0] + calibVoltage[IR_TABLE_SIZE-1]) * 0.5f * ADC_MAX / ADC_REF_VOLTAGE));
  set_adc_sequence_fill(adc_mid, IR_SAMPLE_COUNT);
  fake_millis += IR_SENSOR_INTERVAL_MS;
  updateIrSensor();

  float gap = getGapDistanceCm();
  bool ok = (gap >= calibDistance[0] - 5.0f && gap <= calibDistance[IR_TABLE_SIZE-1] + 5.0f);
  assert_true(ok, "gap percentile/history out of expected range");
}

int main() {
  std::cout << "Running tests...\n";
  test_read_median_and_mapping();
  test_percentile_and_history();

  std::cout << "Tests run: " << tests_run << ", failed: " << tests_failed << "\n";
  return tests_failed == 0 ? 0 : 1;
}
