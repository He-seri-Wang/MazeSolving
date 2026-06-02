#ifndef CONFIG_H
#define CONFIG_H

#ifdef UNIT_TEST
#include "arduino_mock.h"
#else
#include <Arduino.h>
#endif

constexpr int MOTOR_SPEED = 50;

constexpr int DIR_A = 12;
constexpr int PWM_A = 3;
constexpr int BRK_A = 9;

constexpr int DIR_B = 13;
constexpr int PWM_B = 11;
constexpr int BRK_B = 8;

constexpr int ENCA1 = 18;
constexpr int ENCA2 = 20;
constexpr int ENCB1 = 19;
constexpr int ENCB2 = 21;
constexpr int ENCC1 = 50;
constexpr int ENCC2 = 51;
constexpr int ENCD1 = 52;
constexpr int ENCD2 = 53;

constexpr int IR_SENSOR_PIN = A2;

constexpr int LEFT_BUMPER_PIN = 4;
constexpr int RIGHT_BUMPER_PIN = 2;

constexpr float WHEEL_DIAMETER_MM = 50.0f;
constexpr int ENCODER_CPR = 360;
constexpr float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / ENCODER_CPR;

constexpr float ADC_REF_VOLTAGE = 5.0f;
constexpr int ADC_MAX = 1023;
constexpr int IR_TABLE_SIZE = 5;

constexpr int IR_SAMPLE_COUNT = 7;
constexpr int IR_HISTORY_SIZE = 9;
constexpr float IR_EMA_ALPHA = 0.3f;
constexpr float IR_MAX_STEP_VOLTAGE = 0.4f;

constexpr unsigned long IR_SENSOR_INTERVAL_MS = 15;
constexpr unsigned long DEBUG_PRINT_INTERVAL_MS = 200;

constexpr float GAP_TARGET_WIDTH_CM = 30.0f;
constexpr float GAP_VERIFY_TRAVEL_CM = GAP_TARGET_WIDTH_CM * 0.5f;
constexpr float GAP_DISTANCE_THRESHOLD_CM = 12.0f;
constexpr float GAP_MIN_SAMPLE_RATIO = 0.7f;
constexpr int GAP_MIN_SAMPLE_COUNT = 8;

constexpr float FRONT_STOP_DISTANCE_CM = 10.0f;

#endif
